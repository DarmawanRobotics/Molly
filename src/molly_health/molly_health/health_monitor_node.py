#!/usr/bin/env python3
import json
import os
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from molly_health.checks import (
    cpu_temperature,
    CpuUsage,
    disk_free_gb,
    gpu_load_path,
    gpu_percent,
    memory_kb,
    missing_nodes,
    overall_level,
    parse_topic_specs,
    RateTracker,
    state_issues,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String


class HealthMonitorNode(Node):
    """Watches sensor topic rates and the computer, publishes diagnostics and a health summary."""

    def __init__(self):
        """Declare params and start the checks."""
        super().__init__('health_monitor_node')
        defaults = {
            'topics': [
                '/livox/lidar:5.0',
                '/perception/sector_scan:5.0',
                '/camera/camera/color/camera_info:5.0',
                '/{robot_ns}/odom:10.0',
            ],
            'robot_ns': 'robot',
            'max_cpu_temp_c': 85.0,
            'max_memory_percent': 92.0,
            'min_disk_free_gb': 5.0,
            'disk_path': '~',
            'publish_hz': 1.0,
            'expected_nodes': [''],
            'startup_grace_sec': 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._expected = parse_topic_specs(p('topics'), str(p('robot_ns')))
        self._trackers = {topic: RateTracker() for topic in self._expected}
        self._subscribed: set[str] = set()
        self._limits = (
            float(p('max_cpu_temp_c')),
            float(p('max_memory_percent')),
            float(p('min_disk_free_gb')),
        )
        self._disk_path = p('disk_path')
        self._expected_nodes = [n for n in p('expected_nodes') if n]
        self._grace_until = time.monotonic() + float(p('startup_grace_sec'))
        self._states = {'odometry': '', 'localization': '', 'field': '', 'estop': False}
        self._cpu = CpuUsage()
        self._gpu_path = gpu_load_path()

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'molly/diagnostics', 10)
        self._status_pub = self.create_publisher(String, 'health/status', latched)
        for topic, key in (
            ('odometry/status', 'odometry'),
            ('localization/status', 'localization'),
            ('bridge/protective_field', 'field'),
        ):
            self.create_subscription(
                String, topic, lambda m, k=key: self._states.update({k: m.data}), latched
            )
        self.create_subscription(
            Bool, 'bridge/estop_engaged', lambda m: self._states.update(estop=m.data), latched
        )
        self.create_timer(5.0, self.discover)
        self.create_timer(1.0 / float(p('publish_hz')), self.publish)
        self.discover()

    def discover(self):
        """Subscribe (raw, no deserialisation) to watched topics as they appear."""
        types = dict(self.get_topic_names_and_types())
        for topic in self._expected:
            if topic in self._subscribed or topic not in types:
                continue
            try:
                msg_type = get_message(types[topic][0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            tracker = self._trackers[topic]
            self.create_subscription(
                msg_type,
                topic,
                lambda _m, t=tracker: t.tick(time.monotonic()),
                qos_profile_sensor_data,
                raw=True,
            )
            self._subscribed.add(topic)

    def publish(self):
        """Evaluate every check and publish diagnostics plus a JSON summary."""
        now = time.monotonic()
        issues, statuses, rates = [], [], {}
        for topic, min_hz in self._expected.items():
            hz = self._trackers[topic].rate(now)
            rates[topic] = round(hz, 1)
            ok = hz >= min_hz * 0.8
            if not ok:
                issues.append(f'{topic} {hz:.1f}/{min_hz:.0f} Hz')
            statuses.append(self.status(f'topic {topic}', ok, f'{hz:.1f} Hz', {'min_hz': min_hz}))

        max_temp, max_mem, min_disk = self._limits
        temp, disk = cpu_temperature(), disk_free_gb(self._disk_path)
        memory = memory_kb()
        mem = 100.0 * memory[0] / memory[1] if memory and memory[1] else None
        load = os.getloadavg()[0]
        system = {
            'cpu_temp_c': temp,
            'memory_percent': mem,
            'disk_free_gb': disk,
            'load': load,
            'cpu_percent': self._cpu.percent(),
            'gpu_percent': gpu_percent(self._gpu_path),
            'memory_used_gb': memory[0] / 1048576.0 if memory else None,
            'memory_total_gb': memory[1] / 1048576.0 if memory else None,
        }
        for label, value, bad in (
            ('cpu temperature', temp, temp is not None and temp > max_temp),
            ('memory', mem, mem is not None and mem > max_mem),
            ('disk', disk, disk is not None and disk < min_disk),
        ):
            if bad:
                issues.append(f'{label} {value:.0f}')
            statuses.append(self.status(f'system {label}', not bad, f'{value}', {}))

        critical = state_issues(
            self._states['odometry'], self._states['field'], bool(self._states['estop'])
        )
        if self._expected_nodes and now > self._grace_until:
            names = [n for n, _ in self.get_node_names_and_namespaces()]
            for name in missing_nodes(self._expected_nodes, names):
                critical.append(f'node {name} not running')
        for issue in critical:
            statuses.append(self.status(issue, False, issue, {}, error=True))
        issues = critical + issues

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._diag_pub.publish(array)
        summary = {
            'ok': not issues,
            'level': overall_level(issues, critical),
            'issues': issues,
            'topics': rates,
            'odometry': self._states['odometry'] or None,
            'localization': self._states['localization'] or None,
            'field': self._states['field'] or None,
            'estop': bool(self._states['estop']),
            **{k: (round(v, 1) if v is not None else None) for k, v in system.items()},
        }
        self._status_pub.publish(String(data=json.dumps(summary)))

    @staticmethod
    def status(
        name: str, ok: bool, message: str, values: dict, error: bool = False
    ) -> DiagnosticStatus:
        """Return a DiagnosticStatus (WARN when not ok, ERROR for critical issues)."""
        s = DiagnosticStatus()
        s.name = f'molly: {name}'
        if ok:
            s.level = DiagnosticStatus.OK
        else:
            s.level = DiagnosticStatus.ERROR if error else DiagnosticStatus.WARN
        s.message = message
        s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return s


def main(args=None):
    """Spin the health monitor."""
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
