# Time synchronisation

Every stamp of the stack must come from one clock: the Mid-360's points and IMU, the camera, the
odometry, TF and the times the dashboard shows. Two clocks a few hundred milliseconds apart
already make TF "too old" (the robot stops, `TF_STALE`), make the odometry jump, and make the
AprilTag corrections fail their age check.

| Clock               | Kept by                                                                                                                                         | Check                                                                                |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| Molly's computer    | chrony (`scripts/setup.sh install timesync`): follows `TIME_SERVERS`, serves `TIME_ALLOW`                                                       | `scripts/timesync.sh check`, `chronyc tracking`; health `clock:ntp`                  |
| Mid-360             | PTP (`scripts/setup.sh install ptp`): `ptp4l` as master on the LiDAR's interface (the route to `LIVOX_IP`, or `PTP_IFACE`), service `lidar-ptp` | `systemctl status lidar-ptp`; no "in the future" or "steady" issue on `/livox/lidar` |
| Lite3 motion host   | its own; `lite3_ros2` stamps its messages on arrival                                                                                            |                                                                                      |
| Front camera        | `lite3_camera` stamps each frame on arrival minus a nominal stream latency (host clock)                                                         | `/camera/camera/color/camera_info` without a stamp issue in health                   |
| Dashboard (browser) | the device's clock; the times it shows come from the robot                                                                                      |                                                                                      |

## Set it up

Once per robot, in `/etc/molly/molly.env`:

```bash
TIME_SERVERS=        # an NTP server on the venue's network; empty: Ubuntu's pool
TIME_ALLOW=          # subnets of other computers on the robot that sync to this one
LIVOX_IP=            # the Mid-360's address; empty: no PTP
PTP_IFACE=           # empty: the interface that routes to LIVOX_IP
```

```bash
scripts/setup.sh install timesync ptp   # both are in the default install as well
scripts/timesync.sh check
```

```text
chrony: offset 0.000231 s to C0A80101 (192.168.1.1)
ptp: PTP master on eth1 (software timestamps)
```

The offset must stay below 0.05 s (`TIME_MAX_OFFSET`); with a time server on the same network it
is below a millisecond. Without internet, chrony keeps serving the robot (`local stratum 10`), so
everything on the robot stays on one clock even when that clock drifts from the world's.

At boot `scripts/run_robot.sh` waits up to `TIME_SYNC_WAIT` (60 s) for the first
synchronisation, so the clock does not jump under a running stack. On the robot, health watches
the offset (`clock.enabled` in [stack.yaml](../src/molly_bringup/config/stack.yaml); off in the
simulator).

## When it is wrong

| Symptom                                                                   | Check                                                                                                                                                                       |
| ------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Health: `/livox/lidar` stamps "in the future", or `steady (clock offset)` | the Mid-360 is on its own clock: `systemctl status lidar-ptp`, `LIVOX_IP`, the cable's interface                                                                            |
| Health: `clock:ntp` not synchronised or offset too large                  | `chronyc sources -v`: is a time source reachable? `TIME_SERVERS`                                                                                                            |
| Robot stops with `TF_STALE` although nothing is slow                      | the two above; then [Dharma's TF latency diagnosis](https://github.com/DarmawanRobotics/Dharma/blob/main/docs/guide/Troubleshooting.md#tf-latency-diagnosis-in-ten-minutes) |

`scripts/timesync.sh` comes from the Dharma template; the details are in
[Dharma: time synchronisation](https://github.com/DarmawanRobotics/Dharma/blob/main/docs/guide/Troubleshooting.md#time-synchronisation).
