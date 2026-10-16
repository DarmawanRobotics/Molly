"""In-process test of image_throttle_node: 30 fps in, rate_hz out, camera infos all through."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
SingleThreadedExecutor = pytest.importorskip('rclpy.executors').SingleThreadedExecutor
qos_profile_sensor_data = pytest.importorskip('rclpy.qos').qos_profile_sensor_data
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
ImageThrottleNode = pytest.importorskip(
    'molly_perception.image_throttle_node'
).ImageThrottleNode


def test_images_are_thinned_out_and_infos_are_not():
    """One second of a 30 fps camera gives about rate_hz images and every camera info."""
    rclpy.init(args=['--ros-args', '-p', 'rate_hz:=5.0'])
    node = ImageThrottleNode()
    helper = rclpy.create_node('throttle_test_helper')
    images, infos = [], []
    # as apriltag_ros reads them: images best effort, camera_info reliable
    helper.create_subscription(
        sensor_msgs.Image, 'out/image', images.append, qos_profile_sensor_data
    )
    helper.create_subscription(sensor_msgs.CameraInfo, 'out/camera_info', infos.append, 10)
    image_pub = helper.create_publisher(sensor_msgs.Image, 'in/image', 50)
    info_pub = helper.create_publisher(sensor_msgs.CameraInfo, 'in/camera_info', 50)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    try:
        pubs = (image_pub, info_pub, node._image_pub, node._info_pub)
        ready = time.monotonic() + 5.0  # let discovery match every pair first
        while time.monotonic() < ready and not all(p.get_subscription_count() for p in pubs):
            executor.spin_once(timeout_sec=0.05)
        sent = 0
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            image = sensor_msgs.Image(height=2, width=2, encoding='rgb8', step=6, data=[7] * 12)
            image.header.frame_id = 'camera'
            image_pub.publish(image)
            info_pub.publish(sensor_msgs.CameraInfo(width=2, height=2))
            sent += 1
            stop = time.monotonic() + 1 / 30
            while time.monotonic() < stop:
                executor.spin_once(timeout_sec=0.005)
        for _ in range(50):
            executor.spin_once(timeout_sec=0.01)
        assert 3 <= len(images) <= 7, f'{len(images)} of {sent} images forwarded'
        assert len(infos) >= 0.8 * sent
        assert list(images[0].data) == [7] * 12 and images[0].header.frame_id == 'camera'
    finally:
        executor.shutdown()
        helper.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
