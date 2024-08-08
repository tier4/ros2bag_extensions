from ros2bag_extensions.verb import rcl_py_time_to_datetime
from rclpy.time import Time
from datetime import datetime

def test_rcl_py_time_to_datetime() -> None:
    ros_time = Time(seconds=1234567890, nanoseconds=987654321)
    datetime_time = rcl_py_time_to_datetime(ros_time)
    assert datetime_time == datetime.fromtimestamp(1234567890.987654321)
