# ros2bag_extensions

Extension commands for rosbag in ROS 2

## Installation

Supported ROS distributions:

- galactic

#### Build

```bash
git clone git@github.com:tier4/ros2bag_extensions.git
cd ros2bag_extensions
rosdep install --from-paths . --ignore-src --rosdistro=galactic
source /opt/ros/galactic/setup.bash
colcon build
```

#### Check installation

```bash
source install/local_setup.bash
ros2 bag --help
```

Make sure that the `filter`, `merge`, and `slice` commands have been added to the list of Commands in the help message.

## Usage

- ros2 bag merge

Merge multiple bag files.

Usage:

```sh
ros2 bag merge -o rosbag2_merged/ rosbag2_2021_08_20-12_28_24/ rosbag2_2021_08_20-12_30_03/
```

- ros2 bag filter

Filter by topic names.

Usage:

```sh
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -t "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"
```

- ros2 bag slice

Save the specified range of data as a bag file by specifying the start time and end time.

Usage:

```sh
# from 1629430104.911167670 to the bag end time
ros2 bag slice input_bag -o sliced_from -s 1629430104.911167670

# from the bag start time to 1629430124
ros2 bag slice input_bag -o sliced_till -e 1629430124

# from 1629430104.911 to 1629430124
ros2 bag slice input_bag -o sliced_between -s 1629430104.911 -e 1629430124
```
