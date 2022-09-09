# ros2bag_extensions

Extension commands for rosbag in ROS 2

## Installation

Supported ROS distributions:

- galactic
- humble

### Build

```bash
# create workspace for extension
mkdir -p $HOME/extension_ws/src
# clone extension package
cd $HOME/extension_ws/src
git clone git@github.com:tier4/ros2bag_extensions.git
# build workspace
cd $HOME/extension_ws
source /opt/ros/galactic/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro=${ROS_DISTRO}
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

### Check installation

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

Filter by topic names. You can use [python3 regular expression operations](https://docs.python.org/3.8/library/re.html).

Usage:

If you want to include the specified topic, use `-i` or `--include`.

```sh
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"
# use regular expression
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/sensing/.*" "/vehicle/.*"
```

If you want to exclude the specified topic, use `-x` or `--exclude`.

```sh
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"
# use regular expression
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/sensing/.*" "/vehicle/.*"
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
