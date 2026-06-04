# ros2bag_extensions

Extension commands for rosbag in ROS 2

[日本語](README_ja.md)

## Installation

Supported ROS distributions:

- humble

### Optional

If you want to use `mcap` format bags, you need to install the mcap package in advance.

```shell
sudo apt install ros-humble-rosbag2-storage-mcap
```

### Build

```bash
# create workspace for extension
mkdir -p $HOME/extension_ws/src
# clone extension package
cd $HOME/extension_ws/src
git clone git@github.com:tier4/ros2bag_extensions.git
# build workspace
cd $HOME/extension_ws
source /opt/ros/humble/setup.bash
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

### ros2 bag merge

Merge multiple bag files.

Usage:

```sh
# sqlite3 format bag
ros2 bag merge -o rosbag2_merged/ rosbag2_2021_08_20-12_28_24/ rosbag2_2021_08_20-12_30_03/

# mcap format bag
ros2 bag merge -o rosbag2_merged/ rosbag2_2021_08_20-12_28_24/ rosbag2_2021_08_20-12_30_03/ -s mcap
```

### ros2 bag filter

Create a new bag that contains only selected topics. Topics can be filtered by **topic name** (regular expression) and/or **message type** (exact match).

#### Options

| Option | Description |
|--------|-------------|
| `bag_directory` | Input bag directory |
| `-o`, `--output` | Output bag directory (required unless `--dry-run`) |
| `-i`, `--include` | Topic names to keep (regex, `re.fullmatch`) |
| `-x`, `--exclude` | Topic names to remove (regex, `re.fullmatch`) |
| `-X`, `--exclude-type` | Message types to remove (exact match, multiple allowed) |
| `-n`, `--dry-run` | Print selected topics without writing a bag (`-o` not required) |
| `-s`, `--storage` | Storage plugin (`sqlite3` by default, or `mcap`) |

Notes:

- `-i` and `-x` are mutually exclusive.
- `-X` can be combined with `-i` or `-x`.
- If neither `-i` nor `-x` is given, all topics in the bag are candidates before `-X` is applied.
- Topic name patterns use [Python regular expressions](https://docs.python.org/3.8/library/re.html) (`re.fullmatch`). For example, `/sensing/.*` matches topics under `/sensing/`.
- With `--dry-run`, each line is printed as: `<topic_name> <message_type>`.

#### Filter order

1. Apply `-i` or `-x` on topic names (if specified).
2. Apply `-X` on message types (if specified).
3. Write the output bag, or print the list when `--dry-run` is set.

#### Examples

Include topics by name:

```sh
# sqlite3 format bag
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"

# mcap format bag
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/system/emergency/turn_signal_cmd" "/autoware/driving_capability" -s mcap

# regular expression
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/sensing/.*" "/vehicle/.*"
```

Exclude topics by name:

```sh
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/sensing/.*" "/vehicle/.*"
```

Exclude topics by message type:

```sh
# exclude by type only
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -X sensor_msgs/msg/Image sensor_msgs/msg/PointCloud2

# include by topic name, then exclude by type
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/sensing/.*" -X sensor_msgs/msg/Image -s mcap
```

Dry-run (preview selected topics):

```sh
ros2 bag filter rosbag2_merged/ -i "/sensing/.*" -X sensor_msgs/msg/Image -n
ros2 bag filter rosbag2_merged/ -X sensor_msgs/msg/Image -n -s mcap
```

### ros2 bag slice

This verb has two functions

#### extract with start and end time

Save the specified range of data as a bag file by specifying the start time and end time.

Usage:

```sh
# sqlite3 format, from 1629430104.911167670 to the bag end time
ros2 bag slice input_bag -o sliced_from -b 1629430104.911167670

# mcap format, from 1629430104.911167670 to the bag end time
ros2 bag slice input_bag -o sliced_from -b 1629430104.911167670 -s mcap

# from the bag beginning time to 1629430124
ros2 bag slice input_bag -o sliced_till -e 1629430124

# from 1629430104.911 to 1629430124
ros2 bag slice input_bag -o sliced_between -b 1629430104.911 -e 1629430124
```

#### split into multiple files

Split bag file into multiple files with specified duration second.

Usage:

```sh
# split input_bag into sliced bags which has 60secs
ros2 bag slice input_bag -o sliced_bags -d 60
```
