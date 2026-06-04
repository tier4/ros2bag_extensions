# ros2bag_extensions

ROS 2 の rosbag 向け拡張コマンド

[English](README.md)

## インストール

対応 ROS ディストリビューション:

- humble

### オプション

`mcap` 形式の bag を扱う場合は、事前に mcap 用パッケージをインストールしてください。

```shell
sudo apt install ros-humble-rosbag2-storage-mcap
```

### ビルド

```bash
# 拡張用ワークスペースを作成
mkdir -p $HOME/extension_ws/src
# 拡張パッケージを clone
cd $HOME/extension_ws/src
git clone git@github.com:tier4/ros2bag_extensions.git
# ワークスペースをビルド
cd $HOME/extension_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro=${ROS_DISTRO}
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

### インストール確認

```bash
source install/local_setup.bash
ros2 bag --help
```

ヘルプの Commands 一覧に `filter`、`merge`、`slice` が追加されていることを確認してください。

## 使い方

### ros2 bag merge

複数の bag ファイルを結合します。

使用例:

```sh
# sqlite3 形式の bag
ros2 bag merge -o rosbag2_merged/ rosbag2_2021_08_20-12_28_24/ rosbag2_2021_08_20-12_30_03/

# mcap 形式の bag
ros2 bag merge -o rosbag2_merged/ rosbag2_2021_08_20-12_28_24/ rosbag2_2021_08_20-12_30_03/ -s mcap
```

### ros2 bag filter

選択したトピックだけを含む新しい bag を作成します。**トピック名**（正規表現）および／または**メッセージ型**（完全一致）でフィルタできます。

#### オプション

| オプション | 説明 |
|-----------|------|
| `bag_directory` | 入力 bag のディレクトリ |
| `-o`, `--output` | 出力 bag のディレクトリ（`--dry-run` 時は不要） |
| `-i`, `--include` | 残すトピック名（正規表現、`re.fullmatch`） |
| `-x`, `--exclude` | 除外するトピック名（正規表現、`re.fullmatch`） |
| `-X`, `--exclude-type` | 除外するメッセージ型（完全一致、複数指定可） |
| `-n`, `--dry-run` | 選択結果を表示のみ（bag は書き出さない、`-o` 不要） |
| `-s`, `--storage` | ストレージプラグイン（既定は `sqlite3`、`mcap` も可） |

補足:

- `-i` と `-x` は同時に指定できません。
- `-X` は `-i` または `-x` と併用できます。
- `-i` も `-x` も指定しない場合、bag 内の全トピックを対象にしたうえで `-X` が適用されます。
- トピック名のパターンは [Python の正規表現](https://docs.python.org/3.8/library/re.html)（`re.fullmatch`）です。例: `/sensing/.*` は `/sensing/` 配下のトピックに一致します。
- `--dry-run` 時の出力形式は `<トピック名> <メッセージ型>`（1 行 1 トピック）です。

#### フィルタの適用順

1. トピック名に `-i` または `-x` を適用（指定がある場合）
2. メッセージ型に `-X` を適用（指定がある場合）
3. 出力 bag を書き出す、または `--dry-run` の場合は一覧を表示

#### 使用例

トピック名で残す（include）:

```sh
# sqlite3 形式の bag
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"

# mcap 形式の bag
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/system/emergency/turn_signal_cmd" "/autoware/driving_capability" -s mcap

# 正規表現
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/sensing/.*" "/vehicle/.*"
```

トピック名で除外する（exclude）:

```sh
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/system/emergency/turn_signal_cmd" "/autoware/driving_capability"
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -x "/sensing/.*" "/vehicle/.*"
```

メッセージ型で除外する:

```sh
# 型のみで除外
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -X sensor_msgs/msg/Image sensor_msgs/msg/PointCloud2

# トピック名で残したあと、型で除外
ros2 bag filter -o rosbag2_filtered/ rosbag2_merged/ -i "/sensing/.*" -X sensor_msgs/msg/Image -s mcap
```

ドライラン（選択結果の確認のみ）:

```sh
ros2 bag filter rosbag2_merged/ -i "/sensing/.*" -X sensor_msgs/msg/Image -n
ros2 bag filter rosbag2_merged/ -X sensor_msgs/msg/Image -n -s mcap
```

### ros2 bag slice

次の 2 つの機能があります。

#### 開始・終了時刻で切り出す

開始時刻と終了時刻を指定し、該当範囲のデータだけを bag として保存します。

使用例:

```sh
# sqlite3 形式、1629430104.911167670 から bag 終了時刻まで
ros2 bag slice input_bag -o sliced_from -b 1629430104.911167670

# mcap 形式、1629430104.911167670 から bag 終了時刻まで
ros2 bag slice input_bag -o sliced_from -b 1629430104.911167670 -s mcap

# bag 開始時刻から 1629430124 まで
ros2 bag slice input_bag -o sliced_till -e 1629430124

# 1629430104.911 から 1629430124 まで
ros2 bag slice input_bag -o sliced_between -b 1629430104.911 -e 1629430124
```

#### 複数ファイルに分割する

指定した秒数ごとに bag を複数ファイルに分割します。

使用例:

```sh
# input_bag を 60 秒ごとの bag に分割
ros2 bag slice input_bag -o sliced_bags -d 60
```
