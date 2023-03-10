# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
import os

from ros2bag.api import check_path_exists
from ros2bag.verb import VerbExtension
from rosbag2_py import *
from rosbag2_py_wrapper import SequentialWriterWrapper

from . import create_reader, get_default_converter_options, get_default_storage_options


class SliceVerb(VerbExtension):
    ''' Save the specified range of data as a bag file by specifying the start time and end time. '''
    def _bag2slice_with_start_end_time(self, input_bag_dir: str, output_bag_dir: str, start_time: datetime.datetime, end_time: datetime.datetime) -> None:
        # Check timestamp
        metadata = Info().read_metadata(input_bag_dir, "sqlite3")

        if start_time < metadata.starting_time:
            print("No valid start time set. Start time automatically set the bag start time.")
            start_time = metadata.starting_time

        if end_time > metadata.starting_time + metadata.duration:
            print("No valid end time set. End time automatically set the bag end time.")
            end_time = metadata.starting_time + metadata.duration

        # Open writer
        storage_options = get_default_storage_options(output_bag_dir)
        converter_options = get_default_converter_options()
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        # Create topics
        reader = create_reader(input_bag_dir)
        for topic_type in reader.get_all_topics_and_types():
            writer.create_topic(topic_type)

        # Write
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            datetime_stamp = datetime.datetime.fromtimestamp(stamp / 1e9)
            if start_time <= datetime_stamp <= end_time:
                writer.write(topic_name, msg, stamp)

    ''' Split and save bag files by specifying the duration. '''
    def _bag2slice_with_duration(self, input_bag_dir: str, output_bag_dir: str, duration: float) -> None:
        # Open writer
        storage_options = get_default_storage_options(output_bag_dir)
        converter_options = get_default_converter_options()
        writer = SequentialWriterWrapper()
        writer.open(storage_options, converter_options)

        # Create topics
        reader = create_reader(input_bag_dir)
        for topic_type in reader.get_all_topics_and_types():
            writer.create_topic(topic_type)

        # Write
        split_timestamp = None
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            if split_timestamp is None:
                split_timestamp = stamp
            writer.write(topic_name, msg, stamp)
            time_diff = (stamp - split_timestamp) / 1e9
            if time_diff > duration:
                writer.split_bagfile()
                split_timestamp = stamp




    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "bag_directory", type=check_path_exists, help="Bag to filter")
        parser.add_argument(
            "-o", "--output", required=True, help="Output directory")
        parser.add_argument(
            "-s", "--start-time", type=float, help="Start time in nanoseconds")
        parser.add_argument(
            "-e", "--end-time", type=float, help="End time in nanoseconds") # 2100/01/01 00:00:00
        parser.add_argument(
            "-d", "--duration", type=float, help="duration second for slice")

    def main(self, *, args):
        if os.path.isdir(args.output):
            raise FileExistsError("Output folder '{}' already exists.".format(args.output))

        # start_time and end_time exists
        if args.start_time is not None and args.end_time is not None:
            dt_start_time = datetime.datetime.fromtimestamp(args.start_time)
            dt_end_time = datetime.datetime.fromtimestamp(args.end_time)

            self._bag2slice_with_start_end_time(args.bag_directory, args.output, dt_start_time, dt_end_time)
        elif args.duration is not None:
            self._bag2slice_with_duration(args.bag_directory, args.output, args.duration)
        else:
            print("please specify start and end time or duration")
