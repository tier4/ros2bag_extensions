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

import os
from typing import List

from ros2bag.api import check_path_exists
from ros2bag.verb import VerbExtension
from rosbag2_py import *

from . import (create_reader, get_default_converter_options,
               get_storage_options, get_starting_time)


class MergeVerb(VerbExtension):
    ''' Combine multiple bag files '''
    def _bag2merge(self, input_bags: List[str], output_bag_dir: str, storage_type: str) -> None:
        # Open writer
        storage_options = get_storage_options(output_bag_dir, storage_type)
        converter_options = get_default_converter_options()
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        # NOTE: To make SequentialWriter work properly, read starting_time in order of oldest to newest.
        input_bags = sorted(input_bags, key=lambda bag: get_starting_time(bag, storage_type))

        for input_bag in input_bags:
            reader = create_reader(input_bag, storage_type)

            # Merge
            # NOTE: In the case of SQLite3 storage, it is sorted by ORDERED BY timestamp when reading, so sorting is not necessary when writing.
            # Create topics
            for topic_type in reader.get_all_topics_and_types():
                writer.create_topic(topic_type)

            # Write
            while reader.has_next():
                topic_name, msg, stamp = reader.read_next()
                writer.write(topic_name, msg, stamp)


    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "bag_directory", type=check_path_exists, nargs="+", help="Bag to filter")
        parser.add_argument(
            "-o", "--output", required=True, help="Output directory")
        parser.add_argument(
            "-s", "--storage", required=False, default="sqlite3", help="storage identifier to be used, defaults to 'sqlite3'")


    def main(self, *, args):
        if os.path.isdir(args.output):
            raise FileExistsError("Output folder '{}' already exists.".format(args.output))

        self._bag2merge(args.bag_directory, args.output, args.storage)
