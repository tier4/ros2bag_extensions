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

from . import create_reader, get_default_converter_options, get_default_storage_options


class FilterVerb(VerbExtension):
    ''' Filter by topic names '''
    def _bag2filter(self, input_bag_dir: str, output_bag_dir: str, include_topics: List[str], exclude_topics: List[str]) -> None:
        reader = create_reader(input_bag_dir)

        # Filter topics
        if include_topics:
            topic_list = [topic.name for topic in reader.get_all_topics_and_types() if topic.name in include_topics]
        elif exclude_topics:
            topic_list = [topic.name for topic in reader.get_all_topics_and_types() if topic.name not in exclude_topics]
        else:
            topic_list = [topic.name for topic in reader.get_all_topics_and_types()]

        topic_filter = StorageFilter(topics=topic_list)
        reader.set_filter(topic_filter)

        # Open writer
        storage_options = get_default_storage_options(output_bag_dir)
        converter_options = get_default_converter_options()
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        # Create topics
        for topic_type in reader.get_all_topics_and_types():
            writer.create_topic(topic_type)

        # Write
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            writer.write(topic_name, msg, stamp)


    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "bag_directory", type=check_path_exists, help="Bag to filter")
        parser.add_argument(
            "-o", "--output", required=True, help="Output directory")
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument("-i", "--include", nargs="+", help="Topics to include.")
        group.add_argument("-x", "--exclude", nargs="+", help="Topics to exclude.")


    def main(self, *, args):
        if os.path.isdir(args.output):
            raise FileExistsError("Output folder '{}' already exists.".format(args.output))

        self._bag2filter(args.bag_directory, args.output, args.include, args.exclude)
