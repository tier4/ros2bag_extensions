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
from typing import List, Optional, Sequence

from ros2bag.api import check_path_exists
from ros2bag.verb import VerbExtension
from rosbag2_py import *

from . import create_reader, get_default_converter_options, get_storage_options
from ..filter_selection import select_topic_metadata


class FilterVerb(VerbExtension):
    '''Filter bag topics by name and/or message type.'''

    def _write_filtered_bag(
        self,
        reader: SequentialReader,
        output_bag_dir: str,
        topic_metadata: Sequence,
        storage_type: str,
    ) -> None:
        topic_names = [meta.name for meta in topic_metadata]
        reader.set_filter(StorageFilter(topics=topic_names))

        storage_options = get_storage_options(output_bag_dir, storage_type)
        converter_options = get_default_converter_options()
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        for meta in topic_metadata:
            writer.create_topic(meta)

        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            writer.write(topic_name, msg, stamp)
        del writer

        Reindexer().reindex(storage_options)

    def _bag2filter(
        self,
        input_bag_dir: str,
        output_bag_dir: Optional[str],
        include_topics: Optional[List[str]],
        exclude_topics: Optional[List[str]],
        exclude_types: Optional[List[str]],
        dry_run: bool,
        storage_type: str,
    ) -> None:
        reader = create_reader(input_bag_dir, storage_type)
        all_metadata = reader.get_all_topics_and_types()
        selected = select_topic_metadata(
            all_metadata, include_topics, exclude_topics, exclude_types)

        if dry_run:
            for meta in selected:
                print(f'{meta.name} {meta.type}')
            return

        self._write_filtered_bag(reader, output_bag_dir, selected, storage_type)

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'bag_directory', type=check_path_exists, help='Bag to filter')
        parser.add_argument(
            '-o', '--output', required=False, help='Output directory (required unless --dry-run)')
        group = parser.add_mutually_exclusive_group(required=False)
        group.add_argument('-i', '--include', nargs='+', help='Topics to include.')
        group.add_argument('-x', '--exclude', nargs='+', help='Topics to exclude.')
        parser.add_argument(
            '-X', '--exclude-type', nargs='+', dest='exclude_type',
            help='Message types to exclude.')
        parser.add_argument(
            '-n', '--dry-run', action='store_true',
            help='List selected topics without writing an output bag.')
        parser.add_argument(
            '-s', '--storage', required=False, default='sqlite3',
            help="storage identifier to be used, defaults to 'sqlite3'")

    def main(self, *, args):
        if not args.dry_run:
            if args.output is None:
                raise RuntimeError('--output is required unless --dry-run is specified.')
            if os.path.isdir(args.output):
                raise FileExistsError(
                    "Output folder '{}' already exists.".format(args.output))

        self._bag2filter(
            args.bag_directory,
            args.output,
            args.include,
            args.exclude,
            args.exclude_type,
            args.dry_run,
            args.storage,
        )
