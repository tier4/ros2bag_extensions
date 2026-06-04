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

import re
from typing import Iterable, List, Optional, Sequence, Set


def _names_matching_patterns(names: Iterable[str], patterns: Sequence[str]) -> Set[str]:
    matched: Set[str] = set()
    for pattern in patterns:
        matched.update(name for name in names if re.fullmatch(pattern, name))
    return matched


def select_topic_metadata(
    all_metadata: Sequence,
    include_topics: Optional[Sequence[str]] = None,
    exclude_topics: Optional[Sequence[str]] = None,
    exclude_types: Optional[Sequence[str]] = None,
) -> List:
    names = [meta.name for meta in all_metadata]
    name_to_type = {meta.name: meta.type for meta in all_metadata}

    if include_topics:
        selected = _names_matching_patterns(names, include_topics)
    elif exclude_topics:
        selected = set(names) - _names_matching_patterns(names, exclude_topics)
    else:
        selected = set(names)

    if exclude_types:
        excluded_types = set(exclude_types)
        selected = {name for name in selected if name_to_type[name] not in excluded_types}

    return [meta for meta in all_metadata if meta.name in selected]
