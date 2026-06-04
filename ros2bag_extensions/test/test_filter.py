from ros2bag_extensions.filter_selection import select_topic_metadata


class _TopicMeta:
    def __init__(self, name: str, type_: str) -> None:
        self.name = name
        self.type = type_


def _meta(*entries: tuple[str, str]) -> list[_TopicMeta]:
    return [_TopicMeta(name, type_) for name, type_ in entries]


def test_select_all_when_no_filters() -> None:
    all_meta = _meta(('/a', 'pkg/msg/A'), ('/b', 'pkg/msg/B'))
    assert [m.name for m in select_topic_metadata(all_meta)] == ['/a', '/b']


def test_select_include_by_topic_pattern() -> None:
    all_meta = _meta(('/sensing/a', 'pkg/msg/A'), ('/other/b', 'pkg/msg/B'))
    selected = select_topic_metadata(all_meta, include_topics=['/sensing/.*'])
    assert [m.name for m in selected] == ['/sensing/a']


def test_select_exclude_by_topic_pattern() -> None:
    all_meta = _meta(('/sensing/a', 'pkg/msg/A'), ('/other/b', 'pkg/msg/B'))
    selected = select_topic_metadata(all_meta, exclude_topics=['/sensing/.*'])
    assert [m.name for m in selected] == ['/other/b']


def test_select_exclude_by_type() -> None:
    all_meta = _meta(
        ('/a', 'sensor_msgs/msg/Image'),
        ('/b', 'sensor_msgs/msg/PointCloud2'),
        ('/c', 'std_msgs/msg/String'),
    )
    selected = select_topic_metadata(
        all_meta, exclude_types=['sensor_msgs/msg/Image'])
    assert [m.name for m in selected] == ['/b', '/c']


def test_select_include_then_exclude_type() -> None:
    all_meta = _meta(
        ('/sensing/a', 'sensor_msgs/msg/Image'),
        ('/sensing/b', 'sensor_msgs/msg/PointCloud2'),
        ('/other/c', 'sensor_msgs/msg/Image'),
    )
    selected = select_topic_metadata(
        all_meta,
        include_topics=['/sensing/.*'],
        exclude_types=['sensor_msgs/msg/Image'],
    )
    assert [m.name for m in selected] == ['/sensing/b']


def test_select_preserves_metadata_order() -> None:
    all_meta = _meta(('/z', 'pkg/msg/Z'), ('/a', 'pkg/msg/A'), ('/m', 'pkg/msg/M'))
    selected = select_topic_metadata(all_meta, exclude_types=['pkg/msg/M'])
    assert [m.name for m in selected] == ['/z', '/a']
