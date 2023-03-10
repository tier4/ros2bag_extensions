// Copyright 2023 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "./pybind11.hpp"

namespace rosbag2_py_wrapper
{

class SequentialWriterWrapper : public rosbag2_cpp::writers::SequentialWriter
{
public:
  void split_bagfile_wrapper() {
    this->split_bagfile();
  }

  /// Write a serialized message to a bag file
  void write(
    const std::string & topic_name, const std::string & message,
    const rcutils_time_point_value_t & time_stamp)
  {
    auto bag_message =
      std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->topic_name = topic_name;
    bag_message->serialized_data =
      rosbag2_storage::make_serialized_message(message.c_str(), message.length());
    bag_message->time_stamp = time_stamp;

    rosbag2_cpp::writers::SequentialWriter::write(bag_message);
  }
};

std::unordered_set<std::string> get_registered_writers()
{
  rosbag2_storage::StorageFactory storage_factory;
  const auto read_write = storage_factory.get_declared_read_write_plugins();
  return std::unordered_set<std::string>(read_write.begin(), read_write.end());
}

}  // namespace rosbag2_py_wrapper

PYBIND11_MODULE(_writer, m) {
  m.doc() = "Python wrapper of the SequentialWriter API";

  pybind11::class_<rosbag2_py_wrapper::SequentialWriterWrapper>(
    m, "SequentialWriterWrapper")
  .def(pybind11::init())
  .def("open", &rosbag2_py_wrapper::SequentialWriterWrapper::open)
  .def("write", &rosbag2_py_wrapper::SequentialWriterWrapper::write)
  .def("remove_topic", &rosbag2_py_wrapper::SequentialWriterWrapper::remove_topic)
  .def("create_topic", &rosbag2_py_wrapper::SequentialWriterWrapper::create_topic)
  .def("split_bagfile", &rosbag2_py_wrapper::SequentialWriterWrapper::split_bagfile_wrapper);

  m.def(
    "get_registered_writers",
    &rosbag2_py_wrapper::get_registered_writers,
    "Returns list of discovered plugins that support rosbag2 recording");
}
