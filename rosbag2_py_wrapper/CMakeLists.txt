cmake_minimum_required(VERSION 3.5)
project(rosbag2_py_wrapper)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_writer SHARED
  src/rosbag2_py_wrapper/_writer.cpp
)
ament_target_dependencies(_writer PUBLIC
  "rosbag2_cpp"
)

# Install cython modules as sub-modules of the project
install(
  TARGETS
    _writer
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()
