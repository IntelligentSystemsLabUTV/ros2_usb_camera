cmake_minimum_required(VERSION 3.8)
project(usb_camera_driver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Build options
option(NO_CUDA "Disables CUDA code" OFF)
option(VPI "Enables Nvidia VPI API code" OFF)

# If we're on a Jetson device with L4T 35, some paths and options must be configured to find stubs
# that the Nvidia container runtime will then mount when an executable is loaded
# NOTE: The condition we check is a bit of a hack and might stop working, but for now it works
if(EXISTS "/etc/nv_tegra_release" AND EXISTS "/usr/lib/aarch64-linux-gnu/tegra")
  file(STRINGS "/etc/nv_tegra_release" FILE_CONTENTS LIMIT_COUNT 1)
  string(FIND "${FILE_CONTENTS}" "# R35" MATCH_INDEX)
  if(${MATCH_INDEX} GREATER -1)
    message(STATUS "Jetson R35 platform detected, configuring build to check for CUDA stubs")
    list(APPEND CMAKE_LIBRARY_PATH "/usr/local/cuda/lib64/stubs_")
    list(APPEND CMAKE_CXX_FLAGS "-Wl,--allow-shlib-undefined")
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
if(NOT NO_CUDA)
  find_package(CUDA QUIET)
endif()
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(params_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(theora_wrappers REQUIRED)
find_package(Threads REQUIRED)
if(VPI)
  find_package(vpi QUIET)
endif()

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/usb_camera_driver/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure USB Camera Driver node source files
file(GLOB USB_CAMERA_DRIVER_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/usb_camera_driver/*.cpp")
list(APPEND USB_CAMERA_DRIVER_NODE_SRCS init_parameters.cpp)

# USB Camera Driver node
add_library(usb_camera_driver SHARED ${USB_CAMERA_DRIVER_NODE_SRCS})
target_compile_definitions(usb_camera_driver PRIVATE COMPOSITION_BUILDING_DLL)
if(vpi_FOUND)
  message(STATUS "Compiling code with VPI API")
  target_compile_definitions(usb_camera_driver PUBLIC WITH_VPI)
  target_link_libraries(usb_camera_driver
    vpi)
elseif(NOT NO_CUDA AND CUDA_FOUND AND OpenCV_CUDA_VERSION)
  message(STATUS "OpenCV was built with CUDA version: ${OpenCV_CUDA_VERSION}")
  message(STATUS "Compiling code with cv::cuda API")
  target_compile_definitions(usb_camera_driver PUBLIC WITH_CUDA)
endif()
target_include_directories(usb_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(usb_camera_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(usb_camera_driver
  Threads::Threads)
ament_target_dependencies(usb_camera_driver
  camera_calibration_parsers
  camera_info_manager
  dua_node
  dua_qos
  image_transport
  OpenCV
  params_manager
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  theora_wrappers)
rclcpp_components_register_nodes(usb_camera_driver "USBCameraDriver::CameraDriverNode")

# USB Camera Driver standalone application
add_executable(usb_camera_app
  src/ucd_app.cpp)
target_include_directories(usb_camera_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(usb_camera_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(usb_camera_app usb_camera_driver)
ament_target_dependencies(usb_camera_app
  dua_app_management
  rclcpp)

# Install USB Camera Driver component
install(TARGETS usb_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install USB Camera Driver app
install(TARGETS usb_camera_app
  DESTINATION lib/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
