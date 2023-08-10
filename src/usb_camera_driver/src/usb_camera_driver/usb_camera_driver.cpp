/**
 * ROS 2 USB Camera Driver node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 7, 2023
 */

/**
 * Copyright © 2023 Intelligent Systems Lab
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <stdexcept>

#include <usb_camera_driver/usb_camera_driver.hpp>

using namespace std::chrono_literals;
namespace USBCameraDriver
{

/**
 * @brief Builds a new CameraDriverNode.
 *
 * @param opts ROS 2 node options.
 *
 * @throws RuntimeError
 */
CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions & opts)
: NodeBase("usb_camera_driver", opts, true)
{
  // Initialize node parameters
  init_parameters();

  // Initialize synchronization primitives
  stopped_.store(true, std::memory_order_release);

#if defined(WITH_VPI)
  // Confirm VPI backend availability
  VPIContext vpi_context = nullptr;
  uint64_t vpi_context_flags = 0UL;
  if (vpiContextGetCurrent(&vpi_context) != VPIStatus::VPI_SUCCESS ||
    vpiContextGetFlags(vpi_context, &vpi_context_flags) != VPIStatus::VPI_SUCCESS ||
    (!(vpi_context_flags & VPIBackend::VPI_BACKEND_VIC) &&
    !(vpi_context_flags & VPIBackend::VPI_BACKEND_CUDA)))
  {
    RCLCPP_FATAL(this->get_logger(), "No compatible VPI backend found");
    throw std::runtime_error("No compatible VPI backend found");
  }
  if (vpi_context_flags & VPIBackend::VPI_BACKEND_VIC) {
    vpi_backend_ = VPIBackend::VPI_BACKEND_VIC;
    RCLCPP_INFO(this->get_logger(), "VPI VIC backend available");
  } else {
    vpi_backend_ = VPIBackend::VPI_BACKEND_CUDA;
    RCLCPP_INFO(this->get_logger(), "VPI CUDA backend available");
  }
#elif defined(WITH_CUDA)
  // Check for GPU device availability
  if (!cv::cuda::getCudaEnabledDeviceCount()) {
    RCLCPP_FATAL(this->get_logger(), "cv::cuda: No GPU device found");
    throw std::runtime_error("cv::cuda: No GPU device found");
  }
  RCLCPP_INFO(this->get_logger(), "cv::cuda: GPU device available");
#endif

  // Create and set up CameraInfoManager
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  cinfo_manager_->setCameraName(this->get_parameter("camera_name").as_string());
  if (!cinfo_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }

  // Create image_transport publishers (this will use all available transports, see docs)
  uint depth = uint(this->get_parameter("publisher_depth").as_int());
  camera_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/image_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      DUAQoS::Visualization::get_image_qos(depth).get_rmw_qos_profile() :
      DUAQoS::get_image_qos(depth).get_rmw_qos_profile()));
  rect_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/image_rect_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      DUAQoS::Visualization::get_image_qos(depth).get_rmw_qos_profile() :
      DUAQoS::get_image_qos(depth).get_rmw_qos_profile()));

  // Create Theora stream publishers
  stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_color",
    DUAQoS::Visualization::get_image_qos(depth).get_rmw_qos_profile());
  rect_stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_rect_color",
    DUAQoS::Visualization::get_image_qos(depth).get_rmw_qos_profile());

  // Get and store current camera info and compute undistorsion and rectification maps
  if (cinfo_manager_->isCalibrated()) {
    camera_info_ = cinfo_manager_->getCameraInfo();
#if !defined(WITH_VPI)

    A_ = cv::Mat(3, 3, CV_64FC1, camera_info_.k.data());
    D_ = cv::Mat(1, 5, CV_64FC1, camera_info_.d.data());

#if defined(WITH_CUDA)
    cv::initUndistortRectifyMap(
      A_,
      D_,
      cv::Mat::eye(3, 3, CV_64F),
      A_,
      cv::Size(image_width_, image_height_),
      CV_32FC1,
      map1_,
      map2_);
    gpu_map1_.upload(map1_);
    gpu_map2_.upload(map2_);
#else
    cv::initUndistortRectifyMap(
      A_,
      D_,
      cv::Mat::eye(3, 3, CV_64F),
      A_,
      cv::Size(image_width_, image_height_),
      CV_16SC2,
      map1_,
      map2_);
#endif
#endif
  }
  if (rotation_ == 90 || rotation_ == -90) {
    camera_info_.width = image_height_;
    camera_info_.height = image_width_;
  } else {
    camera_info_.width = image_width_;
    camera_info_.height = image_height_;
  }
#if defined(WITH_VPI)

  // Initialize VPI data
  init_vpi();
#endif

  // Initialize service servers
  hw_enable_server_ = this->create_service<SetBool>(
    "~/enable_camera",
    std::bind(
      &CameraDriverNode::hw_enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Node initialized");

  // Start sampling if requested
  if (this->get_parameter("autostart").as_bool()) {
    if (!open_camera()) {
      RCLCPP_FATAL(this->get_logger(), "Autostart failed: failed to open camera device");
      throw std::runtime_error("Autostart failed: failed to open camera device");
    }
    stopped_.store(false, std::memory_order_release);
    camera_sampling_thread_ = std::thread(
      &CameraDriverNode::camera_sampling_routine,
      this);
  }
}

/**
 * @brief Cleans up stuff upon node termination.
 */
CameraDriverNode::~CameraDriverNode()
{
  // Stop camera sampling thread
  bool expected = false;
  if (stopped_.compare_exchange_strong(
      expected,
      true,
      std::memory_order_release,
      std::memory_order_acquire))
  {
    camera_sampling_thread_.join();
    close_camera();
  }
  camera_pub_->shutdown();
  rect_pub_->shutdown();
  camera_pub_.reset();
  rect_pub_.reset();
  stream_pub_.reset();
  rect_stream_pub_.reset();

#if defined(WITH_VPI)
  // Destroy VPI resources
  vpiStreamDestroy(vpi_stream_);
  vpiImageDestroy(vpi_frame_);
  vpiImageDestroy(vpi_frame_resized_);
  vpiImageDestroy(vpi_frame_rot_);
  vpiImageDestroy(vpi_frame_rect_);
  vpiImageDestroy(vpi_frame_rect_rot_);
  vpiImageDestroy(vpi_frame_wrap_);
  vpiImageDestroy(vpi_frame_rect_wrap_);
  vpiImageDestroy(vpi_frame_rot_wrap_);
  vpiImageDestroy(vpi_frame_rect_rot_wrap_);
  vpiPayloadDestroy(vpi_remap_payload_);
  vpiPayloadDestroy(vpi_rot_payload_);
  vpiWarpMapFreeData(&vpi_rect_map_);
  vpiWarpMapFreeData(&vpi_rot_map_);
#endif
}

/**
 * @brief Gets new frames from the camera and publishes them.
 */
void CameraDriverNode::camera_sampling_routine()
{
  // High-resolution sleep timer, in nanoseconds
  rclcpp::WallRate sampling_timer(std::chrono::nanoseconds(int(1.0 / double(fps_) * 1000000000.0)));

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread started");

  while (true) {
    // Check if thread cancellation has been requested
    if (stopped_.load(std::memory_order_acquire)) {
      break;
    }

    // Get a new frame from the camera
    video_cap_ >> frame_;

    // Process the new frame
    if (!frame_.empty()) {
      rclcpp::Time timestamp = this->get_clock()->now();

      if (!process_frame()) {
        goto sleep;
      }
      cv::Mat final_frame, final_rect_frame;
      if (rotation_ != 0) {
        final_frame = frame_rot_;
        if (cinfo_manager_->isCalibrated()) {
          final_rect_frame = frame_rect_rot_;
        }
      } else {
        final_frame = frame_;
        if (cinfo_manager_->isCalibrated()) {
          final_rect_frame = rectified_frame_;
        }
      }

      // Generate Image messages
      Image::SharedPtr image_msg = nullptr, rect_image_msg = nullptr;
      if (cinfo_manager_->isCalibrated()) {
        rect_image_msg = frame_to_msg(final_rect_frame);
        rect_image_msg->header.set__stamp(timestamp);
        rect_image_msg->header.set__frame_id(frame_id_);
      }
      image_msg = frame_to_msg(final_frame);
      image_msg->header.set__stamp(timestamp);
      image_msg->header.set__frame_id(frame_id_);

      // Generate CameraInfo message
      CameraInfo::SharedPtr camera_info_msg = std::make_shared<CameraInfo>(camera_info_);
      camera_info_msg->header.set__stamp(timestamp);
      camera_info_msg->header.set__frame_id(frame_id_);

      // Publish new frame together with its CameraInfo on all available transports
      camera_pub_->publish(image_msg, camera_info_msg);
      stream_pub_->publish(image_msg);
      if (cinfo_manager_->isCalibrated()) {
        rect_pub_->publish(rect_image_msg);
        rect_stream_pub_->publish(rect_image_msg);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Empty frame");
    }

sleep:
    sampling_timer.sleep();
  }

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread stopped");
}

/**
 * @brief Toggles the video capture device and related sampling thread.
 *
 * @param req Service request to parse.
 * @param resp Service response to populate.
 */
void CameraDriverNode::hw_enable_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = true;
    if (stopped_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Open camera
      if (!open_camera()) {
        stopped_.store(true, std::memory_order_release);
        resp->set__success(false);
        resp->set__message("Failed to open camera");
        return;
      }

      // Start camera sampling thread
      camera_sampling_thread_ = std::thread(
        &CameraDriverNode::camera_sampling_routine,
        this);
    }
    resp->set__success(true);
    resp->set__message("");
  } else {
    bool expected = false;
    if (stopped_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      camera_sampling_thread_.join();
      close_camera();
    }
    resp->set__success(true);
    resp->set__message("");
  }
}

} // namespace USBCameraDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriver::CameraDriverNode)
