/**
 * ROS 2 USB Camera Driver node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 4, 2022
 */

/**
 * Copyright © 2022 Intelligent Systems Lab
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
: Node("usb_camera_driver", opts)
{
  // Initialize node parameters
  init_parameters();

  // Initialize synchronization primitives
  is_canceling_.store(false, std::memory_order_release);

  // Open capture device
  if (!video_cap_.open(this->get_parameter("camera_id").as_int()) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
    !video_cap_.set(cv::CAP_PROP_FPS, fps_))
  {
    throw std::runtime_error("Failed to open and set up capture device");
  }

  // Create and set up CameraInfoManager
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  if (!cinfo_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }

  // Create image_transport publisher (this will use all available transports, see docs)
  camera_info_pub_ = image_transport::create_camera_publisher(
    this,
    this->get_parameter("base_topic_name").as_string(),
    this->get_parameter("best_effort_qos").as_bool() ?
    usb_camera_qos_profile : usb_camera_reliable_qos_profile);

  // Get and store current camera info
  camera_info_ = cinfo_manager_->getCameraInfo();

  // Start camera sampling thread
  camera_sampling_thread_ = std::thread{
    &CameraDriverNode::camera_sampling_routine,
    this};
}

/**
 * @brief Cleans up stuff upon node termination.
 */
CameraDriverNode::~CameraDriverNode()
{
  // Stop camera sampling thread
  is_canceling_.store(true, std::memory_order_release);
  camera_sampling_thread_.join();

  // Close video capture device
  video_cap_.release();
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
    if (is_canceling_.load(std::memory_order_acquire)) {
      break;
    }

    // Get a new frame from the camera
    video_cap_ >> frame_;

    // Process the new frame
    if (!frame_.empty()) {
      rclcpp::Time timestamp = this->get_clock()->now();

      // Generate Image message
      Image::SharedPtr image_msg = nullptr;
      if (is_flipped_) {
        cv::flip(frame_, flipped_frame_, 0);
        image_msg = frame_to_msg(flipped_frame_);
      } else {
        image_msg = frame_to_msg(frame_);
      }
      image_msg->header.set__stamp(timestamp);
      image_msg->header.set__frame_id(frame_id_);

      // Generate CameraInfo message
      CameraInfo::SharedPtr camera_info_msg = std::make_shared<CameraInfo>(camera_info_);
      camera_info_msg->header.set__stamp(timestamp);
      camera_info_msg->header.set__frame_id(frame_id_);

      // Publish new frame together with its CameraInfo on all available transports
      camera_info_pub_.publish(image_msg, camera_info_msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Empty frame");
    }

    sampling_timer.sleep();
  }

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread stopped");
}

} // namespace USBCameraDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriver::CameraDriverNode)
