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
  stopped_.store(true, std::memory_order_release);

  // Create and set up CameraInfoManager
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  if (!cinfo_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }

  // Create image_transport publishers (this will use all available transports, see docs)
  camera_pub_ = image_transport::create_camera_publisher(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_color",
    this->get_parameter("best_effort_qos").as_bool() ?
    usb_camera_qos_profile : usb_camera_reliable_qos_profile);
  rect_pub_ = image_transport::create_publisher(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_rect_color",
    this->get_parameter("best_effort_qos").as_bool() ?
    usb_camera_qos_profile : usb_camera_reliable_qos_profile);

  // Get and store current camera info
  if (cinfo_manager_->isCalibrated()) {
    camera_info_ = cinfo_manager_->getCameraInfo();
    A_ = cv::Mat(3, 3, CV_64FC1, camera_info_.k.data());
    D_ = cv::Mat(1, 5, CV_64FC1, camera_info_.d.data());
  }

  // Initialize service servers
  hw_enable_server_ = this->create_service<SetBool>(
    "~/enable_camera",
    std::bind(
      &CameraDriverNode::hw_enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
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
  }
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

      // Generate Image message
      Image::SharedPtr image_msg = nullptr, rect_image_msg = nullptr;
      if (is_flipped_) {
        cv::flip(frame_, flipped_frame_, 0);
        if (cinfo_manager_->isCalibrated()) {
          cv::undistort(flipped_frame_, rectified_frame_, A_, D_);
          rect_image_msg = frame_to_msg(rectified_frame_);
          rect_image_msg->header.set__stamp(timestamp);
          rect_image_msg->header.set__frame_id(frame_id_);
        }
        image_msg = frame_to_msg(flipped_frame_);
      } else {
        if (cinfo_manager_->isCalibrated()) {
          cv::undistort(frame_, rectified_frame_, A_, D_);
          rect_image_msg = frame_to_msg(rectified_frame_);
          rect_image_msg->header.set__stamp(timestamp);
          rect_image_msg->header.set__frame_id(frame_id_);
        }
        image_msg = frame_to_msg(frame_);
      }
      image_msg->header.set__stamp(timestamp);
      image_msg->header.set__frame_id(frame_id_);

      // Generate CameraInfo message
      CameraInfo::SharedPtr camera_info_msg = std::make_shared<CameraInfo>(camera_info_);
      camera_info_msg->header.set__stamp(timestamp);
      camera_info_msg->header.set__frame_id(frame_id_);

      // Publish new frame together with its CameraInfo on all available transports
      camera_pub_.publish(image_msg, camera_info_msg);
      if (cinfo_manager_->isCalibrated()) {
        rect_pub_.publish(rect_image_msg);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Empty frame");
    }

    sampling_timer.sleep();
  }

  // Close video capture device
  video_cap_.release();

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
    if (stopped_.load(std::memory_order_acquire)) {
      // Open capture device
      if (!video_cap_.open(this->get_parameter("camera_id").as_int()) ||
        !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
        !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
        !video_cap_.set(cv::CAP_PROP_FPS, fps_))
      {
        resp->set__success(false);
        resp->set__message("Failed to open capture device");
        return;
      }

      // Set camera parameters
      double exposure = this->get_parameter("exposure").as_double();
      double brightness = this->get_parameter("brightness").as_double();
      bool success;
      if (exposure != 0.0) {
        success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
        if (!success) {
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
          return;
        }
        success = video_cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
        if (!success) {
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
          return;
        }
      }
      if (brightness != 0.0) {
        success = video_cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
        if (!success) {
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_BRIGHTNESS) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera brightness");
          return;
        }
      }

      stopped_.store(false, std::memory_order_release);

      // Start camera sampling thread
      camera_sampling_thread_ = std::thread{
        &CameraDriverNode::camera_sampling_routine,
        this};
    }
    resp->set__success(true);
    resp->set__message("");
    return;
  } else {
    if (!stopped_.load(std::memory_order_acquire)) {
      stopped_.store(true, std::memory_order_release);
      camera_sampling_thread_.join();
    }
    resp->set__success(true);
    resp->set__message("");
    return;
  }
}

} // namespace USBCameraDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriver::CameraDriverNode)
