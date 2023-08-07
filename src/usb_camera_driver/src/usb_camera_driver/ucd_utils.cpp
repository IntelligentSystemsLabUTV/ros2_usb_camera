/**
 * ROS 2 USB Camera Driver node auxiliary routines.
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

#include <cfloat>
#include <cstdint>
#include <cstring>

#include <usb_camera_driver/usb_camera_driver.hpp>

namespace USBCameraDriver
{

/**
 * @brief Opens the camera.
 *
 * @return True if the camera was opened successfully, false otherwise.
 */
bool CameraDriverNode::open_camera()
{
  // Open capture device
  if (!video_cap_.open(this->get_parameter("camera_id").as_int(), cv::CAP_V4L2) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
    !video_cap_.set(cv::CAP_PROP_FPS, fps_))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "CameraDriverNode::open_camera: Failed to open capture device");
    return false;
  }

  // Set camera parameters
  double exposure = this->get_parameter("exposure").as_double();
  double brightness = this->get_parameter("brightness").as_double();
  double wb_temperature = this->get_parameter("wb_temperature").as_double();
  bool success;
  if (exposure != 0.0) {
    success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
      return false;
    }
    success = video_cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
      return false;
    }
  }
  if (brightness != 0.0) {
    success = video_cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_BRIGHTNESS) failed");
      return false;
    }
  }
  if (wb_temperature == 0.0) {
    success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
      return false;
    }
  } else {
    success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
      return false;
    }
    success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, wb_temperature);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CameraDriverNode::open_camera: cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
      return false;
    }
  }

  return true;
}

/**
 * @brief Closes the camera.
 */
void CameraDriverNode::close_camera()
{
  if (video_cap_.isOpened()) {
    video_cap_.release();
  }
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr CameraDriverNode::frame_to_msg(cv::Mat & frame)
{
  // Resize the frame as per node parameters
  if (frame.rows != image_width_ || frame.cols != image_height_) {
    cv::resize(frame, frame, cv::Size(image_width_, image_height_));
  }

  // Allocate new image message
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  std::memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Validates the brightness parameter.
 *
 * @param p Parameter to validate.
 */
bool CameraDriverNode::validate_brightness(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened() && !video_cap_.set(cv::CAP_PROP_BRIGHTNESS, p.as_double())) {
    RCLCPP_ERROR(
      this->get_logger(),
      "CameraDriverNode::validate_brightness: Failed to set camera brightness to: %.4f",
      p.as_double());
    return false;
  }
  return true;
}

/**
 * @brief Validates the exposure parameter.
 *
 * @param p Parameter to validate.
 */
bool CameraDriverNode::validate_exposure(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened()) {
    bool success;
    if (p.as_double() == 0.0) {
      success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode::validate_exposure: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 3.0) failed");
        return false;
      }
    } else {
      success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode::validate_exposure: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
        return false;
      }
      success = video_cap_.set(cv::CAP_PROP_EXPOSURE, p.as_double());
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode::validate_exposure: cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Validates the WB temperature parameter.
 *
 * @param p Parameter to validate.
 */
bool CameraDriverNode::validate_wb_temperature(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened()) {
    bool success;
    if (p.as_double() == 0.0) {
      success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode::validate_wb_temperature: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
        return false;
      }
    } else {
      success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode:: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
        return false;
      }
      success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, p.as_double());
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CameraDriverNode:: cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
        return false;
      }
    }
  }
  return true;
}

} // namespace USBCameraDriver
