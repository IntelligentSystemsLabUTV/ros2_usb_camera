/**
 * USB Camera Calibrator node implementation.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2022
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

#include <calibrator/calibrator.hpp>

namespace Calibrator
{

/**
 * @brief Calibrator node constructor.
 *
 * @param node_opts Options for the base node.
 */
CalibratorNode::CalibratorNode(const rclcpp::NodeOptions & node_options)
: Node("calibrator", node_options)
{
  // Initialize ArUco dictionary
  aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

  // Initialize node parameters
  // Register parameter updates callback
  on_set_params_chandle_ = this->add_on_set_parameters_callback(
    std::bind(
      &CalibratorNode::on_set_parameters_callback,
      this,
      std::placeholders::_1));

  // ArUco marker size
  declare_double_parameter(
    "aruco_size",
    0.5, 0.001, 1, 0.001,
    "ArUco marker size [m].",
    "Must be positive.",
    false,
    aruco_size_descriptor_);

  // ArUco marker distance
  declare_double_parameter(
    "calibration_hgt",
    0.5, 0.01, 2, 0.01,
    "Distance between camera and ArUco marker [m].",
    "Must be positive.",
    false,
    calibration_hgt_descriptor_);

  // USB camera topic name
  declare_string_parameter(
    "usb_camera_topic_name",
    "/usb_camera_driver/camera/image_rect_color",
    "USB camera topic name.",
    "Cannot be changed.",
    true,
    usb_camera_topic_name_descriptor_);

  // Initialize camera subscription
  camera_data_sub_ = image_transport::create_camera_subscription(
    this,
    usb_camera_topic_name_,
    std::bind(
      &CalibratorNode::camera_data_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    "compressed",
    usb_camera_qos_profile);

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Camera calibration routine, as an Image callback.
 *
 * @param msg Image message to parse.
 * @param cam_info CameraInfo message.
 */
void CalibratorNode::camera_data_clbk(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  UNUSED(cam_info); // Remove this if you need cam_info!

  // Parse image message into an OpenCV image
  cv::Mat image(msg->height, msg->width, CV_8UC3, (void *)(msg->data.data()));

  // Look for markers in the image
  cv::aruco::detectMarkers(image, aruco_dictionary_, corners_, ids_);
  if (ids_.size() > 0) {
    // Marker found
    for (int k = 0; k < int(ids_.size()); k++) {
      float x1 = corners_[k][0].x;
      float x2 = corners_[k][1].x;
      float x3 = corners_[k][2].x;

      // Compute focal length, in pixels
      float l = abs(x1 - x2) > abs(x2 - x3) ? abs(x1 - x2) : abs(x2 - x3);
      float focal_len = (l * calibration_hgt_) / aruco_size_;
      RCLCPP_INFO(this->get_logger(), "\nx2 = %f\nx3 = %f\nf_len = %f", x2, x3, focal_len);
      RCLCPP_INFO(this->get_logger(), "Seen marker no. %d, focal length: %f", ids_[k], focal_len);
      break;
    }

    cv::aruco::drawDetectedMarkers(image, corners_, ids_);
  }

  // Show image in a GUI window
  cv::imshow("USB Camera", image);
  cv::waitKey(1);
}

/**
 * @brief Routine to declare a 64-bit floating point node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Floating point range initial value.
 * @param to Floating point range final value.
 * @param step Floating point range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CalibratorNode::declare_double_parameter(
  std::string && name,
  double default_val, double from, double to, double step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_DOUBLE);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a string node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CalibratorNode::declare_string_parameter(
  std::string && name, std::string && default_val, std::string && desc,
  std::string && constraints, bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_STRING);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Parameters update validation callback.
 *
 * @param params Vector of parameters for which a change has been requested.
 * @return Operation result in SetParametersResult message.
 */
SetParametersResult CalibratorNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // First, check if each update is feasible
  // Initial checks must be added here!
  for (const rclcpp::Parameter & p : params) {
    // ArUco marker size
    if (p.get_name() == "aruco_size") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for aruco_size");
        break;
      }
      continue;
    }

    // ArUco marker distance
    if (p.get_name() == "calibration_hgt") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for calibration_hgt");
        break;
      }
      continue;
    }

    // USB camera topic name
    if (p.get_name() == "usb_camera_topic_name") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for usb_camera_topic_name");
        break;
      }
      continue;
    }
  }

  if (!res.successful) {
    return res;
  }

  // Then, do what is necessary to update each parameter
  // Add ad-hoc update procedures must be added here!
  for (const rclcpp::Parameter & p : params) {
    // ArUco marker size
    if (p.get_name() == "aruco_size") {
      aruco_size_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "aruco_size: %f m",
        aruco_size_);
      continue;
    }

    // ArUco marker distance
    if (p.get_name() == "calibration_hgt") {
      calibration_hgt_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "calibration_hgt: %f m",
        calibration_hgt_);
      continue;
    }

    // USB camera topic name
    if (p.get_name() == "usb_camera_topic_name") {
      usb_camera_topic_name_ = p.as_string();
      RCLCPP_INFO(
        this->get_logger(),
        "usb_camera_topic_name: %s",
        usb_camera_topic_name_.c_str());
      continue;
    }
  }

  return res;
}

} // namespace Calibrator
