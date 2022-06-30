/**
 * ROS 2 USB Camera standalone application.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
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
 * @brief Precision Landing node constructor.
 *
 * @param node_opts Options for the base node.
 */
CalibratorNode::CalibratorNode(const rclcpp::NodeOptions & node_options)
: Node("calibrator", node_options)
{
    aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    // Initialize node parameters
    // Register parameter updates callback
    on_set_params_chandle_ = this->add_on_set_parameters_callback(
        std::bind(
        &CalibratorNode::on_set_parameters_callback,
        this,
        std::placeholders::_1));

    // Real landing target radius
    declare_double_parameter(
        "aruco_size",
        0.5, 0.001, 1, 0.001,
        "Aruco lenght size.",
        "Must be positive and in meters.",
        false,
        aruco_size_descriptor_);

    // Real landing target radius
    declare_double_parameter(
        "calibration_hgt",
        0.5, 0.01, 2, 0.01,
        "Distance between camera and aruco.",
        "Must be positive and in meters.",
        false,
        calibration_hgt_descriptor_);

    // Initialize topic subscription
    camera_data_sub_ptr_ = image_transport::create_camera_subscription(
                                this,
                                "/usb_camera_driver/bottom_camera/image_rect_color",
                                std::bind(&CalibratorNode::camera_data_clbk,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2),
                                "compressed",
                                usb_camera_qos_profile);

    RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Position corrector that uses floor grid.
 *
 * @param msg Image message to parse.
 * @param cam_info CameraInfo message not used 
 */
void CalibratorNode::camera_data_clbk(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
{
    UNUSED(cam_info);

    cv::Mat image(msg->height, msg->width, CV_8UC3, (void*)(&msg->data[0]));

    // Look for markers in the image.
    cv::aruco::detectMarkers(image, aruco_dictionary_, corners_, ids_);
    if (ids_.size() > 0)
    {
        // Aruco found.
        for (int k = 0; k < int(ids_.size()); k++)
        {
            float x1 = corners_[k][0].x;
            // float y1 = corners_[k][0].y;

            float x2 = corners_[k][1].x;
            // float y2 = corners_[k][1].y;

            float x3 = corners_[k][2].x;
            // float y3 = corners_[k][2].y;

            // float x4 = corners_[k][3].x;
            // float y4 = corners_[k][3].y;

            float l = abs(x1 - x2) > abs(x2 - x3) ? abs(x1 - x2) : abs(x2 - x3);
            float focal_len = (l * calibration_hgt) / aruco_size;
            RCLCPP_INFO(this->get_logger(), "\nx2 = %f\nx3 = %f\nf_len = %f\n", x2, x3, focal_len);
            RCLCPP_INFO(this->get_logger(), "Seen pad no. %d, focal length: %f\n", ids_[k], focal_len);
            break;
        }

        cv::aruco::drawDetectedMarkers(image, corners_, ids_);
    }

    // Update GUI window.
    cv::imshow("Bottom Camera", image);
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
        // Aruco size
        if (p.get_name() == "aruco_size") {
            if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
                res.set__successful(false);
                res.set__reason("Invalid parameter type for aruco_size");
                break;
            }
            continue;
        }

        // Calibration height
        if (p.get_name() == "calibration_hgt") {
            if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
                res.set__successful(false);
                res.set__reason("Invalid parameter type for calibration_hgt");
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
        // Aruco size
        if (p.get_name() == "aruco_size") {
            aruco_size = p.as_double();
            RCLCPP_INFO(
                this->get_logger(),
                "aruco_size: %f m",
                aruco_size);
            continue;
        }

        // Calibration height
        if (p.get_name() == "calibration_hgt") {
            calibration_hgt = p.as_double();
            RCLCPP_INFO(
                this->get_logger(),
                "calibration_hgt: %f m/s",
                calibration_hgt);
            continue;
        }
    }

    return res;
}

} // namespace Calibrator