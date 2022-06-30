/**
 * @brief Precision landing module namespace contents.
 *
 * @author Lorenzo Bianchi <lnz.bnc@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 * @author Asgard Flight Group <asgard.drone@gmail.com>
 *
 * @date June 30, 2022
 */
/**
 * Copyright © 2022 Asgard Flight Group
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
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
 */

#ifndef CALIBRATOR_HPP
#define CALIBRATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp> 

using namespace rcl_interfaces::msg;

#define UNUSED(arg) (void)(arg)
#define LINE() std::cout << __LINE__ << std::endl 

namespace Calibrator
{

/**
 * @brief Lands on coordinates if requested.
 */
class CalibratorNode : public rclcpp::Node
{
public:
    CalibratorNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

private:
    image_transport::CameraSubscriber camera_data_sub_ptr_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    std::vector<int> ids_;
    std::vector<std::vector<cv::Point2f>> corners_;

    /* Utility routines */
    void declare_double_parameter(
        std::string && name,
        double default_val, double from, double to, double step,
        std::string && desc, std::string && constraints,
        bool read_only, ParameterDescriptor & descriptor);

    /* Node parameters descriptors */
    ParameterDescriptor aruco_size_descriptor_;
    ParameterDescriptor calibration_hgt_descriptor_;

    /* Node parameters */
    double aruco_size = 0.0; // m
    double calibration_hgt = 0.0; // m

    /* Parameters callback */
    OnSetParametersCallbackHandle::SharedPtr on_set_params_chandle_;
    SetParametersResult on_set_parameters_callback(
        const std::vector<rclcpp::Parameter> & params);

    //void camera_data_clbk(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void camera_data_clbk(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);

    cv::Mat image;

    /**
    * QoS profile for best-effort image transmission.
    */
    rmw_qos_profile_t usb_camera_qos_profile = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };

    void init_parameters();
};

} // namespace Calibrator

#endif
