/*
Copyright (c) 2019 Andreas Klintberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// USEFUL LINK
// https://www.stereolabs.com/docs/ros2/ros2-composition/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "usb_camera_driver.hpp"

using namespace std::chrono_literals;
namespace usb_camera_driver
{

CameraDriver::CameraDriver(const rclcpp::NodeOptions &node_options) : Node("usb_camera_driver", node_options)
{

    frame_id_ = this->declare_parameter("frame_id", "camera");

    image_width_ = this->declare_parameter("image_width", 640);
    image_height_ = this->declare_parameter("image_height", 480);
    fps_ = this->declare_parameter("fps", 10.0);

    camera_id = this->declare_parameter("camera_id", 0);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

    camera_info_sub_ = image_transport::create_camera_subscription(
        this, 
        "image", 
        std::bind(&CameraDriver::second_callback, this, std::placeholders::_1, std::placeholders::_2), 
        "raw",
        custom_qos_profile);

    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    /* get ROS2 config parameter for camera calibration file */
    auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);

    cap.open(camera_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

    last_frame_ = std::chrono::steady_clock::now();

    timer_ = this->create_wall_timer(1ms, std::bind(&CameraDriver::ImageCallback, this));
}

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::ConvertFrameToMessage(cv::Mat &frame)
{
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // Make sure output in the size the user wants even if it is not native
    if(frame.rows != image_width_ || frame.cols != image_height_){
        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
    }

    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8";

    int num = 1; // for endianness detection
    ros_image.is_bigendian = !(*(char*)&num == 1);

    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
}

void CameraDriver::second_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img, 
                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
{
    UNUSED(cam_info);
    RCLCPP_INFO(this->get_logger(), "Image received");

    // Convert BGRA to BGR using OpenCV
    cv::Mat bgr(img->height, img->width, CV_8UC3, (void*)(&img->data[0]));

    cv::Mat greyMat;
    cv::cvtColor(bgr, greyMat, cv::COLOR_BGR2GRAY);

    // imwrite("/home/ros/workspace/pics/" + std::to_string(num_pic) + "a.jpg", greyMat);
    // num_pic++;
}

void CameraDriver::ImageCallback()
{
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/fps_*1000)
    {
        last_frame_ = now;

        // Convert to a ROS2 image
        if (!is_flipped)
        {
            image_msg_ = ConvertFrameToMessage(frame);
        }
        else
        {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
            image_msg_ = ConvertFrameToMessage(frame);
        }

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        image_msg_->header.stamp = timestamp;
        image_msg_->header.frame_id = frame_id_;

        camera_info_msg_->header.stamp = timestamp;
        camera_info_msg_->header.frame_id = frame_id_;

        camera_info_pub_.publish(image_msg_, camera_info_msg_);
    }
}
} // namespace usb_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_driver::CameraDriver)
