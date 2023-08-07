/**
 * ROS 2 USB Camera Driver node.
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

#ifndef ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP
#define ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef WITH_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

using namespace sensor_msgs::msg;
using namespace std_srvs::srv;

namespace USBCameraDriver
{

/**
 * Drives USB, V4L-compatible cameras with OpenCV.
 */
class CameraDriverNode : public DUANode::NodeBase
{
public:
  explicit CameraDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~CameraDriverNode();

private:
  /* Node initialization routines. */
  void init_parameters();

  /* Video capture device and buffers. */
  cv::VideoCapture video_cap_;
  cv::Mat frame_;
  cv::Mat flipped_frame_;
  cv::Mat rectified_frame_;
  cv::Mat A_, D_;
  cv::Mat map1_, map2_;

#ifdef WITH_CUDA
  cv::cuda::GpuMat gpu_frame_;
  cv::cuda::GpuMat gpu_flipped_frame_;
  cv::cuda::GpuMat gpu_rectified_frame_;
  cv::cuda::GpuMat gpu_map1_, gpu_map2_;
#endif

  /* Node parameters. */
  std::string frame_id_;
  int64_t fps_ = 0;
  int64_t image_height_ = 0;
  int64_t image_width_ = 0;
  bool is_flipped_ = false;

  /* Node parameters validation routine. */
  bool validate_brightness(const rclcpp::Parameter & p);
  bool validate_exposure(const rclcpp::Parameter & p);
  bool validate_wb_temperature(const rclcpp::Parameter & p);

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr hw_enable_server_;

  /* Service callbacks. */
  void hw_enable_callback(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr resp);

  /* image_transport publishers and buffers. */
  image_transport::CameraPublisher camera_pub_;
  image_transport::Publisher rect_pub_;
  camera_info_manager::CameraInfo camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  /* Utility routines. */
  Image::SharedPtr frame_to_msg(cv::Mat & frame);

  /* Camera sampling thread. */
  std::thread camera_sampling_thread_;
  void camera_sampling_routine();

  /* Synchronization primitives. */
  std::atomic<bool> stopped_;
};

} // namespace USBCameraDriver

#endif // ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP