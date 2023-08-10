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

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#if defined(WITH_VPI)
#include <vpi/Context.h>
#include <vpi/Image.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/Rescale.h>
#elif defined(WITH_CUDA)
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>

#include <theora_wrappers/publisher.hpp>

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
#if defined(WITH_VPI)
  void init_vpi();
#endif

  /* Video capture device and buffers. */
  cv::VideoCapture video_cap_;
  cv::Mat frame_, frame_rot_;
  cv::Mat rectified_frame_, frame_rect_rot_;

  /* Image processing pipeline buffers and maps. */
#if defined(WITH_VPI)
  VPIBackend vpi_backend_;
  VPIStream vpi_stream_ = nullptr;
  VPIPayload vpi_remap_payload_ = nullptr, vpi_rot_payload_ = nullptr;
  VPIImage vpi_frame_ = nullptr, vpi_frame_resized_ = nullptr, vpi_frame_rot_ = nullptr;
  VPIImage vpi_frame_rect_ = nullptr, vpi_frame_rect_rot_ = nullptr;
  VPIImage vpi_frame_wrap_ = nullptr, vpi_frame_rect_wrap_ = nullptr;
  VPIImage vpi_frame_rot_wrap_ = nullptr, vpi_frame_rect_rot_wrap_ = nullptr;
  VPIWarpMap vpi_rect_map_, vpi_rot_map_;
  VPIPolynomialLensDistortionModel vpi_distortion_model_;
  VPICameraIntrinsic vpi_camera_int_;
  const VPICameraExtrinsic vpi_camera_ext_ = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0}};
#else
  cv::Mat A_, D_;
  cv::Mat map1_, map2_;
#if defined(WITH_CUDA)
  cv::cuda::GpuMat gpu_frame_, gpu_frame_rot_;
  cv::cuda::GpuMat gpu_rectified_frame_, gpu_rectified_frame_rot_;
  cv::cuda::GpuMat gpu_map1_, gpu_map2_;
#endif
#endif

  /* Node parameters. */
  std::string frame_id_;
  int64_t fps_ = 0;
  int64_t image_height_ = 0;
  int64_t image_width_ = 0;
  int64_t rotation_ = 0;

  /* Node parameters validation routines. */
  bool validate_brightness(const rclcpp::Parameter & p);
  bool validate_exposure(const rclcpp::Parameter & p);
  bool validate_wb_temperature(const rclcpp::Parameter & p);

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr hw_enable_server_;

  /* Service callbacks. */
  void hw_enable_callback(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr resp);

  /* image_transport publishers and buffers. */
  std::shared_ptr<image_transport::CameraPublisher> camera_pub_;
  std::shared_ptr<image_transport::Publisher> rect_pub_;
  camera_info_manager::CameraInfo camera_info_{};
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  /* Theora stream publishers. */
  std::shared_ptr<TheoraWrappers::Publisher> stream_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> rect_stream_pub_;

  /* Utility routines. */
  bool open_camera();
  void close_camera();
  bool process_frame();
  Image::SharedPtr frame_to_msg(cv::Mat & frame);

  /* Camera sampling thread. */
  std::thread camera_sampling_thread_;
  void camera_sampling_routine();

  /* Synchronization primitives. */
  std::atomic<bool> stopped_;
};

} // namespace USBCameraDriver

#endif // ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP
