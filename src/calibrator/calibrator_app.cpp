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

#define MODULE_NAME "calibrator_app"
#define UNUSED(arg) (void)(arg)

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <iostream>

#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <calibrator/calibrator.hpp>

using namespace Calibrator;

int main(int argc, char ** argv)
{
    // Disable I/O buffering
    if (setvbuf(stdout, NULL, _IONBF, 0)) {
        RCLCPP_FATAL(
        rclcpp::get_logger(MODULE_NAME),
        "Failed to set I/O buffering");
        exit(EXIT_FAILURE);
    }

    // Create and initialize ROS 2 context
    rclcpp::init(argc, argv);

    // Initialize ROS 2 node
    auto calibrator_node = std::make_shared<CalibratorNode>();

    // Create and configure executor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(calibrator_node);

    RCLCPP_WARN(
        rclcpp::get_logger(MODULE_NAME),
        "(%d) " MODULE_NAME " online",
        getpid());

    // Spin the executor
    executor->spin();

    // Destroy ROS 2 node and context
    calibrator_node.reset();
    rclcpp::shutdown();

    exit(EXIT_SUCCESS);
}