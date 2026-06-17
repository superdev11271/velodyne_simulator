/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef GAZEBO_ROS_VELODYNE_LASER_H_
#define GAZEBO_ROS_VELODYNE_LASER_H_

#include <sdf/Param.hh>

#include <gazebo/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gazebo_ros/node.hpp>

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace gazebo
{

  class GazeboRosVelodyneLaser : public SensorPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosVelodyneLaser();

    /// \brief Destructor
    public: ~GazeboRosVelodyneLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Subscribe on-demand
    private: void ConnectCb();

    /// \brief The parent ray sensor
    private: sensors::SensorPtr parent_ray_sensor_;

    private: gazebo_ros::Node::SharedPtr ros_node_;

    /// \brief ROS publisher
    private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    /// \brief ROS timer to emulate publisher connection callback
    private: rclcpp::TimerBase::SharedPtr timer_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief organize cloud
    private: bool organize_cloud_;

    /// \brief the intensity beneath which points will be filtered
    private: double min_intensity_;

    /// \brief Minimum range to publish
    private: double min_range_;

    /// \brief Maximum range to publish
    private: double max_range_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Scale intensity by distance: retro * (ref_range / range)^exponent
    private: bool use_distance_intensity_;
    private: double intensity_reference_range_;
    private: double intensity_distance_exponent_;

    private: float ComputeIntensity(double range, double material_retro) const;
    private: bool IsReturnValid(
      double range, double material_retro,
      double min_range, double max_range) const;

    /// \brief OpenMP threads: 1 = serial (fastest for typical lidars), >1 = parallel
    /// when point count is large enough, 0 = auto
    private: int omp_threads_;
    private: static constexpr int kMinPointsForParallel = 100000;
    private: bool ShouldUseOpenMP(int total_points) const;
    private: int EffectiveOpenMPThreads() const;

    /// \brief Reusable PointCloud2 buffer and scan caches
    private: static constexpr uint32_t kPointStep = 26;
    private: sensor_msgs::msg::PointCloud2 cloud_;
    private: std::vector<double> cos_yaw_;
    private: std::vector<double> sin_yaw_;
    private: std::vector<double> cos_pitch_;
    private: std::vector<double> sin_pitch_;
    private: std::vector<double> azimuth_timestamps_;
    private: int angle_table_range_count_ = 0;
    private: int angle_table_vertical_count_ = 0;

    private: void InitCloudTemplate();
    private: void UpdateAngleTables(
      int range_count, int vertical_count,
      double min_yaw, double yaw_step,
      double min_pitch, double pitch_step);
    private: void FillOrganizedCloud(
      const msgs::LaserScan & scan,
      int range_count, int vertical_count,
      double min_range, double max_range);
    private: void FillUnorganizedCloud(
      const msgs::LaserScan & scan,
      int range_count, int vertical_count,
      double min_range, double max_range);

    /// \brief A mutex to lock access
    private: std::mutex lock_;

    // Subscribe to gazebo laserscan
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: void OnScan(const ConstLaserScanStampedPtr &_msg);

    /// \brief Re-implementation of old tf::resolve
    private: static std::string tf_resolve(const std::string& prefix, const std::string& frame_id)
    {
      if (prefix.empty()) {
        return frame_id;
      }
      return prefix + "/" + frame_id;
    }
  };

} // namespace gazebo

#endif /* GAZEBO_ROS_VELODYNE_LASER_H_ */
