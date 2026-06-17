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

#include <velodyne_gazebo_plugins/GazeboRosVelodyneLaser.hpp>
#include <gazebo_ros/utils.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>

#include <gazebo/sensors/Sensor.hh>
static_assert(GAZEBO_MAJOR_VERSION >= 11, "Gazebo version is too old");

#ifdef _OPENMP
#include <omp.h>
#endif

namespace gazebo
{
namespace
{
double GaussianNoise(double sigma)
{
  static thread_local std::mt19937 rng{std::random_device{}()};
  static thread_local std::normal_distribution<double> dist(0.0, 1.0);
  return dist(rng) * sigma;
}

}  // namespace

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

GazeboRosVelodyneLaser::GazeboRosVelodyneLaser()
: min_range_(0),
  max_range_(0),
  gaussian_noise_(0),
  use_distance_intensity_(true),
  intensity_reference_range_(15.0),
  intensity_distance_exponent_(2.0),
  omp_threads_(0)
{
}

GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
}

void GazeboRosVelodyneLaser::InitCloudTemplate()
{
  cloud_.fields.resize(6);
  cloud_.fields[0].name = "x";
  cloud_.fields[0].offset = 0;
  cloud_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_.fields[0].count = 1;
  cloud_.fields[1].name = "y";
  cloud_.fields[1].offset = 4;
  cloud_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_.fields[1].count = 1;
  cloud_.fields[2].name = "z";
  cloud_.fields[2].offset = 8;
  cloud_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_.fields[2].count = 1;
  cloud_.fields[3].name = "intensity";
  cloud_.fields[3].offset = 12;
  cloud_.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_.fields[3].count = 1;
  cloud_.fields[4].name = "ring";
  cloud_.fields[4].offset = 16;
  cloud_.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  cloud_.fields[4].count = 1;
  cloud_.fields[5].name = "timestamp";
  cloud_.fields[5].offset = 18;
  cloud_.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
  cloud_.fields[5].count = 1;
  cloud_.point_step = kPointStep;
  cloud_.is_bigendian = false;
}

void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  ros_node_ = gazebo_ros::Node::Get(_sdf);
  parent_ray_sensor_ = _parent;

  const std::string tf_prefix = _sdf->Get<std::string>("tf_prefix", std::string("")).first;
  frame_name_ = tf_resolve(tf_prefix, gazebo_ros::SensorFrameID(*_parent, *_sdf));

  organize_cloud_ = _sdf->Get("organize_cloud", false).first;
  min_range_ = _sdf->Get("min_range", 0.0).first;
  max_range_ = _sdf->Get("max_range", std::numeric_limits<double>::infinity()).first;
  min_intensity_ = _sdf->Get("min_intensity", std::numeric_limits<double>::lowest()).first;
  gaussian_noise_ = _sdf->Get("gaussian_noise", 0.0).first;
  use_distance_intensity_ = _sdf->Get("use_distance_intensity", true).first;
  intensity_reference_range_ = _sdf->Get("intensity_reference_range", 15.0).first;
  intensity_distance_exponent_ = _sdf->Get("intensity_distance_exponent", 2.0).first;
  omp_threads_ = _sdf->Get("threads", 0).first;

  InitCloudTemplate();

  pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out", 10);

  using namespace std::chrono_literals;
  timer_ = ros_node_->create_wall_timer(0.5s, std::bind(&GazeboRosVelodyneLaser::ConnectCb, this));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "Velodyne laser plugin ready (organize_cloud=%s, use_distance_intensity=%s, threads=%d)",
    organize_cloud_ ? "true" : "false",
    use_distance_intensity_ ? "true" : "false",
    omp_threads_);
}

void GazeboRosVelodyneLaser::ConnectCb()
{
  std::lock_guard<std::mutex> lock(lock_);
  if (pub_->get_subscription_count()) {
    if (!sub_) {
      sub_ = gazebo_node_->Subscribe(
        parent_ray_sensor_->Topic(), &GazeboRosVelodyneLaser::OnScan, this);
    }
    parent_ray_sensor_->SetActive(true);
  } else {
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
    parent_ray_sensor_->SetActive(false);
  }
}

void GazeboRosVelodyneLaser::UpdateAngleTables(
  const int range_count, const int vertical_count,
  const double min_yaw, const double yaw_step,
  const double min_pitch, const double pitch_step)
{
  if (range_count == angle_table_range_count_ &&
    vertical_count == angle_table_vertical_count_)
  {
    return;
  }

  angle_table_range_count_ = range_count;
  angle_table_vertical_count_ = vertical_count;

  cos_yaw_.resize(range_count);
  sin_yaw_.resize(range_count);
  for (int i = 0; i < range_count; ++i) {
    const double yaw = min_yaw + static_cast<double>(i) * yaw_step;
    cos_yaw_[i] = std::cos(yaw);
    sin_yaw_[i] = std::sin(yaw);
  }

  cos_pitch_.resize(vertical_count);
  sin_pitch_.resize(vertical_count);
  for (int j = 0; j < vertical_count; ++j) {
    const double pitch = min_pitch + static_cast<double>(j) * pitch_step;
    cos_pitch_[j] = std::cos(pitch);
    sin_pitch_[j] = std::sin(pitch);
  }
}

bool GazeboRosVelodyneLaser::IsReturnValid(
  const double range,
  const double material_retro,
  const double min_range,
  const double max_range) const
{
  if (!std::isfinite(range)) {
    return false;
  }
  if (!(min_range < range && range < max_range)) {
    return false;
  }
  // Gazebo reports zero intensity for rays that miss all geometry.
  if (material_retro <= 0.0) {
    return false;
  }
  if (!use_distance_intensity_ && material_retro < min_intensity_) {
    return false;
  }
  return true;
}

float GazeboRosVelodyneLaser::ComputeIntensity(
  const double range, const double material_retro) const
{
  if (!use_distance_intensity_) {
    return static_cast<float>(material_retro);
  }

  const double safe_range = std::max(range, 0.1);
  const double distance_factor = std::pow(
    intensity_reference_range_ / safe_range, intensity_distance_exponent_);
  return static_cast<float>(material_retro * distance_factor);
}

bool GazeboRosVelodyneLaser::ShouldUseOpenMP(const int total_points) const
{
#ifdef _OPENMP
  if (omp_threads_ == 1 || total_points < kMinPointsForParallel) {
    return false;
  }
  return true;
#else
  (void)total_points;
  return false;
#endif
}

int GazeboRosVelodyneLaser::EffectiveOpenMPThreads() const
{
#ifdef _OPENMP
  if (omp_threads_ > 1) {
    return omp_threads_;
  }
  return std::min(4, std::max(2, omp_get_max_threads() / 2));
#else
  return 1;
#endif
}

void GazeboRosVelodyneLaser::FillOrganizedCloud(
  const msgs::LaserScan & scan,
  const int range_count, const int vertical_count,
  const double min_range, const double max_range)
{
  cloud_.height = static_cast<uint32_t>(range_count);
  cloud_.width = static_cast<uint32_t>(vertical_count);
  cloud_.row_step = kPointStep * cloud_.width;
  cloud_.data.resize(static_cast<size_t>(range_count) * vertical_count * kPointStep);
  cloud_.is_dense = false;

  const float nan_value = std::numeric_limits<float>::quiet_NaN();
  const int total_points = range_count * vertical_count;
  const bool use_parallel = ShouldUseOpenMP(total_points);

#ifdef _OPENMP
  if (use_parallel) {
    omp_set_num_threads(EffectiveOpenMPThreads());
  }
#pragma omp parallel for collapse(2) schedule(static) if(use_parallel)
#endif
  for (int i = 0; i < range_count; ++i) {
    for (int j = 0; j < vertical_count; ++j) {
      const int index = i + j * range_count;
      uint8_t * const ptr = cloud_.data.data() + static_cast<size_t>(index) * kPointStep;

      const double material_retro = scan.intensities(index);
      double range = scan.ranges(index);

      // Gaussian noise applies to range only, not intensity.
      if (gaussian_noise_ != 0.0) {
        range += GaussianNoise(gaussian_noise_);
      }

      const double cos_p = cos_pitch_[j];
      const double sin_p = sin_pitch_[j];
      const double cos_y = cos_yaw_[i];
      const double sin_y = sin_yaw_[i];
      const double timestamp = azimuth_timestamps_[i];

      if (IsReturnValid(range, material_retro, min_range, max_range)) {
        const float intensity = ComputeIntensity(range, material_retro);
        if (use_distance_intensity_ && intensity < min_intensity_) {
          *reinterpret_cast<float *>(ptr + 0) = nan_value;
          *reinterpret_cast<float *>(ptr + 4) = nan_value;
          *reinterpret_cast<float *>(ptr + 8) = nan_value;
          *reinterpret_cast<float *>(ptr + 12) = nan_value;
          *reinterpret_cast<uint16_t *>(ptr + 16) = static_cast<uint16_t>(j);
          *reinterpret_cast<double *>(ptr + 18) = timestamp;
          continue;
        }
        *reinterpret_cast<float *>(ptr + 0) = static_cast<float>(range * cos_p * cos_y);
        *reinterpret_cast<float *>(ptr + 4) = static_cast<float>(range * cos_p * sin_y);
        *reinterpret_cast<float *>(ptr + 8) = static_cast<float>(range * sin_p);
        *reinterpret_cast<float *>(ptr + 12) = intensity;
        *reinterpret_cast<uint16_t *>(ptr + 16) = static_cast<uint16_t>(j);
        *reinterpret_cast<double *>(ptr + 18) = timestamp;
      } else {
        *reinterpret_cast<float *>(ptr + 0) = nan_value;
        *reinterpret_cast<float *>(ptr + 4) = nan_value;
        *reinterpret_cast<float *>(ptr + 8) = nan_value;
        *reinterpret_cast<float *>(ptr + 12) = nan_value;
        *reinterpret_cast<uint16_t *>(ptr + 16) = static_cast<uint16_t>(j);
        *reinterpret_cast<double *>(ptr + 18) = timestamp;
      }
    }
  }
}

void GazeboRosVelodyneLaser::FillUnorganizedCloud(
  const msgs::LaserScan & scan,
  const int range_count, const int vertical_count,
  const double min_range, const double max_range)
{
  cloud_.data.resize(static_cast<size_t>(range_count) * vertical_count * kPointStep);
  cloud_.height = 1;
  cloud_.is_dense = true;

  uint8_t * out = cloud_.data.data();
  const uint8_t * const out_begin = out;

  for (int i = 0; i < range_count; ++i) {
    for (int j = 0; j < vertical_count; ++j) {
      const int index = i + j * range_count;
      double range = scan.ranges(index);
      const double material_retro = scan.intensities(index);

      if (gaussian_noise_ != 0.0) {
        range += GaussianNoise(gaussian_noise_);
      }

      if (!IsReturnValid(range, material_retro, min_range, max_range)) {
        continue;
      }

      const float intensity = ComputeIntensity(range, material_retro);
      if (use_distance_intensity_ && intensity < min_intensity_) {
        continue;
      }

      const double cos_p = cos_pitch_[j];
      const double sin_p = sin_pitch_[j];
      const double cos_y = cos_yaw_[i];
      const double sin_y = sin_yaw_[i];

      *reinterpret_cast<float *>(out + 0) = static_cast<float>(range * cos_p * cos_y);
      *reinterpret_cast<float *>(out + 4) = static_cast<float>(range * cos_p * sin_y);
      *reinterpret_cast<float *>(out + 8) = static_cast<float>(range * sin_p);
      *reinterpret_cast<float *>(out + 12) = intensity;
      *reinterpret_cast<uint16_t *>(out + 16) = static_cast<uint16_t>(j);
      *reinterpret_cast<double *>(out + 18) = azimuth_timestamps_[i];
      out += kPointStep;
    }
  }

  cloud_.data.resize(static_cast<size_t>(out - out_begin));
  cloud_.width = cloud_.data.size() / kPointStep;
  cloud_.row_step = cloud_.data.size();
}

void GazeboRosVelodyneLaser::OnScan(ConstLaserScanStampedPtr & _msg)
{
  const auto & scan = _msg->scan();
  const int range_count = scan.count();
  const int vertical_count = scan.vertical_count();

  const double min_yaw = scan.angle_min();
  const double max_yaw = scan.angle_max();
  const double min_pitch = scan.vertical_angle_min();
  const double max_pitch = scan.vertical_angle_max();

  const double yaw_step = (range_count > 1) ? (max_yaw - min_yaw) / (range_count - 1) : 0.0;
  const double pitch_step =
    (vertical_count > 1) ? (max_pitch - min_pitch) / (vertical_count - 1) : 0.0;

  const double min_range = std::max(min_range_, scan.range_min());
  const double max_range = std::min(max_range_, scan.range_max());

  cloud_.header.frame_id = frame_name_;
  cloud_.header.stamp.sec = _msg->time().sec();
  cloud_.header.stamp.nanosec = _msg->time().nsec();

  const double scan_time =
    static_cast<double>(cloud_.header.stamp.sec) +
    static_cast<double>(cloud_.header.stamp.nanosec) * 1e-9;

  azimuth_timestamps_.resize(range_count);
  for (int i = 0; i < range_count; ++i) {
    azimuth_timestamps_[i] = scan_time + static_cast<double>(i) * yaw_step;
  }

  UpdateAngleTables(range_count, vertical_count, min_yaw, yaw_step, min_pitch, pitch_step);

  if (organize_cloud_) {
    FillOrganizedCloud(scan, range_count, vertical_count, min_range, max_range);
  } else {
    FillUnorganizedCloud(scan, range_count, vertical_count, min_range, max_range);
  }

  pub_->publish(cloud_);
}

}  // namespace gazebo
