/*
 * Copyright 2019 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "gflags/gflags.h"
#include "glog/logging.h"

using namespace std;

DEFINE_string(input, "", "pbstream file to process");

namespace cartographer_ros {
namespace {

geometry_msgs::msg::TransformStamped ToTransformStamped(
    int64_t timestamp_uts, const std::string& parent_frame_id,
    const std::string& child_frame_id,
    const ::cartographer::transform::proto::Rigid3d& parent_T_child) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.header.stamp = ::cartographer_ros::ToRos(
      ::cartographer::common::FromUniversal(timestamp_uts));
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = ::cartographer_ros::ToGeometryMsgTransform(
      ::cartographer::transform::ToRigid3(parent_T_child));
  return transform_stamped;
}

void pbstream_trajectory(const std::string& input,
                         const std::string& parent_frame_id) {
  
  std::string child_frame_id("imu_frame");
  std::string filename("slam_trajectory_from_pbstream.txt");
  double sec{0.0};
  double nanosec{0.0};
  double timestamp{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  double q_w{1.0};

  ofstream slam_trajectory;
  slam_trajectory.open(filename);

  slam_trajectory << "#" << " " << "filename" << " " << "=" << " " << filename << std::endl;
  slam_trajectory << "#" << " " << "ParentFrame" << " " << "=" << " " << parent_frame_id << std::endl;
  slam_trajectory << "#" << " " << "ChildFrame" << " " << "=" << " " << child_frame_id << std::endl;
  slam_trajectory << "#" << " " << "timestamp" << " " << "x" << " " << "y" << " " << "z" << " " << "q_x" << " " << "q_y" << " " << "q_z" << " " << "q_w" << std::endl;

  const auto pose_graph =
      ::cartographer::io::DeserializePoseGraphFromFile(input);

  for (const auto trajectory : pose_graph.trajectory()) {
    LOG(INFO)
        << trajectory.trajectory_id() << " with " << trajectory.node_size()
        << " nodes.";
    for (const auto& node : trajectory.node()) {
      geometry_msgs::msg::TransformStamped transform_stamped = ToTransformStamped(node.timestamp(), parent_frame_id, child_frame_id, node.pose());

      sec = transform_stamped.header.stamp.sec*1.0;
      nanosec = transform_stamped.header.stamp.nanosec*1.0*pow(10.0,-9.0);
      timestamp = sec + nanosec;
      x = transform_stamped.transform.translation.x,
      y = transform_stamped.transform.translation.y;
      z = transform_stamped.transform.translation.z;
      q_x = transform_stamped.transform.rotation.x;
      q_y = transform_stamped.transform.rotation.y;
      q_z = transform_stamped.transform.rotation.z;
      q_w = transform_stamped.transform.rotation.w;

      slam_trajectory << std::fixed << std::setprecision(9) << timestamp << " " << x << " " << y << " " << z << " " << q_x << " " << q_y << " " << q_z << " " << q_w << std::endl;
    }
  }
  slam_trajectory.close();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input.empty()) << "-input pbstream is missing.";
  
  ::cartographer_ros::pbstream_trajectory(FLAGS_input, "map");
}