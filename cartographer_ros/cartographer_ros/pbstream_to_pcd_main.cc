/*
 * Copyright 2020 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include <cartographer/io/submap_painter.h>
#include <queue>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include <cartographer/transform/rigid_transform.h>
#include <cartographer/sensor/range_data.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/io/image.h>
#include "cartographer_ros/node_options.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");

namespace cartographer_ros {
namespace {

void PbstreamToPcd(const std::string& pbstream_filename) {
  ::cartographer::sensor::PointCloud high_resolution_cloud_;
  ::cartographer::mapping::MapById<::cartographer::mapping::NodeId, ::cartographer::transform::Rigid3d> node_poses_;
  ::cartographer::mapping::MapById<::cartographer::mapping::NodeId, ::cartographer::sensor::PointCloud> points_with_id_;
  std::queue<::cartographer::mapping::NodeId> frame_ids_;

  ::cartographer::io::ProtoStreamReader stream(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&stream);
  ::cartographer::mapping::proto::PoseGraph pose_graph_proto =
  deserializer.pose_graph();
  
  for (const ::cartographer::mapping::proto::Trajectory& trajectory_proto :
    pose_graph_proto.trajectory())
  {
    for (const ::cartographer::mapping::proto::Trajectory::Node& node_proto :
      trajectory_proto.node())
    {
      
      ::cartographer::transform::Rigid3d global_pose = ::cartographer::transform::ToRigid3(node_proto.pose());
      node_poses_.Insert(
        ::cartographer::mapping::NodeId{ trajectory_proto.trajectory_id(), node_proto.node_index() },
                        global_pose);    
    }
  }

  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case ::cartographer::mapping::proto::SerializedData::kNode:
      {
        ::cartographer::mapping::proto::Node* proto_node = proto.mutable_node();
        ::cartographer::mapping::proto::TrajectoryNodeData proto_node_data = *proto_node->mutable_node_data();
        
        ::cartographer::mapping::TrajectoryNode::Data node_data = ::cartographer::mapping::FromProto(proto_node_data);
        ::cartographer::mapping::NodeId node_id{proto_node->node_id().trajectory_id(),proto_node->node_id().node_index()};
        ::cartographer::transform::Rigid3d node_pose = node_poses_.at(node_id);
        points_with_id_.Insert(node_id,node_data.low_resolution_point_cloud);
        
        ::cartographer::sensor::PointCloud temp
          = ::cartographer::sensor::TransformPointCloud(node_data.high_resolution_point_cloud, node_pose.cast<float>());
        high_resolution_cloud_.insert(high_resolution_cloud_.end(),temp.begin(),temp.end());
        frame_ids_.push(node_id);
      }
      default: break;
    }
  }
  
  // Google Cartographer Point Cloud Origin IMU Frame

  pcl::PointCloud<pcl::PointXYZ> cloud;
  
  cloud.width = high_resolution_cloud_.size();
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);
  
  for(unsigned long int i=0; i<high_resolution_cloud_.size(); i++)
  {
    cloud[i].x = high_resolution_cloud_[i][0];
    cloud[i].y = high_resolution_cloud_[i][1];
    cloud[i].z = high_resolution_cloud_[i][2];  
  }

  pcl::io::savePCDFile ("map_pcd.pcd", cloud);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  ::cartographer_ros::PbstreamToPcd(FLAGS_pbstream_filename);
}