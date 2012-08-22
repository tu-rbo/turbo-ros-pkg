/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc. and Robotics and Biology Lab TU Berlin
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Kinematic Analyzer Node
 *
 * Connects the Kinematic Analyzer functionality to the visual net and the feature tracker
 * by converting types, initializing and invoking the Kinematic Analyzer and publishing
 * its results.
 *
 *  Created on: Nov 15th, 2011
 *      Author: Roberto Martin, Sebastian Hoefer
 *
 *
 */

#ifndef KINEMATIC_ANALYZER_NODE_H_
#define KINEMATIC_ANALYZER_NODE_H_

#include "ros/ros.h"
#include "opencv2/features2d/features2d.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt32.h"

#include "KinematicAnalyzer.h"
#include "feature.h"
#include "FeatureSet.h"
#include "FeatureTrajectorySet.h"
#include "ClusterTrajectorySet.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>


#include <tf/transform_broadcaster.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <map>

class KinematicAnalyzerNode
{
public:
  KinematicAnalyzerNode(tf::TransformBroadcaster& br);

  virtual ~KinematicAnalyzerNode();

  /**
   * @brief Captures an update in number of features (and resets the KinematicAnalyzer)
   */
  void numberOfFeaturesCallback(const std_msgs::UInt32 &number_of_feats);

  void newSetOfFeaturesCallback(const std_msgs::Empty &reset_signal);

  /**
   * Callback function executed when a new 2D point cloud is received
   * @param feature_set - 2D point cloud
   */
  void featureSet2DCallback(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Callback function executed when a new 3D point cloud is received. It generates a new
   * FeatureSet from the data of the message and passes it to the KinematicAnalyzer object.
   * @param feature_set - 3D point cloud message
   */
  void featureSet3DCallback(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Callback function executed when a new segmentation is received. It passes this new
   * segmentation to the KinematicAnalyzer object
   * @param segmentation - Message containing the segmentation of the features in clusters
   */
  void segmentationCallback(const sensor_msgs::PointCloud2 &segmentation);

  /**
   * Signal function executed when the trigger signal is received. It triggers the kinematic
   * analysis
   * @param trigger_signal - Signal that triggers the analysis
   */
  void analysisTriggerSignal(const std_msgs::Empty &trigger_signal);

  /**
   * @brief Publish the result of a kinematic analyzer run
   *
   * Invoked by clusterSetCallback
   */
  void publishKinematicAnalyzerResult(KinematicStructure&);

  /**
   * Publish the result of a kinematic analyzer run as rviz markers
   *
   * Invoked by clusterSetCallback
   */
  void publishRvizMarkers(KinematicStructure& ks) ;

  /**
   * @brief Reset the node such that a fresh run of the kinematic
   * analyzer on new data can be executed
   */
  void reset();

  /**
   * Callback function executed when a new frame from the sensor (or video) is received.
   * @param rgb_image_msgs - Image received
   */
  void RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image_msgs);

private:

  tf::TransformBroadcaster _br;

  double _min_motion;

  std::string _joint_estimator;

  int _frame_number;

  IntervalSelection _interval_selection;
  int _interval_selection_overlap;

  ros::NodeHandle _node_handle;
  ros::Subscriber _feature_set_2d_subscriber;
  ros::Subscriber _feature_set_3d_subscriber;
  ros::Subscriber _segmentation_subscriber;
  ros::Subscriber _manual_segmentation_subscriber;
  ros::Subscriber _feature_reset_subscriber;
  ros::Subscriber _number_features_subscriber;
  ros::Subscriber _external_trigger_subscriber;
  ros::Publisher _rviz_publisher;
  ros::Publisher _rviz_publisher_cones;
  ros::Publisher _pub_kinematic_structure;

  image_transport::ImageTransport _image_transport;
  image_transport::Subscriber _rgb_image_subscriber;
  cv_bridge::CvImagePtr _cv_ptr_rgb;
  cv_bridge::CvImagePtr _cv_ptr_rgb_cpy;

  std::map<int, std::map<int, std::pair<vision::FeatureSet::MotionDefinition, std::vector<
      vision::FeatureSet::MotionDefinition> > > > _global_and_local_transformations;


  vision::FeatureSetPtr _last_feature_set_2d; //(Size=_number_features) Last set of 2D features received in the previous step
  std::vector<bool> _features_status_2d; //(Size=_number_features) Status of the features: true -> tracked, false -> lost
  vision::FeatureSetPtr _last_feature_set_3d; //(Size=_number_features) Last set of 3D features received in the previous step
  std::vector<bool> _features_status_3d; //(Size=_number_features) Status of the features: true -> tracked, false -> lost
  std::vector<vision::FeatureSetPtr> _last_n_feature_sets_3d;

  KinematicAnalyzer* _kinematic_analyzer;

  vision::FeatureTrajectorySetPtr _accumulator_2d; // a trajectory of 2D feature sets
  vision::FeatureTrajectorySetPtr _accumulator_3d; // a trajectory of 3D feature sets
  vision::ClusterTrajectorySetPtr _cts; // The cluster trajectory set in each run
  int _good_tracked_feats_2d;
  int _good_tracked_feats_3d;
  double _accumulated_motion_2d;
  double _accumulated_motion_3d;
  int _number_of_clusters;
  bool _trigger_by_segmentation;

  std::vector<int> _feature_cluster_ids; //(Size=_number_features) Ids of the cluster to whom each feature belongs (-1 if it is lost)

  std::string _video_sensor_type;
  bool _tracking_in_3d;

  bool _new_set_of_features;

  // Parameters from ROS
  int _number_features;
  int _min_cluster_size;

  // save last time a cluster
  ros::Time _last_cluster_set_time_stamp;

  template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
      if (!(_node_handle.getParam(param_name, param_container)))
      {
        ROS_ERROR(
            "[KinematicAnalyzer] The parameter %s can not be found.", param_name.c_str());
        throw(std::string("[KinematicAnalyzer] The parameter can not be found. Parameter name: ") + param_name);
        return false;
      }
      else
        return true;
    }

  std::ofstream _trigger_frames;
};

#endif /* KINEMATIC_ANALYZER_NODE_H_ */
