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
 * kinematic_analyzer.cpp
 *
 *  Created on: Dec 9, 2011
 *      Author: roberto
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
 */

#include "KinematicAnalyzerNode.h"
#include "geometry_msgs/Point32.h"
#include <iostream>
#include <fstream>

#include "pcl_ros/publisher.h"
#include "pcl/io/io.h"
#include "pcl/point_types.h"

#include "sensor_msgs/PointCloud2.h"

#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Range.h"

#include "FeatureSet.h"
#include "transformation_based/TransformationBasedJointEstimator.h"
#include "recursive/RecursiveJointEstimator.h"
#include "recursive/RecursiveJoint.h"
//#include "probabilistic/ProbabilisticJointEstimator.h"
#include "feature_based/FeatureBasedJointEstimator.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

// ROS messages
#include "iap_common/ClusteredFeatureSet.h"

#include <assert.h>
#include <math.h>
#include <numeric>

#include <ros/callback_queue.h>
#include <ostream>

// FIXME externalize parameters

#define MIN_GLOBAL_MOTION 700
#define JOINT_WINDOW_NAME "Resulting Joint"

#define AXIS_THRESHOLD 0.05

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

KinematicAnalyzerNode::KinematicAnalyzerNode(tf::TransformBroadcaster& broadcaster) :
  _frame_number(0), _image_transport(_node_handle), _last_cluster_set_time_stamp(0), _br(broadcaster), _min_motion(0.)
{
  //Subscribe to feature_set topic
  _feature_set_2d_subscriber = _node_handle.subscribe("iap/feature_set_2d", 1,
                                                      &KinematicAnalyzerNode::featureSet2DCallback, this);

  // Reset topic for kinematic analyzer
  _feature_reset_subscriber = _node_handle.subscribe("iap/reset", 1, &KinematicAnalyzerNode::newSetOfFeaturesCallback,
                                                     this);

  // Reset of number of features parameter
  _number_features_subscriber = _node_handle.subscribe("iap/number_features", 1,
                                                       &KinematicAnalyzerNode::numberOfFeaturesCallback, this);

  int interval_selection = 0;
  getROSParameter<int> ("/kinematic_analyzer/interval_selection", interval_selection);
  this->_interval_selection = (IntervalSelection)interval_selection;

  this->_interval_selection_overlap = 10;
  getROSParameter<int> ("/kinematic_analyzer/interval_selection_overlap", this->_interval_selection_overlap);

  getROSParameter<std::string> ("/video_sensor_type", this->_video_sensor_type);
  if (this->_video_sensor_type == std::string("kinect"))
  {
    this->_rgb_image_subscriber = this->_image_transport.subscribe("camera/rgb/image_color", 1,
                                                                   &KinematicAnalyzerNode::RGBImageCallback, this);
  }
  else if (this->_video_sensor_type == std::string("usb_cam"))
  {
    this->_rgb_image_subscriber = this->_image_transport.subscribe("camera/image_raw", 1,
                                                                   &KinematicAnalyzerNode::RGBImageCallback, this);
  }
  else
  {
    std::string error_message =
        std::string("[KinematicAnalyzerNode::KinematicAnalyzerNode] Wrong parameter \"video_sensor_type\". Value: ")
            + std::string(this->_video_sensor_type);
    ROS_ERROR_STREAM(error_message);
    throw error_message;
  }

  getROSParameter<bool> ("/3d_from_sensor", _tracking_in_3d);
  if (this->_tracking_in_3d)
  {
    _feature_set_3d_subscriber = _node_handle.subscribe("iap/feature_set_3d", 1,
                                                        &KinematicAnalyzerNode::featureSet3DCallback, this);
  }
  else
  {
    _feature_set_3d_subscriber = _node_handle.subscribe("iap/estimated_feature_set_3d", 1,
                                                        &KinematicAnalyzerNode::featureSet3DCallback, this);
  }

  //Subscribe to segmentation topic
  this->_segmentation_subscriber = this->_node_handle.subscribe("iap/segmentation", 1,
                                                                &KinematicAnalyzerNode::segmentationCallback, this);
  this->_manual_segmentation_subscriber = this->_node_handle.subscribe("iap/manual_segmentation", 1,
                                                                       &KinematicAnalyzerNode::segmentationCallback,
                                                                       this);
  //  _cluster_set_subscriber = _node_handle.subscribe("iap/cluster_set", 1, &KinematicAnalyzerNode::segmentationCallback,
  //                                                   this);

  this->_external_trigger_subscriber = _node_handle.subscribe("kinematic_analyzer/external_trigger", 1,
                                                              &KinematicAnalyzerNode::analysisTriggerSignal, this);

  _new_set_of_features = true;
  _number_features = 0;
  _min_cluster_size = 0;
  _trigger_by_segmentation = false;

  getROSParameter<std::string> ("/video_sensor_type", _video_sensor_type);
  getROSParameter<int> ("/number_features", _number_features);
  getROSParameter<int> ("/min_cluster_size", _min_cluster_size);
  getROSParameter<std::string> ("/kinematic_analyzer/joint_estimator", this->_joint_estimator);
  // optional param
  _node_handle.getParam("/kinematic_analyzer/trigger_by_segmentation", this->_trigger_by_segmentation);

  // Advertise KinematicStructure publisher

  _pub_kinematic_structure = _node_handle.advertise<iap_common::KinematicStructureMsg> ("iap/kinematic_structure", 1);

  // Initialize KinematicAnalyzer
  _kinematic_analyzer = NULL;//new KinematicAnalyzer;

  this->_last_feature_set_2d = vision::FeatureSetPtr(new vision::FeatureSet(0));
  for (int i = 0; i < this->_number_features; i++)
  {
    vision::FeaturePtr temp_f = vision::FeaturePtr(new vision::Feature(-1., -1., -1.));
    this->_last_feature_set_2d->addFeature(temp_f);
  }
  this->_last_feature_set_3d = this->_last_feature_set_2d->clone();
  _accumulator_2d = vision::FeatureTrajectorySetPtr(new vision::FeatureTrajectorySet());
  _accumulator_3d = vision::FeatureTrajectorySetPtr(new vision::FeatureTrajectorySet());

  reset();

  //cv::namedWindow(JOINT_WINDOW_NAME, CV_WINDOW_NORMAL);

  _rviz_publisher = _node_handle.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array", 0);
  _rviz_publisher_cones = _node_handle.advertise<sensor_msgs::Range> ("cones", 0);

  this->_trigger_frames.open("axes/analysis_frames.txt", std::ios_base::trunc);
}

void KinematicAnalyzerNode::reset()
{
  _good_tracked_feats_2d = 0;
  _good_tracked_feats_3d = 0;
  _accumulated_motion_2d = 0;
  _accumulated_motion_3d = 0.0;

  _features_status_2d.clear();
  _features_status_2d.resize(_number_features, true);
  _features_status_3d.clear();
  _features_status_3d.resize(_number_features, true);
  _feature_cluster_ids.clear();
  _feature_cluster_ids.resize(_number_features, -1);

  _global_and_local_transformations.clear();

  _last_feature_set_2d->clear();
  for (int i = 0; i < this->_number_features; i++)
  {
    vision::FeaturePtr temp_f = vision::FeaturePtr(new vision::Feature(-1., -1., -1.));
    this->_last_feature_set_2d->addFeature(temp_f);
  }
  _last_feature_set_3d->clear();
  _last_feature_set_3d = _last_feature_set_2d->clone();
  _accumulator_2d->clear();
  _accumulator_3d->clear();

  getROSParameter<double> ("/min_motion", this->_min_motion);

  //  _cts.reset(new vision::ClusterTrajectorySet(_accumulator_3d, clusters));

  if (_kinematic_analyzer != NULL)
    delete _kinematic_analyzer;
  _kinematic_analyzer = new KinematicAnalyzer;

  if (this->_joint_estimator == std::string("TransformationBasedJointEstimator"))
  {
    JointEstimatorPtr je(new TransformationBasedJointEstimator(this->_min_motion));
    this->_kinematic_analyzer->setJointEstimator(je);
  }
  else if (this->_joint_estimator == std::string("RecursiveJointEstimator"))
  {
    ROS_INFO_NAMED("KinematicAnalyzerNode.reset", "Recursive Estimation ON");
    std::string joint_estimator_in_re;
    getROSParameter<std::string> ("/kinematic_analyzer/joint_estimator_in_re", joint_estimator_in_re);

    JointEstimatorPtr je;
    if (joint_estimator_in_re == std::string("TransformationBasedJointEstimator"))
    {
      je = JointEstimatorPtr(new TransformationBasedJointEstimator(this->_min_motion));
    }
    //    else if (joint_estimator_in_re == std::string("ProbabilisticJointEstimator"))
    //    {
    //      je = JointEstimatorPtr(new ProbabilisticJointEstimator(this->_min_motion));
    //    }
    else if (joint_estimator_in_re == std::string("FeatureBasedJointEstimator"))
    {
      je = JointEstimatorPtr(new FeatureBasedJointEstimator(this->_min_motion));
    }
    else
    {
      ROS_ERROR_NAMED("KinematicAnalyzerNode.reset", "The type of the JointEstimator to be used inside the "
        "RecursiveJointEstimator is wrong");
    }
    RecursiveJointEstimatorPtr jee(new RecursiveJointEstimator(this->_min_motion));
    jee->setEstimator(je);
    this->_kinematic_analyzer->setJointEstimator(jee);
  }
  //  else if (this->_joint_estimator == std::string("ProbabilisticJointEstimator"))
  //  {
  //    JointEstimatorPtr je(new ProbabilisticJointEstimator(this->_min_motion));
  //    this->_kinematic_analyzer->setJointEstimator(je);
  //  }
  else if (this->_joint_estimator == std::string("FeatureBasedJointEstimator"))
  {
    JointEstimatorPtr je(new FeatureBasedJointEstimator(this->_min_motion));
    this->_kinematic_analyzer->setJointEstimator(je);
  }
  else
  {
    ROS_ERROR_NAMED("KinematicAnalyzerNode.reset", "The type of the JointEstimator for the KinematicAnalyzer is wrong");
  }

  this->_kinematic_analyzer->setIntervalSelectionCriterion(this->_interval_selection);
  this->_kinematic_analyzer->setIntervalSelectionOverlap(this->_interval_selection_overlap);

  //  _kinematic_analyzer->reset();
  _new_set_of_features = true;

  // flush all queues
  //  dynamic_cast<ros::CallbackQueue*>(_node_handle.getCallbackQueue())->clear();
}

KinematicAnalyzerNode::~KinematicAnalyzerNode()
{
}

void KinematicAnalyzerNode::RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image_msgs)
{
  try
  {
    this->_cv_ptr_rgb = cv_bridge::toCvCopy(rgb_image_msgs, sensor_msgs::image_encodings::BGR8);
    this->_kinematic_analyzer->setLastRGBImage(this->_cv_ptr_rgb->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("[KinematicAnalyzerNode::RGBImageCallback] cv_bridge exception: " << e.what());
    return;
  }
}

void KinematicAnalyzerNode::newSetOfFeaturesCallback(const std_msgs::Empty & reset_signal)
{
  ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.newSetOfFeaturesCallback",
                        "[KinematicAnalyzerNode::newSetOfFeaturesCallback] Resetting features");
  reset();
}

void KinematicAnalyzerNode::numberOfFeaturesCallback(const std_msgs::UInt32 &number_features)
{
  _number_features = number_features.data;
  ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.KinematicAnalyzerNode",
                        "Reset number of features to " << _number_features);
  reset();
}

void KinematicAnalyzerNode::featureSet2DCallback(const sensor_msgs::PointCloud2 &feature_set)
{
}

void KinematicAnalyzerNode::featureSet3DCallback(const sensor_msgs::PointCloud2 &feature_set)
{
  // reject feature set if it is timed and if it is older than the last cluster set
  if (feature_set.header.stamp.nsec != 0 && _last_cluster_set_time_stamp > feature_set.header.stamp)
  {
    ROS_WARN_STREAM_NAMED(
                          "KinematicAnalyzerNode.featureSet3DCallback",
                          " Feature set is older than last cluster set (nsec diff: "
                              << (_last_cluster_set_time_stamp.nsec - feature_set.header.stamp.nsec)
                              << ") --> ignoring");
    return;
  }

  static int counting_three = 0;

  if (counting_three == 0)
  {

    vision::FeatureSetPtr fs_3d_temp = this->_last_feature_set_3d->clone();
    bool succeed = fs_3d_temp->updateFromROS(feature_set);
    if (!succeed)
    {
      ROS_ERROR_NAMED("KinematicAnalyzerNode.featureSet3DCallback",
                      "Error occurred when getting the new Feature positions");
      return;
    }

    this->_frame_number++;
    this->_kinematic_analyzer->addFeatureSet3D(fs_3d_temp);
    counting_three = 3;
  }
  else
  {
    counting_three--;
  }

  //  this->_last_n_feature_sets_3d.push_back(fs_3d_temp);
  //  if (this->_last_n_feature_sets_3d.size() >= 15)
  //  {
  //    vision::FeatureSet::const_iterator fs_it_1 = this->_last_n_feature_sets_3d.at(0)->begin();
  //    vision::FeatureSet::const_iterator fs_it_2 = this->_last_n_feature_sets_3d.at(14)->begin();
  //    double total_distance = 0.0;
  //    for (; fs_it_1 != this->_last_n_feature_sets_3d.at(0)->end() && fs_it_1
  //        != this->_last_n_feature_sets_3d.at(14)->end(); fs_it_1++, fs_it_2++)
  //    {
  //      if (!((*fs_it_1)->isLost()) && !((*fs_it_2)->getX()))
  //      {
  //        double distance_one_feat = sqrt(
  //                                        pow((*fs_it_1)->getX() - (*fs_it_2)->getX(), 2) + pow(
  //                                                                                              (*fs_it_1)->getY()
  //                                                                                                  - (*fs_it_2)->getY(),
  //                                                                                              2)
  //                                            + pow((*fs_it_1)->getZ() - (*fs_it_2)->getZ(), 2));
  //        total_distance += distance_one_feat;
  //      }
  //    }
  //    total_distance /= this->_number_features;
  //    vision::FeatureSetPtr fs_3d_temp2;
  //    if (total_distance > (this->_min_motion/10.0))
  //    {
  //      ROS_WARN_STREAM_NAMED("KinematicAnalyzerNode.featureSet3DCallback",
  //                               "Motion in the last frames. Passing FeatureSets to KA");
  //      for (int i = 0; i < 14; i++)
  //      {
  //        this->_last_n_feature_sets_3d.at(i)->setFrame(_frame_number);
  //        this->_frame_number++;
  //        this->_kinematic_analyzer->addFeatureSet3D(this->_last_n_feature_sets_3d.at(i));
  //      }
  //
  //    }
  //    else
  //    {
  //      this->_last_n_feature_sets_3d.at(0)->setFrame(_frame_number);
  //      this->_frame_number++;
  //      this->_kinematic_analyzer->addFeatureSet3D(this->_last_n_feature_sets_3d.at(0));
  //    }
  //    fs_3d_temp2 = this->_last_n_feature_sets_3d.at(14);
  //    this->_last_n_feature_sets_3d.clear();
  //    this->_last_n_feature_sets_3d.push_back(fs_3d_temp2);
  //  }
  //
  //  this->_last_feature_set_3d = fs_3d_temp->clone();

  ROS_DEBUG_STREAM_NAMED("KinematicAnalyzerNode.featureSet3DCallback", "New 3D feature set received in KA");
}

void KinematicAnalyzerNode::segmentationCallback(const sensor_msgs::PointCloud2 &segmentation)
{
  //  ROS_INFO_STREAM_NAMED("KinematicAnalyzer", "Cluster time stamp : " << _last_cluster_set_time_stamp);
  _last_cluster_set_time_stamp = segmentation.header.stamp;

  PointCloud segmentation_pcl;
  pcl::fromROSMsg(segmentation, segmentation_pcl);

  ROS_INFO_STREAM_NAMED("KinematicAnalyzer.segmentationCallback", "Segmentation received in KA node");
  //  ROS_INFO("[KinematicAnalyzerNode::clusterSetCallback] New cluster set received in KA node");
  std::vector<int> received_segmentation;
  PointCloud::iterator int_it = segmentation_pcl.begin();
  PointCloud::iterator int_it_end = segmentation_pcl.end();
  for (; int_it != int_it_end; int_it++)
  {
    received_segmentation.push_back(int_it->x);
  }

  this->_kinematic_analyzer->setSegmentation(received_segmentation);

  // Optionally we analyze after a new segmentation is received:
  if (_trigger_by_segmentation)
  {
    ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.segmentationCallback",
                          "Triggering Kinematic Analysis from SegmentationCallback");
    std_msgs::Empty nose;
    this->analysisTriggerSignal(nose);
  }
}

void KinematicAnalyzerNode::analysisTriggerSignal(const std_msgs::Empty &trigger_signal)
{
  ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.analysisTriggerSignal", "Kinematic Analysis triggered by a signal");
  this->_trigger_frames << this->_frame_number << std::endl;
  this->_kinematic_analyzer->estimateKinematicStructure();

  KinematicStructure ks = this->_kinematic_analyzer->getKinematicStructure();
  this->publishRvizMarkers(ks);
  this->publishRvizMarkers(ks);
  // publish kinematic structure message
  this->publishKinematicAnalyzerResult(ks);
}

void KinematicAnalyzerNode::publishRvizMarkers(KinematicStructure& ks)
{
  int marker_id = 0;
  visualization_msgs::MarkerArray marker_array;
  KinematicStructure::iterator ks_it = ks.begin();
  KinematicStructure::iterator ks_it_end = ks.end();
  char buffer_one_dir[20];
  char buffer_other_dir[20];
  for (; ks_it != ks_it_end; ks_it++)
  {
    sprintf(buffer_one_dir, "c%d_c%d", ks_it->first.first, ks_it->first.second);
    sprintf(buffer_other_dir, "c%d_c%d_", ks_it->first.first, ks_it->first.second);
    ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.publishRvizMarkers",
                          "Clusters " << ks_it->first.first << " " << ks_it->first.second);

    JointPtr most_probable_joint = ks_it->second->getMostProbableJoint();
    //JointPtr most_probable_joint = ks_it->second->getJointHypothesis(PRISMATIC_JOINT);
    ROS_INFO_STREAM_NAMED("KinematicAnalyzerNode.publishRvizMarkers",
                          "Most probable joint type: " << most_probable_joint->getTypeStr());
    ros::Time time_markers = ros::Time::now() - ros::Duration(3.);

    std::pair < tf::Transform, tf::Transform > transformations = most_probable_joint->getTransformationsToOrigin();
    this->_br.sendTransform(
                            tf::StampedTransform(transformations.first, time_markers, "world",
                                                 std::string(buffer_one_dir)));
    this->_br.sendTransform(
                            tf::StampedTransform(transformations.second, time_markers, "world",
                                                 std::string(buffer_other_dir)));

    std::pair < sensor_msgs::Range, sensor_msgs::Range > uncertainty_cones
        = most_probable_joint->getAxisOrientationUncertaintyMarker();
    uncertainty_cones.first.header.frame_id = std::string(buffer_one_dir);
    uncertainty_cones.second.header.frame_id = std::string(buffer_other_dir);
    uncertainty_cones.first.header.stamp = time_markers;
    uncertainty_cones.second.header.stamp = time_markers;

    this->_rviz_publisher_cones.publish(uncertainty_cones.first);
    this->_rviz_publisher_cones.publish(uncertainty_cones.second);

    visualization_msgs::Marker axis_marker = most_probable_joint->getAxisMarker();
    axis_marker.id = marker_id;
    marker_id++;
    marker_array.markers.push_back(axis_marker);
    visualization_msgs::Marker axis_posi_uncertainty_marker = most_probable_joint->getAxisPositionUncertaintyMarker();
    axis_posi_uncertainty_marker.id = marker_id;
    marker_id++;
    marker_array.markers.push_back(axis_posi_uncertainty_marker);
  }
  this->_rviz_publisher.publish(marker_array);
}

void KinematicAnalyzerNode::publishKinematicAnalyzerResult(KinematicStructure& ks)
{
  //_pub_kinematic_structure.publish(ks.toROSMsg());
}

/******************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematic_analyzer_node");
  ros::NodeHandle nh("~");
  tf::TransformBroadcaster br;
  KinematicAnalyzerNode ka(br);
  ros::spin();

  return (0);
}
