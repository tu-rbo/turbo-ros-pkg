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
 */

/*
 * visual_net.cpp
 *
 *  Created on: Nov 15th, 2011
 *      Author: Roberto Martin
 */

#include "VisualNetNode.h"
#include "geometry_msgs/Point32.h"
#include <iostream>
#include <fstream>

#include "pcl_ros/publisher.h"
#include "pcl/io/io.h"
#include "pcl/point_types.h"

#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace vision;

VisualNetNode::VisualNetNode() :
  _image_transport(_node_handle), _estimate_segmentation(true), _estimate_3d_shape(true), _external_trigger(false),
      _min_motion(5.), _new_set_of_features(true), _number_features(200), _use_rm_predictor(true), _rm_min_dist(5.),
      _use_rm3d_predictor(true), _rm3d_min_dist(0.2), _use_fm_predictor(false), _fm_num_hypo(0),
      _fm_num_trials_per_hypo(0), _use_sd_predictor(true), _sd_min_dist(5.), _use_ld_predictor(true), _ld_min_dist(5.),
      _ld_max_dist(30.), _use_t_predictor(false), _use_csegm_predictor(false), _use_csimil_predictor(false),
      _min_cluster_size(20), _min_global_motion(1000), _first_frame(0), _last_frame(0), _last_frame_trigger(0),
      _video_height(480), _video_width(320), _focal_length(520.), _segmenting_in_3d(false)

{
  // Subscriptions
  this->_feature_set_2d_subscriber = this->_node_handle.subscribe("iap/feature_set_2d", 1,
                                                                  &VisualNetNode::featureSet2DCallback, this);
  this->_feature_set_3d_subscriber = this->_node_handle.subscribe("iap/feature_set_3d", 1,
                                                                  &VisualNetNode::featureSet3DCallback, this);
  this->_reset_subscriber = this->_node_handle.subscribe("iap/reset", 1, &VisualNetNode::resetSignalHandler, this);
  this->_run_segm_and_sfm_subscriber = this->_node_handle.subscribe("iap/visual_net/run_segm_and_sfm", 1,
                                                                    &VisualNetNode::runSegmentationAndSfMSignalHandler,
                                                                    this);
  this->_manual_segmentation_subscriber = this->_node_handle.subscribe("iap/manual_segmentation", 1,
                                                                       &VisualNetNode::externalSegmentationCallback,
                                                                       this);
  getROSParameter<std::string> ("/video_sensor_type", this->_video_sensor_type);
  if (this->_video_sensor_type == std::string("kinect"))
  {
    this->_rgb_image_subscriber = this->_image_transport.subscribe("camera/rgb/image_color", 1,
                                                                   &VisualNetNode::RGBImageCallback, this);
  }
  else if (this->_video_sensor_type == std::string("usb_cam"))
  {
    this->_rgb_image_subscriber = this->_image_transport.subscribe("camera/image_raw", 1,
                                                                   &VisualNetNode::RGBImageCallback, this);
  }
  else
  {
    std::string error_message =
        std::string("[VisualNetNode::VisualNetNode] Wrong parameter \"video_sensor_type\". Value: ")
            + std::string(this->_video_sensor_type);
    ROS_ERROR_STREAM( error_message );
    throw error_message;
  }

  // Publishing
  this->_segmentation_publisher = this->_node_handle.advertise<sensor_msgs::PointCloud2> ("iap/segmentation", 1000);
  this->_feature_set_3d_estimated_publisher
      = this->_node_handle.advertise<sensor_msgs::PointCloud2> ("iap/estimated_feature_set_3d", 1000);
  this->_kinematic_analyzer_trigger_publisher
      = this->_node_handle.advertise<const std_msgs::Empty> ("kinematic_analyzer/external_trigger", 1000);

  // Parameters
  getROSParameter<bool> ("/visual_net/external_trigger", this->_external_trigger);
  bool manual_segmentation = false;
  getROSParameter<bool> ("/manual_segmentation", manual_segmentation);
  this->_estimate_segmentation = !manual_segmentation;
  bool from_sensor = false;
  getROSParameter<bool> ("/3d_from_sensor", from_sensor);
  this->_estimate_3d_shape = !from_sensor;
  getROSParameter<int> ("/number_features", this->_number_features);
  getROSParameter<bool> ("/visual_net/segmenting_in_3d", this->_segmenting_in_3d);

  getROSParameterBlocking<int> ("/video_height", this->_video_height);
  getROSParameterBlocking<int> ("/video_width", this->_video_width);

  // CREATE AND SET VISUALNET //////////////////////////////////////////////////////////////////////////////////
  this->_visual_net = new VisualNet(this->_number_features, this->_estimate_segmentation, this->_estimate_3d_shape,
                                    this->_segmenting_in_3d, this->_external_trigger, &this->_segmentation_publisher,
                                    &this->_feature_set_3d_estimated_publisher,
                                    &this->_kinematic_analyzer_trigger_publisher, this->_video_width, this->_video_height);
  if (!this->_external_trigger)
  {
    this->_setAutomaticTriggerParameters();
  }
  if (this->_estimate_segmentation)
  {
    this->_setSegmentationParameters();
  }
  if (this->_estimate_3d_shape)
  {
    getROSParameterBlocking<double> ("/focal_length", this->_focal_length);
    this->_visual_net->setSfMParameters(this->_video_width, this->_video_height, this->_focal_length);
  }

  this->_first_frame = 0;
  this->_last_frame = 0;

  this->_fs_2d_last_frame = FeatureSetPtr(new FeatureSet(0));
  for (int i = 0; i < this->_number_features; i++)
  {
    FeaturePtr temp_f = FeaturePtr(new Feature(-1., -1., -1.));
    this->_fs_2d_last_frame->addFeature(temp_f);
  }
  this->_fs_3d_last_frame = this->_fs_2d_last_frame->clone();
}

VisualNetNode::~VisualNetNode()
{
  if (this->_visual_net)
  {
    delete this->_visual_net;
  }
}

void VisualNetNode::_setSegmentationParameters()
{
  // PREDICTORS /////////////////////////////////////////////////////////////////////////////////////////
  getROSParameter<bool> ("/visual_net/use_rm_predictor", this->_use_rm_predictor);
  if (this->_use_rm_predictor && !this->_segmenting_in_3d)
  {
    getROSParameter<double> ("/visual_net/rm_min_dist", this->_rm_min_dist);
    this->_visual_net->addRelativeMotionPredictor(this->_rm_min_dist);
  }
  getROSParameter<bool> ("/visual_net/use_rm3d_predictor", this->_use_rm3d_predictor);
  if (this->_use_rm3d_predictor && this->_segmenting_in_3d)
  {
    getROSParameter<double> ("/visual_net/rm3d_min_dist", this->_rm3d_min_dist);
    this->_visual_net->addRelativeMotion3DPredictor(this->_rm3d_min_dist);
  }
  getROSParameter<bool> ("/visual_net/use_fm_predictor", this->_use_fm_predictor);
  if (this->_use_fm_predictor)
  {
    getROSParameter<int> ("/visual_net/fm_num_hypo", this->_fm_num_hypo);
    getROSParameter<int> ("/visual_net/fm_num_trials_per_hypo", this->_fm_num_trials_per_hypo);
    this->_visual_net->addFundamentalMatrixPredictor(this->_fm_num_hypo, this->_fm_num_trials_per_hypo);
  }
  getROSParameter<bool> ("/visual_net/use_sd_predictor", this->_use_sd_predictor);
  if (this->_use_sd_predictor)
  {
    getROSParameter<double> ("/visual_net/sd_min_dist", this->_sd_min_dist);
    this->_visual_net->addShortDistancePredictor(this->_sd_min_dist);
  }
  getROSParameter<bool> ("/visual_net/use_ld_predictor", this->_use_ld_predictor);
  if (this->_use_ld_predictor)
  {
    getROSParameter<double> ("/visual_net/ld_min_dist", this->_ld_min_dist);
    getROSParameter<double> ("/visual_net/ld_max_dist", this->_ld_max_dist);
    this->_visual_net->addLongDistancePredictor(this->_ld_min_dist, this->_ld_max_dist);
  }
  getROSParameter<bool> ("/visual_net/use_t_predictor", this->_use_t_predictor);
  if (this->_use_t_predictor)
  {
    this->_visual_net->addTriangulationPredictor();
  }
  getROSParameter<bool> ("/visual_net/use_csegm_predictor", this->_use_csegm_predictor);
  if (this->_use_csegm_predictor)
  {
    this->_visual_net->addColorSegmentationPredictor();
  }
  getROSParameter<bool> ("/visual_net/use_csimil_predictor", this->_use_csimil_predictor);
  if (this->_use_csimil_predictor)
  {
    this->_visual_net->addColorSimilarityPredictor();
  }
}

void VisualNetNode::_setAutomaticTriggerParameters()
{
  getROSParameter<int> ("/min_cluster_size", this->_min_cluster_size);
  getROSParameter<double> ("/min_motion", this->_min_motion);
  getROSParameter<double> ("/visual_net/min_global_motion", this->_min_global_motion);
  this->_visual_net->setSegmentationParameters(this->_min_motion, this->_min_global_motion, this->_min_cluster_size);
}

void VisualNetNode::resetSignalHandler(const std_msgs::Empty & reset_signal)
{
  this->_new_set_of_features = true;
}

void VisualNetNode::runSegmentationAndSfMSignalHandler(const std_msgs::Empty & run_segm_and_sfm_signal)
{
  this->_visual_net->launchSegmentationAndSfMThread();
}

void VisualNetNode::featureSet2DCallback(const sensor_msgs::PointCloud2 &feature_set)
{
  FeatureSetPtr fs_2d_temp = this->_fs_2d_last_frame->clone();
  bool succeed = fs_2d_temp->updateFromROS(feature_set);
  if (!succeed)
  {
    ROS_ERROR("[VisualNetNode::featureSet2DCallback] Error occurred when getting the new Feature positions");
    return;
  }
  this->_visual_net->addFeatureSet2D(fs_2d_temp);
  this->_fs_2d_last_frame = fs_2d_temp->clone();
  this->_fs_2d_last_frame->setFrame(fs_2d_temp->getFrame() + 1);
}

void VisualNetNode::featureSet3DCallback(const sensor_msgs::PointCloud2 &feature_set)
{
  FeatureSetPtr fs_3d_temp = this->_fs_3d_last_frame->clone();
  bool succeed = fs_3d_temp->updateFromROS(feature_set);
  if (!succeed)
  {
    ROS_ERROR("[VisualNetNode::featureSet3DCallback] Error occurred when getting the new Feature positions");
    return;
  }
  this->_visual_net->addFeatureSet3D(fs_3d_temp);
  this->_fs_3d_last_frame = fs_3d_temp->clone();
  this->_fs_3d_last_frame->setFrame(fs_3d_temp->getFrame() + 1);
}

void VisualNetNode::RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image_msgs)
{
  try
  {
    this->_cv_ptr_rgb = cv_bridge::toCvCopy(rgb_image_msgs, sensor_msgs::image_encodings::BGR8);
    this->_visual_net->setLastRGBImage(this->_cv_ptr_rgb->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("[VisualNetNode::RGBImageCallback] cv_bridge exception: " << e.what());
    return;
  }
}

void VisualNetNode::externalSegmentationCallback(const sensor_msgs::PointCloud2 &manual_segmentation)
{
  std::vector<int> manual_segmentation_std;
  pcl::PointCloud<pcl::PointXYZ> manual_segmentation_pcl;
  pcl::fromROSMsg(manual_segmentation, manual_segmentation_pcl);

  pcl::PointCloud<pcl::PointXYZ>::iterator ms_it = manual_segmentation_pcl.begin();
  pcl::PointCloud<pcl::PointXYZ>::iterator ms_it_end = manual_segmentation_pcl.end();
  for (; ms_it != ms_it_end; ms_it++)
  {
    manual_segmentation_std.push_back(ms_it->x);
  }
  this->_visual_net->setSegmentation(manual_segmentation_std);
  std::stringstream segmentation_stream;
  segmentation_stream
      << std::string("[VisualNetNode::externalSegmentationCallback] New external segmentation received: ");
  for (int i = 0; i < this->_number_features; i++)
  {
    segmentation_stream << manual_segmentation_std.at(i) << std::string(" ");
  }
  ROS_INFO_STREAM(segmentation_stream.str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_net_node");
  VisualNetNode vnn;

  ros::spin();
  return (0);
}
