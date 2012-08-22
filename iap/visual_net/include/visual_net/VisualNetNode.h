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
 * visual_net.h
 *
 *  Created on: Nov 15th, 2011
 *      Author: Roberto Martin
 */

#ifndef VISUAL_NET_NODE_H_
#define VISUAL_NET_NODE_H_

#include "VisualNet.h"

#include "ros/ros.h"
#include "opencv2/features2d/features2d.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "FeatureSet.h"
#include "FeatureTrajectorySet.h"

class VisualNetNode
{
public:
  /**
   * Constructor
   */
  VisualNetNode();

  /**
   * Destructor
   */
  virtual ~VisualNetNode();

  /**
   * Callback for the reception of a 2D FeatureSet with the new position of the tracked 2D points
   * @param feature_set - ROS message with the new 2D position of the tracked points
   */
  void featureSet2DCallback(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Callback for the reception of a 3D FeatureSet with the new position of the tracked 2D points
   * @param feature_set - ROS message with the new 3D position of the tracked points
   */
  void featureSet3DCallback(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Callback for the reception of a new RGB image from the camera. This image will be used to show the Clustering result
   * @param rgb_image
   */
  void RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image);

  /**
   * Callback for the reception of a segmentation
   * @param manual_segmentation - ROS message with the new segmentation
   */
  void externalSegmentationCallback(const sensor_msgs::PointCloud2 &manual_segmentation);

  /**
   * Signal handler to trigger the reset of the VisualNet
   * @param reset_signal - Empty message (=signal) from ROS
   */
  void resetSignalHandler(const std_msgs::Empty & reset_signal);

  /**
   * Signal handler to trigger the segmentation and Sfm
   * @param run_segm_and_sfm_signal - Empty message (=signal) from ROS
   */
  void runSegmentationAndSfMSignalHandler(const std_msgs::Empty & run_segm_and_sfm_signal);

private:

  VisualNet* _visual_net;

  bool _estimate_segmentation;

  bool _estimate_3d_shape;
  bool _segmenting_in_3d;
  bool _external_trigger;

  ros::NodeHandle _node_handle;
  ros::Subscriber _feature_set_2d_subscriber;
  ros::Subscriber _feature_set_3d_subscriber;
  ros::Subscriber _reset_subscriber;
  ros::Subscriber _run_segm_and_sfm_subscriber;
  ros::Subscriber _manual_segmentation_subscriber;
  ros::Publisher _feature_set_3d_estimated_publisher;
  ros::Publisher _segmentation_publisher;
  ros::Publisher _kinematic_analyzer_trigger_publisher;

  image_transport::ImageTransport _image_transport;
  image_transport::Subscriber _rgb_image_subscriber;
  cv_bridge::CvImagePtr _cv_ptr_rgb;
  cv_bridge::CvImagePtr _cv_ptr_rgb2;

  vision::FeatureSetPtr _fs_2d_last_frame;
  vision::FeatureSetPtr _fs_3d_last_frame;

  double _min_motion;

  std::string _video_sensor_type;

  bool _new_set_of_features;
  int _number_features;
  bool _use_rm_predictor;
  double _rm_min_dist;
  bool _use_rm3d_predictor;
  double _rm3d_min_dist;
  bool _use_fm_predictor;
  int _fm_num_hypo;
  int _fm_num_trials_per_hypo;
  bool _use_sd_predictor;
  double _sd_min_dist;
  bool _use_ld_predictor;
  double _ld_min_dist;
  double _ld_max_dist;
  bool _use_t_predictor;
  bool _use_csegm_predictor;
  bool _use_csimil_predictor;
  int _min_cluster_size;
  double _min_global_motion;

  int _first_frame; //First frame to analyze with the predictors
  int _last_frame; //Last frame to analyze with the predictors
  int _last_frame_trigger; // Last frame before the SfM and segmentation was triggered (as we use an extra thread we should store this value)

  int _video_height;
  int _video_width;
  double _focal_length;

  /**
   * Set all the parameters of the VisualNet object for the Segmentation procedure
   */
  void _setSegmentationParameters();

  /**
   * Set all the parameters of the VisualNet object to automatic trigger segmentation and/or SfM
   */
  void _setAutomaticTriggerParameters();

  /**
   * Obtains a parameter from the parameter server of ROS. It throws an exception if the parameter can't be found.
   * @param param_name - The name of the parameter
   * @param param_container - The location to store the value of the parameter
   * @return - True when the parameter is found
   */
  template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
      if (!(_node_handle.getParam(param_name, param_container)))
      {
        ROS_ERROR(
            "[VisualNet:getROSParameter] The parameter %s can not be found.", param_name.c_str());
        throw(std::string("[VisualNet] The parameter can not be found. Parameter name: ") + param_name);
        return false;
      }
      else
        return true;
    }

  /**
   * Obtains a parameter from the parameter server of ROS. It waits until the parameter is available.
   * @param param_name - The name of the parameter
   * @param param_container - The location to store the value of the parameter
   * @return - True when the parameter can be found
   */
  template<class T>
    bool getROSParameterBlocking(std::string param_name, T & param_container)
    {
      while (true)
      {
        if (!(_node_handle.getParam(param_name, param_container)))
        {
          ROS_WARN(
              "[VisualNet:getROSParameterBlocking] Waiting for the parameter %s to be obtained.", param_name.c_str());
          sleep(1);
        }
        else
          return true;
      }
    }

};

#endif /* VISUAL_NET_NODE_H_ */
