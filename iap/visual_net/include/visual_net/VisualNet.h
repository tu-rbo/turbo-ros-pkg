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

#ifndef VISUAL_NET_H_
#define VISUAL_NET_H_

#include "ros/ros.h"
#include "opencv2/features2d/features2d.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "feature.h"
#include "Graph.h"
#include "FeatureSet.h"
#include "FeatureTrajectorySet.h"
#include "ClusterTrajectorySet.h"

#include "sfm/bundler/BundlerInitializer.h"
#include "ekf.h"
#include "SFMClass.h"

#include <map>
#include <pthread.h>

class VisualNet
{
public:
  /**
   * Constructor
   */
  VisualNet(int number_features, bool estimate_segmentation, bool estimate_3d_shape, bool segmenting_in_3d, bool external_trigger,
            ros::Publisher* segmentation_publiser, ros::Publisher* shape_3d_publisher,
            ros::Publisher* ka_trigger_publisher, int width, int height);

  /**
   * Destructor
   */
  virtual ~VisualNet();

  /**
   * Set the segmentation results
   * @param segmentation - New values of the segmentation. Each value is the cluster id of a feature
   */
  virtual void setSegmentation(std::vector<int> segmentation);

  /**
   * Set the general parameters to run the segmentation procedure
   * @param min_motion - Minimum motion required from a feature to trigger the segmentation
   * @param min_global_motion - Minimum motion required from all features together to trigger the segmentation
   * @param min_cluster_size - Minimum number of features that have to move min_motion to trigger the segmentation
   */
  virtual void setSegmentationParameters(double min_motion, double min_global_motion, int min_cluster_size);

  /**
   * Set the parameters of the camera for SfM
   * @param width - Width of the images
   * @param height - Height of the images
   */
  virtual void setSfMParameters(int width, int height, double focal_length);

  /**
   * A new set of 2D features is added to the internal storage
   * @param fs_2d_new - Feature set to store
   */
  virtual void addFeatureSet2D(vision::FeatureSetPtr fs_2d_new);

  /**
   * A new set of 3D features is added to the internal storage
   * @param fs_3d_new - Feature set to store
   */
  virtual void addFeatureSet3D(vision::FeatureSetPtr fs_3d_new);

  /**
   * Create and add a Relative motion predictor to the list of predictors
   * @param rm_min_dist - Minimum variation of the distance to consider that two features don't belong to the same rigid body
   */
  virtual void addRelativeMotionPredictor(double rm_min_dist);

  /**
   * Create and add a Relative motion 3D predictor to the list of predictors
   * @param rm_min_dist - Minimum variation of the distance to consider that two features don't belong to the same rigid body
   */
  virtual void addRelativeMotion3DPredictor(double rm3d_min_dist);

  /**
   * Create and add a Fundamental matrix predictor to the list of predictors
   * @param fm_num_hypo - Number of hypothesis to test
   * @param fm_num_trials_per_hypo - Number of trials to test pro hypothesis
   */
  virtual void addFundamentalMatrixPredictor(int fm_num_hypo, int fm_num_trials_per_hypo);

  /**
   * Create and add a Short distance predictor to the list of predictors
   * @param sd_min_dist - Minimum distance to consider two features to belong to the same rigid body
   */
  virtual void addShortDistancePredictor(double sd_min_dist);

  /**
   * Create and add a Long distance predictor to the list of predictors
   * @param ld_min_dist - Minimum distance to start considering that two features are not on the same rigid body
   * @param ld_max_dist - Maximum distance
   */
  virtual void addLongDistancePredictor(double ld_min_dist, double ld_max_dist);

  /**
   * Create and add a Triangulation predictor to the list of predictors
   */
  virtual void addTriangulationPredictor();

  /**
   * Create and add a Color segmentation predictor to the list of predictors
   */
  virtual void addColorSegmentationPredictor();

  /**
   * Create and add a Color similarity predictor to the list of predictors
   */
  virtual void addColorSimilarityPredictor();

  /**
   * Reset the VisualNet object
   */
  virtual void reset();

  /**
   * Copy the last received image
   * @param last_image_ptr - Boost shared pointer to the last received image from the sensor
   */
  virtual void setLastRGBImage(cv::Mat last_image);

  /**
   * Create a new thread that will run the segmentation algorithm and the SfM to estimate the 3D coordinates
   */
  void launchSegmentationAndSfMThread();

private:

  bool _estimate_segmentation;
  bool _estimate_3d_shape;
  bool _external_trigger;
  bool _segmenting_in_3d;
  int _number_features;
  int _first_frame;
  int _last_frame;
  int _trigger_frame;
  int _video_width;
  int _video_height;
  bool _first_execution;

  ros::Publisher* _segmentation_publiser;
  ros::Publisher* _shape_3d_publisher;
  ros::Publisher* _ka_trigger_publisher;


  cv::Mat _composed_image_res;

  cv::Mat _last_image;
  cv::Mat _last_image_cpy;

  vision::FeatureSetPtr _fs_2d_last_frame;
  vision::FeatureSetPtr _fs_3d_last_frame;

  vision::FeatureTrajectorySetPtr _fts_2d_acc;
  vision::FeatureTrajectorySetPtr _fts_2d_acc_cpy;
  vision::FeatureTrajectorySetPtr _fts_3d_acc;
  vision::FeatureTrajectorySetPtr _fts_3d_acc_cpy;

  vision::FeatureTrajectorySetPtr _fts_2d_total_acc;
  vision::FeatureTrajectorySetPtr _fts_2d_total_acc_cpy;
  vision::FeatureTrajectorySetPtr _fts_3d_total_acc;
  vision::FeatureTrajectorySetPtr _fts_3d_total_acc_cpy;

  vision::FeatureSetPtr _sfm_last_result;

  std::vector<std::vector<vision::FeaturePtr> > _accumulator_2d_copy;
  std::vector<std::vector<vision::FeaturePtr> > _accumulator_2d_total_copy;
  std::vector<std::vector<vision::FeaturePtr> > _accumulator_3d_copy;
  std::vector<std::vector<vision::FeaturePtr> > _accumulator_3d_total_copy;

  std::vector<int> _segmentation_last_result; //(Size=_number_features) Ids of the cluster to whom each feature belongs (-1 if it is lost)

  int _min_cluster_size;
  double _min_motion;
  double _min_global_motion;

  VisualGraph::Graph _graph;
  VisualGraph::Vertex** _vertices;
  std::vector<int> _vertex_ids; //(Size=_number_features) Ids of the corresponding vertex in the graph

  //SfM
  vision::CameraModel* _camera;
  SFM::SFMClass* _sfm_class;

  std::map<int, std::vector<vision::FeaturePtr> > _last_frame_previous_loop;
  std::map<int, bool> _first_motion;

  std::vector<int> _number_feats_per_cluster;

  int _number_of_active_threads;
  std::vector<pthread_t> _segmentation_and_sfm_threads;
  std::vector<int> _thread_ids;

  /**
   * Check the motion registered in the latest frame triggers the Clustering algorithm
   * @return - True if either enough Features moved more than th1 or if all Features together moved more than th2
   */
  bool _automaticTriggerSegmentationAndSfM();

  /**
   * The Clustering Algorithm
   */
  void _runSegmentationAndSfM();

  /**
   * Segmentation Algorithm
   */
  void _runSegmentation();

  /**
   * SfM using ClusterTrajectorySet objects
   */
  void _runSfM();

  /**
   * Draw the result of the Segmentation Algorithm
   */
  void _drawSegmentationResults();

};

#endif /* VISUAL_NET_NODE_H_ */
