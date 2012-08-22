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

#include "VisualNet.h"
#include "ClusterSet.h"
#include "ClusterTrajectorySet.h"
#include "geometry_msgs/Point32.h"
#include <iostream>
#include <fstream>

#include "pcl_ros/publisher.h"
#include "pcl/io/io.h"
#include "pcl/point_types.h"

#include "sensor_msgs/PointCloud2.h"

#include "Predictor.h"
#include "FundamentalMatrix.h"
#include "ShortDistance.h"
#include "ShortDistance3D.h"
#include "LongDistance.h"
#include "LongDistance3D.h"
#include "Triangulation.h"
#include "ColorSegmentation.h"
#include "RelativeMotion.h"
#include "RelativeMotion3D.h"

#include <assert.h>
#include <math.h>
#include <numeric>

#define CLUSTERS_WDW "Resulting Clusters"
#define CLUSTER_CONNECTIVITY_WDW "Cluster connectivity"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using std::vector;
using namespace vision;

VisualNet::VisualNet(int number_features, bool estimate_segmentation, bool estimate_3d_shape, bool segmenting_in_3d,
                     bool external_trigger, ros::Publisher* segmentation_publiser, ros::Publisher* shape_3d_publisher,
                     ros::Publisher* ka_trigger_publisher, int width, int height)
{
  this->_number_features = number_features;
  this->_estimate_segmentation = estimate_segmentation;
  this->_estimate_3d_shape = estimate_3d_shape;
  this->_segmenting_in_3d = segmenting_in_3d;
  this->_external_trigger = external_trigger;
  this->_sfm_class = NULL;
  this->_vertices = NULL;
  this->_number_of_active_threads = 0;
  this->_first_frame = 0;
  this->_last_frame = 0;
  this->_trigger_frame = -1;
  this->_camera = NULL;
  this->_first_execution = true;
  this->_segmentation_publiser = segmentation_publiser;
  this->_shape_3d_publisher = shape_3d_publisher;
  this->_ka_trigger_publisher = ka_trigger_publisher;
  this->_video_width = width;
  this->_video_height = height;

  this->_fts_2d_acc = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_fts_3d_acc = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_fts_2d_acc_cpy = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_fts_3d_acc_cpy = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_fts_2d_total_acc = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_fts_3d_total_acc = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
  this->_sfm_last_result = FeatureSetPtr(new FeatureSet(0));
  for (int i = 0; i < this->_number_features; i++)
  {
    FeaturePtr temp = FeaturePtr(new Feature(-1., -1., -1.));
    temp->setLost(true);
    this->_sfm_last_result->addFeature(temp->clone());
  }
  if(this->_estimate_segmentation)
  {
  this->_composed_image_res = cv::Mat(this->_video_height, 2 * this->_video_width, CV_8UC3, cv::Scalar(0.));
  cv::namedWindow("VisualNet", 1);
  cv::imshow("VisualNet", this->_composed_image_res);
  cv::waitKey(10);
  }
}

VisualNet::~VisualNet()
{
  if (this->_sfm_class)
    delete this->_sfm_class;
  if (this->_camera)
    delete this->_camera;
}

void VisualNet::setSegmentation(vector<int> segmentation)
{
  if (this->_number_features != segmentation.size())
  {
    ROS_ERROR_STREAM("[VisualNet::setSegmentation] The size of the provided segmentation (" << segmentation.size() << ")"
        "is not the expected number of features (" << this->_number_features << ").");
  }
  if (this->_estimate_segmentation)
  {
    ROS_WARN("[VisualNet::setSegmentation] Segmentation provided externally.");
  }
  this->_segmentation_last_result = segmentation;
}

void VisualNet::setSegmentationParameters(double min_motion, double min_global_motion, int min_cluster_size)
{
  this->_min_motion = min_motion;
  this->_min_global_motion = min_global_motion;
  this->_min_cluster_size = min_cluster_size;
}

void VisualNet::setSfMParameters(int width, int height, double focal_length)
{
  this->_camera = new CameraModel(focal_length, width, height);
  SFM::BundlerInitializer* bundler_initializer = new SFM::BundlerInitializer(*this->_camera, this->_min_motion);
  SFM::EKF* ekf = new SFM::EKF(*this->_camera);
  this->_sfm_class = new SFM::SFMClass(bundler_initializer, ekf);
}

void VisualNet::addFeatureSet2D(FeatureSetPtr fs_2d_new)
{
  if (this->_number_features != fs_2d_new->getNumberOfFeatures())
  {
    ROS_ERROR_STREAM("[VisualNet::addFeatureSet2D] The size of the provided FeatureSet (" << fs_2d_new->getNumberOfFeatures() << ")"
        "is not the expected number of features (" << this->_number_features << ").");
  }
  this->_fs_2d_last_frame = fs_2d_new->clone();
  //fs_2d_new->printFS();
  this->_fts_2d_acc->addFeatureSet(fs_2d_new);
  this->_fts_2d_total_acc->addFeatureSet(fs_2d_new);

  if (!this->_segmenting_in_3d)
  {
    // We allow only one thread at time
    if (!this->_external_trigger && this->_automaticTriggerSegmentationAndSfM() && this->_number_of_active_threads == 0)
    {
      //this->_fts_2d_acc->printFTS();
      this->launchSegmentationAndSfMThread();
    }
    else
    {
      this->_last_frame++;
    }
  }
}

void VisualNet::addFeatureSet3D(FeatureSetPtr fs_3d_new)
{
  if (this->_number_features != fs_3d_new->getNumberOfFeatures())
  {
    ROS_ERROR_STREAM("[VisualNet::addFeatureSet3D] The size of the provided FeatureSet (" << fs_3d_new->getNumberOfFeatures() << ")"
        "is not the expected number of features (" << this->_number_features << ").");
  }
  if (this->_estimate_3d_shape)
  {
    ROS_WARN("[VisualNet::addFeatureSet3D] 3D Feature Set provided externally.");
  }
  this->_fs_3d_last_frame = fs_3d_new->clone();
  this->_fts_3d_acc->addFeatureSet(fs_3d_new);
  this->_fts_3d_total_acc->addFeatureSet(fs_3d_new);

  if (this->_segmenting_in_3d)
  {
    // We allow only one thread at time
    if (!this->_external_trigger && this->_automaticTriggerSegmentationAndSfM() && this->_number_of_active_threads == 0)
    {
      this->launchSegmentationAndSfMThread();
    }
    else
    {
      this->_last_frame++;
    }
  }
}

void VisualNet::addRelativeMotionPredictor(double rm_min_dist)
{
  VisualGraph::Predictor* rm_predictor = new VisualGraph::RelativeMotion(rm_min_dist);
  ROS_INFO("[VisualNet::_loadPredictors] Minimum distance to be considered zero relative motion: %f pixels", rm_min_dist);
  _graph.addPredictor(rm_predictor);
}

void VisualNet::addRelativeMotion3DPredictor(double rm3d_min_dist)
{
  VisualGraph::Predictor* rm3d_predictor = new VisualGraph::RelativeMotion3D(rm3d_min_dist);
  ROS_INFO("[VisualNet::_loadPredictors] Minimum distance to be considered zero relative motion: %f meters", rm3d_min_dist);
  _graph.addPredictor(rm3d_predictor);
}

void VisualNet::addFundamentalMatrixPredictor(int fm_num_hypo, int fm_num_trials_per_hypo)
{
  VisualGraph::Predictor* fm_predictor = new VisualGraph::FundamentalMatrix(fm_num_hypo, fm_num_trials_per_hypo);
  _graph.addPredictor(fm_predictor);
}

void VisualNet::addShortDistancePredictor(double sd_min_dist)
{
  if (this->_segmenting_in_3d)
  {
    VisualGraph::Predictor* sd_predictor_3d = new VisualGraph::ShortDistance3D(sd_min_dist);
    _graph.addPredictor(sd_predictor_3d);
  }
  else
  {
    VisualGraph::Predictor* sd_predictor = new VisualGraph::ShortDistance(sd_min_dist);
    _graph.addPredictor(sd_predictor);
  }
}

void VisualNet::addLongDistancePredictor(double ld_min_dist, double ld_max_dist)
{
  if (this->_segmenting_in_3d)
  {
    VisualGraph::Predictor* ld_predictor_3d = new VisualGraph::LongDistance3D(ld_min_dist, ld_max_dist);
    _graph.addPredictor(ld_predictor_3d);
  }
  else
  {
    VisualGraph::Predictor* ld_predictor = new VisualGraph::LongDistance(ld_min_dist, ld_max_dist);
    _graph.addPredictor(ld_predictor);
  }
}

void VisualNet::addTriangulationPredictor()
{
  VisualGraph::Predictor* t_predictor = new VisualGraph::Triangulation();
  _graph.addPredictor(t_predictor);
}

void VisualNet::addColorSegmentationPredictor()
{
  ROS_ERROR("[VisualNet::addColorSegmentationPredictor] Color segmentation predictor is not defined");
}

void VisualNet::addColorSimilarityPredictor()
{
  ROS_ERROR("[VisualNet::addColorSegmentationPredictor] Color similarity predictor is not defined");
}

void VisualNet::reset()
{
  this->_first_execution = true;
  this->_fts_2d_acc->clear();
  this->_fts_3d_acc->clear();
}

void VisualNet::setLastRGBImage(cv::Mat last_image)
{
  this->_last_image = last_image;
}

template<class T, void(T::*mem_fn)()>
  void* segmentationAndSfMThread(void* p)
  {
    (static_cast<T*> (p)->*mem_fn)();
    return 0;
  }

void VisualNet::launchSegmentationAndSfMThread()
{
  this->_accumulator_2d_copy.clear();
  this->_accumulator_2d_copy = this->_fts_2d_acc->FeatureTrajectorySet2Vector();
  this->_accumulator_2d_total_copy.clear();
  this->_accumulator_2d_total_copy = this->_fts_2d_total_acc->FeatureTrajectorySet2Vector();
  if (this->_fts_2d_total_acc_cpy)
    this->_fts_2d_total_acc_cpy->clear();
  this->_fts_2d_total_acc_cpy = this->_fts_2d_total_acc->clone();
  this->_fts_2d_acc_cpy->clear();
  this->_fts_2d_acc_cpy = this->_fts_2d_acc->clone();

  this->_accumulator_3d_copy.clear();
  this->_accumulator_3d_copy = this->_fts_3d_acc->FeatureTrajectorySet2Vector();
  this->_accumulator_3d_total_copy.clear();
  this->_accumulator_3d_total_copy = this->_fts_3d_total_acc->FeatureTrajectorySet2Vector();
  if (this->_fts_3d_total_acc_cpy)
    this->_fts_3d_total_acc_cpy->clear();
  this->_fts_3d_total_acc_cpy = this->_fts_3d_total_acc->clone();
  this->_fts_3d_acc_cpy->clear();
  this->_fts_3d_acc_cpy = this->_fts_2d_acc->clone();

  this->_trigger_frame = this->_last_frame;
  this->_last_frame = 0;
  this->_last_image_cpy = this->_last_image.clone();

  pthread_t this_thread;
  int thread_id = pthread_create(&this_thread, NULL,
                                 segmentationAndSfMThread<VisualNet, &VisualNet::_runSegmentationAndSfM> , (void*)this);

  this->_segmentation_and_sfm_threads.push_back(this_thread);
  this->_thread_ids.push_back(thread_id);
  this->_fs_2d_last_frame->setFrame(0);
  if (this->_fs_3d_last_frame)
  {
    this->_fs_3d_last_frame->setFrame(0);
  }
  this->_fts_2d_acc->clear();
  this->_fts_3d_acc->clear();
}

bool VisualNet::_automaticTriggerSegmentationAndSfM()
{
  vector<double> feature_distances;
  if (!this->_segmenting_in_3d)
  {
    feature_distances = this->_fts_2d_acc->estimateFeaturesMotion();
  }
  else
  {
    feature_distances = this->_fts_3d_acc->estimateFeaturesMotion();
  }
  double global_motion = std::accumulate(feature_distances.begin(), feature_distances.end(), 0);
  int moving_features_ctr = std::count_if(feature_distances.begin(), feature_distances.end(),
                                          std::bind2nd(std::greater_equal<double>(), this->_min_motion));

  if (global_motion > this->_min_global_motion)
  {
    ROS_INFO("[VisualNet::_triggerSegmentationAndSfM] Segmentation triggered by global motion");
    return true;
  }
  if (moving_features_ctr > this->_min_cluster_size)
  {
    ROS_INFO("[VisualNet::_triggerSegmentationAndSfM] Segmentation triggered by individual motion. %d Features are moving.", moving_features_ctr);
    return true;
  }
  return false;
}

void VisualNet::_runSegmentationAndSfM()
{
  this->_number_of_active_threads++;
  if (this->_estimate_segmentation)
  {
    // The real Segmentation Algorithm
    this->_runSegmentation();
    this->_drawSegmentationResults();
  }

  if (this->_estimate_3d_shape)
  {
    // Estimating the 3D coordinates of each Cluster-RigidBody
    this->_runSfM();
  }

  usleep(500); // Needed to assure that the signal comes after the segmentation

  std_msgs::Empty ka_trigger_signal;
  this->_ka_trigger_publisher->publish(ka_trigger_signal);

  this->_number_of_active_threads--;
}

void VisualNet::_runSegmentation()
{
  ROS_INFO("[VisualNet::_runSegmentation] Generating clusters");
  ros::Time t_segm = ros::Time::now();
  if (!this->_estimate_segmentation) // External segmentation should be provided
  {
    if (this->_first_execution)
    {
      // Reassign the new cluster ids to the vector
      _segmentation_last_result.resize(_number_features, -1);
      int vector_index = 0;
      int cluster_id_index = 1;
      for (vector<int>::iterator it_int = _number_feats_per_cluster.begin(); it_int != _number_feats_per_cluster.end(); it_int++)
      {
        for (int i = 0; i < (*it_int); i++)
        {
          this->_segmentation_last_result.at(vector_index) = cluster_id_index;
          vector_index++;
        }
        cluster_id_index++;
      }
      this->_first_execution = false;
    }
  }
  else // Estimate segmentation
  {
    vector<int> non_lost_feat_index; //Stores for every non-lost feature, its index in the general vector of features of size _number_features

    if (this->_first_execution)
    {
      this->_vertices = new VisualGraph::Vertex*[this->_number_features];
      this->_vertex_ids.clear();
      this->_vertex_ids.resize(this->_number_features, -1);

      for (int i = 0; i < this->_number_features; i++)
      {
        vector<FeaturePtr> one_feature_trajectory;

        if (!this->_segmenting_in_3d)
        {
          if (!_accumulator_2d_total_copy.rbegin()->at(i)->isLost())
          {
            for (vector<vector<FeaturePtr> >::iterator it = _accumulator_2d_total_copy.begin(); it
                != _accumulator_2d_total_copy.end(); it++)
            {
              one_feature_trajectory.push_back((*it).at(i)->clone());
            }
            this->_vertices[i] = new VisualGraph::Vertex(one_feature_trajectory);
            this->_vertex_ids.at(i) = this->_vertices[i]->getID();
            this->_graph.addVertex(this->_vertices[i]);
            non_lost_feat_index.push_back(i);
          }
        }
        else
        {
          if (!_accumulator_3d_total_copy.rbegin()->at(i)->isLost())
          {
            for (vector<vector<FeaturePtr> >::iterator it = _accumulator_3d_total_copy.begin(); it
                != _accumulator_3d_total_copy.end(); it++)
            {
              one_feature_trajectory.push_back((*it).at(i)->clone());
            }
            this->_vertices[i] = new VisualGraph::Vertex(one_feature_trajectory);
            this->_vertex_ids.at(i) = this->_vertices[i]->getID();
            this->_graph.addVertex(this->_vertices[i]);
            non_lost_feat_index.push_back(i);
          }
        }
      }
      this->_first_execution = false;
    }
    else //Update the graph
    {
      ROS_INFO("[VisualNet::_runSegmentation] Updating the graph");
      for (int i = 0; i < this->_number_features; i++)
      {
        vector<FeaturePtr> one_feature_trajectory;
        if (_vertex_ids.at(i) != -1)
        {
          if (!this->_segmenting_in_3d)
          {
            if (_accumulator_2d_total_copy.rbegin()->at(i)->isLost())
            {
              ROS_INFO("[VisualNet::_runSegmentation] Removed vertex %d", i);
              _graph.removeVertex(_vertex_ids.at(i));
              _vertex_ids.at(i) = -1;
            }
            else
            {
              for (vector<vector<FeaturePtr> >::iterator it = _accumulator_2d_total_copy.begin(); it
                  != _accumulator_2d_total_copy.end(); it++)
              {
                one_feature_trajectory.push_back((*it).at(i)->clone());
              }
              _graph.updateVertex(_vertex_ids.at(i), one_feature_trajectory);
              non_lost_feat_index.push_back(i);
            }
          }
          else
          {
            if (_accumulator_3d_total_copy.rbegin()->at(i)->isLost())
            {
              ROS_INFO("[VisualNet::_runSegmentation] Removed vertex %d", i);
              _graph.removeVertex(_vertex_ids.at(i));
              _vertex_ids.at(i) = -1;
            }
            else
            {
              for (vector<vector<FeaturePtr> >::iterator it = _accumulator_3d_total_copy.begin(); it
                  != _accumulator_3d_total_copy.end(); it++)
              {
                one_feature_trajectory.push_back((*it).at(i)->clone());
              }
              _graph.updateVertex(_vertex_ids.at(i), one_feature_trajectory);
              non_lost_feat_index.push_back(i);
            }

          }
        }
      }
    }

    vector<VisualGraph::Predictor*> predictors = _graph.getPredictors();
    for (vector<VisualGraph::Predictor*>::iterator pred_it = predictors.begin(); pred_it != predictors.end(); pred_it++)
    {
      (*pred_it)->setFirstFrame(this->_first_frame);
      (*pred_it)->setLastFrame(this->_trigger_frame);
    }
    _graph.predict();
    vector<vector<double> > weights = this->_graph.getWeights();
    //    for (int m = 0; m < weights.size(); m++)
    //    {
    //      for (int n = 0; n < weights.at(m).size(); n++)
    //      {
    //        std::cout << weights.at(m).at(n) << " ";
    //      }
    //      std::cout << std::endl;
    //    }
    int r_colour = 0;
    int g_colour = 0;
    int b_colour = 0;
    int first_ix = 0;
    //cv::Mat cluster_connectivity_img = this->_cv_ptr_rgb->image.clone();
    for (unsigned int i = 0; i < _number_features; i++)
    {
      cv::Mat cluster_connectivity_img = this->_last_image_cpy.clone();
      int second_ix = 0;
      if (!this->_segmenting_in_3d)
      {
        if (!_accumulator_2d_copy.rbegin()->at(i)->isLost())
        {
          //std::cout << _accumulator_2d_copy.rbegin()->at(i)->getX() << " " << _accumulator_2d_copy.rbegin()->at(i)->getY() << std::endl;
          for (unsigned int j = 0; j < _number_features; j++)
          {

            if (!_accumulator_2d_copy.rbegin()->at(j)->isLost())
            {
              //std::cout << first_ix << " " << second_ix <<std::endl;
              if (weights.at(first_ix).at(second_ix) != 0)
              {
                // If I don't want to show the smallest Clusters, I should filter it here
                cv::Point p = cvPoint(cvRound(_accumulator_2d_copy.rbegin()->at(i)->getX()),
                                      cvRound(_accumulator_2d_copy.rbegin()->at(i)->getY()));

                // If I don't want to show the smallest Clusters, I should filter it here
                cv::Point p2 = cvPoint(cvRound(_accumulator_2d_copy.rbegin()->at(j)->getX()),
                                       cvRound(_accumulator_2d_copy.rbegin()->at(j)->getY()));
                cv::line(cluster_connectivity_img, p, p2, CV_RGB(r_colour % 255, g_colour % 255, b_colour % 255),
                         10 * weights.at(first_ix).at(second_ix), CV_AA, 0);

              }
              second_ix++;
            }

          }
          r_colour += 5;
          g_colour += 15;
          b_colour += 22;

          // select a ROI
          cv::Mat roi = this->_composed_image_res(cv::Rect(0, 0, this->_video_width, this->_video_height));
          // fill the ROI with (255, 255, 255) (which is white in RGB space);
          // the original image will be modified
          cluster_connectivity_img.copyTo(roi);
          cv::imshow("VisualNet", this->_composed_image_res);
          //          std::stringstream feature_ss;
          //          feature_ss << i;
          //          std::string file_saving_row = std::string("/home/roberto/Desktop/TestSegmentation/iap_segm_f");
          //          std::string file_total_name = file_saving_row + feature_ss.str() + std::string(".jpg");
          //          cv::imwrite(file_total_name.c_str(), cluster_connectivity_img);
          cv::waitKey(10);
          first_ix++;
        }
      }
      else
      {
        if (!_accumulator_3d_copy.rbegin()->at(i)->isLost())
        {
          //std::cout << _accumulator_2d_copy.rbegin()->at(i)->getX() << " " << _accumulator_2d_copy.rbegin()->at(i)->getY() << std::endl;
          for (unsigned int j = 0; j < _number_features; j++)
          {

            if (!_accumulator_3d_copy.rbegin()->at(j)->isLost())
            {
              //std::cout << first_ix << " " << second_ix <<std::endl;
              if (weights.at(first_ix).at(second_ix) != 0)
              {
                // If I don't want to show the smallest Clusters, I should filter it here
                cv::Point p = cvPoint(cvRound(_accumulator_2d_copy.rbegin()->at(i)->getX()),
                                      cvRound(_accumulator_2d_copy.rbegin()->at(i)->getY()));

                // If I don't want to show the smallest Clusters, I should filter it here
                cv::Point p2 = cvPoint(cvRound(_accumulator_2d_copy.rbegin()->at(j)->getX()),
                                       cvRound(_accumulator_2d_copy.rbegin()->at(j)->getY()));
                cv::line(cluster_connectivity_img, p, p2, CV_RGB(r_colour % 255, g_colour % 255, b_colour % 255),
                         10 * weights.at(first_ix).at(second_ix), CV_AA, 0);

              }
              second_ix++;
            }

          }
          r_colour += 5;
          g_colour += 15;
          b_colour += 22;

          // select a ROI
          cv::Mat roi = this->_composed_image_res(cv::Rect(0, 0, this->_video_width, this->_video_height));
          // fill the ROI with (255, 255, 255) (which is white in RGB space);
          // the original image will be modified
          cluster_connectivity_img.copyTo(roi);
          cv::imshow("VisualNet", this->_composed_image_res);
          //          cv::imshow(CLUSTER_CONNECTIVITY_WDW, cluster_connectivity_img);
          //          std::stringstream feature_ss;
          //          feature_ss << i;
          //          std::string file_saving_row = std::string("/home/roberto/Desktop/TestSegmentation/iap_segm_f");
          //          std::string file_total_name = file_saving_row + feature_ss.str() + std::string(".jpg");
          //          cv::imwrite(file_total_name.c_str(), cluster_connectivity_img);
          cv::waitKey(10);
          first_ix++;
        }
      }

    }
    //_graph.printConnectivity();
    vector<int> non_lost_feature_cluster_ids; //Contains the cluster id for every non-lost feature
    map<int, int> cluster_sizes; //Contains the number of features per cluster
    _graph.HCS(non_lost_feature_cluster_ids, cluster_sizes);

    if (!(non_lost_feature_cluster_ids.size() == non_lost_feat_index.size()))
    {
      std::string
                  error_message(
                                "[VisualNet::_runSegmentation] non_lost_feature_cluster_ids and non_lost_feat_index must have the same size!");
      ROS_ERROR_STREAM(error_message);
      throw error_message;
    }

    // Reassign the new cluster ids to the vector
    this->_segmentation_last_result.resize(_number_features, -1);
    std::stringstream resulting_clusters;
    resulting_clusters << std::string("[VisualNet::_runSegmentation] Resulting clusters: ");
    for (int i = 0; i < non_lost_feature_cluster_ids.size(); i++)
    {
      _segmentation_last_result.at(non_lost_feat_index.at(i)) = non_lost_feature_cluster_ids.at(i);
      resulting_clusters << non_lost_feature_cluster_ids.at(i) << " ";
    }
    ROS_INFO_STREAM(resulting_clusters.str());
  }
  // Rejecting small clusters
  std::map<int, int> counter_clusters; // Count the number of features per cluster
  for (int i = 0; i < this->_number_features; i++)
  {
    if (counter_clusters.find(this->_segmentation_last_result.at(i)) == counter_clusters.end())
    {
      counter_clusters[this->_segmentation_last_result.at(i)] = 1;
    }
    else
    {
      counter_clusters[this->_segmentation_last_result.at(i)] += 1;
    }
  }

  // Publishing the segmentation results////////////////////////
  pcl::PointCloud<pcl::PointXYZ> feature_ids_pc;
  feature_ids_pc.header.stamp = ros::Time::now();
  for (int i = 0; i < this->_number_features; i++)
  {
    pcl::PointXYZ feature_id_temp;
    if (counter_clusters[this->_segmentation_last_result.at(i)] >= this->_min_cluster_size)
    {
      feature_id_temp.x = (float)(this->_segmentation_last_result.at(i));
    }
    else
    {
      feature_id_temp.x = -1.;
    }
    feature_id_temp.y = -1.f;
    feature_id_temp.z = -1.f;
    feature_ids_pc.points.push_back(pcl::PointXYZ(feature_id_temp));
  }
  sensor_msgs::PointCloud2 feature_ids_ros;
  pcl::toROSMsg(feature_ids_pc, feature_ids_ros);
  this->_segmentation_publiser->publish(feature_ids_ros);
  ROS_INFO("[VisualNet::_runSegmentation] Spent %f seconds.",(ros::Time::now() - t_segm).toSec());
}

void VisualNet::_runSfM()
{
  ros::Time t_sfm = ros::Time::now();
  if (this->_estimate_3d_shape)
  {
    ROS_INFO("[VisualNet::SfM] Estimating 3D coordinates of the features in each cluster");

    std::cout << this->_fts_2d_total_acc_cpy->getNumberOfFeatures() << std::endl;
    std::cout << this->_segmentation_last_result.size() << std::endl;

    ClusterTrajectorySetPtr cts_total =
        ClusterTrajectorySetPtr(new ClusterTrajectorySet(this->_fts_2d_total_acc_cpy, this->_segmentation_last_result));

    ClusterTrajectorySetPtr cts = ClusterTrajectorySetPtr(
                                                          new ClusterTrajectorySet(this->_fts_2d_acc_cpy,
                                                                                   this->_segmentation_last_result));

    ClusterSetPtr previous_sfm_results_cs = ClusterSetPtr(
                                                          new ClusterSet(this->_sfm_last_result,
                                                                         this->_segmentation_last_result));

    std::map<int, vector<vector<FeaturePtr> > > sfm_cts_out;

    ClusterSet::iterator c_it = previous_sfm_results_cs->begin();
    ClusterSet::iterator c_it_end = previous_sfm_results_cs->end();
    for (; c_it != c_it_end; c_it++)
    {
      int cluster_id = (*c_it)->getClusterId();
      int num_features = (*c_it)->getNumberOfFeatures();
      vector<vector<FeaturePtr> > sfm_one_ct_out;

      ROS_INFO("[VisualNet::SfM] Cluster %d contains %d Features",cluster_id , num_features);
      if ((*c_it)->getNumberNonLostFeatures() == num_features)
      {
        ROS_INFO("[VisualNet::SfM] Cluster %d moved before. Using previous estimation as initial structure.", cluster_id);
        vector<FeaturePtr> last_this_frame = (*c_it)->FeatureSet2Vector();

        ClusterTrajectoryPtr ct = cts->getClusterTrajectory(cluster_id);
        bool sfm_before_for_all_feats = true;
        FeatureSetPtr last_sfm = FeatureSetPtr(new FeatureSet(0));
        vector<vector<FeaturePtr> > sfm_one_ct_in;

        ClusterTrajectory::trajectory_set_iterator ct_it = ct->begin();
        ClusterTrajectory::trajectory_set_iterator ct_it_end = ct->end();
        for (; ct_it != ct_it_end; ct_it++)
        {
          vector<FeaturePtr> sfm_one_c_in;
          Cluster::iterator c_it = (*ct_it)->begin();
          Cluster::iterator c_it_end = (*ct_it)->end();
          for (; c_it != c_it_end; c_it++)
          {
            // Bundler Adjustment and EKF in SfM needs the 2D coordinates in this format:
            // u' = u
            // v' = imageHeight - v
            // Origin = top-left corner of the image
            // Origin' = bottom-left corner of the image
            sfm_one_c_in.push_back(
                                   (*c_it)->cloneAndUpdate((*c_it)->getX(),
                                                           this->_camera->getYresolution() - (*c_it)->getY()));
          }
          sfm_one_ct_in.push_back(sfm_one_c_in);
        }

        sfm_one_ct_out = _sfm_class->run(sfm_one_ct_in, last_this_frame);

        vector<FeaturePtr>::iterator feats_it = (sfm_one_ct_out.rbegin())->begin();
        vector<FeaturePtr>::iterator feats_it_end = (sfm_one_ct_out.rbegin())->end();
        for (; feats_it != feats_it_end; feats_it++)
        {
          FeatureSet::iterator fs_it = this->_sfm_last_result->begin();
          FeatureSet::iterator fs_it_end = this->_sfm_last_result->end();
          for (; fs_it != fs_it_end; fs_it++)
          {
            if ((*feats_it)->getId() == (*fs_it)->getId())
            {
              (*fs_it)->setPos((*feats_it)->getX(), (*feats_it)->getY(), (*feats_it)->getZ());
            }
          }
        }

      }
      else
      {
        // The cluster didn't move previously enough to run bundler, so we run it in this iteration
        ROS_INFO("[VisualNet::SfM] Cluster %d did NOT moved before. Estimating initial structure with Bundler.", cluster_id);
        ClusterTrajectoryPtr ct = cts_total->getClusterTrajectory(cluster_id);
        FeatureSetPtr last_sfm = FeatureSetPtr(new FeatureSet(0));
        vector<vector<FeaturePtr> > sfm_one_ct_in;

        ClusterTrajectory::trajectory_set_iterator ct_it = ct->begin();
        ClusterTrajectory::trajectory_set_iterator ct_it_end = ct->end();
        for (; ct_it != ct_it_end; ct_it++)
        {
          vector<FeaturePtr> sfm_one_c_in;
          Cluster::iterator c_it = (*ct_it)->begin();
          Cluster::iterator c_it_end = (*ct_it)->end();
          for (; c_it != c_it_end; c_it++)
          {
            // Bundler Adjustment and EKF in SfM needs the 2D coordinates in this format:
            // u' = u
            // v' = imageHeight - v
            // Origin = top-left corner of the image
            // Origin' = bottom-left corner of the image
            sfm_one_c_in.push_back(
                                   (*c_it)->cloneAndUpdate((*c_it)->getX(),
                                                           this->_camera->getYresolution() - (*c_it)->getY()));
          }
          sfm_one_ct_in.push_back(sfm_one_c_in);
        }

        bool moving;
        vector<vector<FeaturePtr> > sfm_one_ct_out_total = _sfm_class->run(sfm_one_ct_in, moving);

        vector<FeaturePtr>::iterator feats_it = (sfm_one_ct_out_total.rbegin())->begin();
        vector<FeaturePtr>::iterator feats_it_end = (sfm_one_ct_out_total.rbegin())->end();
        if (moving)
        {
          for (; feats_it != feats_it_end; feats_it++)
          {
            FeatureSet::iterator fs_it = this->_sfm_last_result->begin();
            FeatureSet::iterator fs_it_end = this->_sfm_last_result->end();
            for (; fs_it != fs_it_end; fs_it++)
            {
              if ((*feats_it)->getId() == (*fs_it)->getId())
              {
                (*fs_it)->setPos((*feats_it)->getX(), (*feats_it)->getY(), (*feats_it)->getZ());
                (*fs_it)->setLost(false);
              }
            }
          }
        }
        // Copy the 3D coordinates in the last frames
        int num_inter_frames = _fts_2d_acc_cpy->getTrajectoryLength();
        for (int interesting_frame = 0; interesting_frame < num_inter_frames; interesting_frame++)
        {
          sfm_one_ct_out.push_back(
                                   sfm_one_ct_out_total.at(
                                                           sfm_one_ct_out_total.size() - (num_inter_frames
                                                               - interesting_frame)));
        }
      }
      sfm_cts_out[cluster_id] = sfm_one_ct_out;
    }

    // Problem: We are changing here the features order, publishing them according to the cluster they belong
    for (int i = 0; i < sfm_cts_out.begin()->second.size(); i++) // Iterates over each frame
    {
      pcl::PointCloud<pcl::PointXYZ> temp_pcl_one_frame; // One point cloud set sent per frame

      FeatureSet::iterator fs_it = this->_sfm_last_result->begin();
      FeatureSet::iterator fs_it_end = this->_sfm_last_result->end();
      for (; fs_it != fs_it_end; fs_it++)
      {
        bool feat_found = false;
        std::map<int, vector<vector<FeaturePtr> > >::iterator total_result_it = sfm_cts_out.begin();
        std::map<int, vector<vector<FeaturePtr> > >::iterator total_result_it_end = sfm_cts_out.end();
        for (; total_result_it != total_result_it_end; total_result_it++) // Iterates over each cluster (on each frame)
        {
          vector<FeaturePtr>::iterator features_it = total_result_it->second.at(i).begin();
          vector<FeaturePtr>::iterator features_it_end = total_result_it->second.at(i).end();
          for (; features_it != features_it_end; features_it++) // Iterates over each feature (on each cluster on each frame)
          {
            if ((*fs_it)->getId() == (*features_it)->getId())
            {
              pcl::PointXYZ temp_point_3d;
              // get the real world values
              if (!(*features_it)->isLost())
              {
                temp_point_3d.x = (*features_it)->getX();
                temp_point_3d.y = (*features_it)->getY();
                temp_point_3d.z = (*features_it)->getZ();
              }
              else
              {
                temp_point_3d.x = -1.;
                temp_point_3d.y = -1.;
                temp_point_3d.z = -1.;
              }

              temp_pcl_one_frame.points.push_back(pcl::PointXYZ(temp_point_3d));
              feat_found = true;
              break;
            }
          }
          if (feat_found)
          {
            break;
          }
        }
        // We assure that the feature_tracker sends always the same number of features: MAX_CORNERS
        // If we couldn't find/track enough, we fill with (-1.0,-1.0,-1.0) until MAX_CORNERS features
        if (!feat_found)
        {
          //std::cout << "Feature not found in frame " << i << std::endl;
          pcl::PointXYZ temp_point_3d(-1.f, -1.f, -1.f);
          temp_pcl_one_frame.points.push_back(pcl::PointXYZ(temp_point_3d));
        }
      }

      sensor_msgs::PointCloud2 temp_ros_one_frame;
      pcl::toROSMsg(temp_pcl_one_frame, temp_ros_one_frame);
      temp_ros_one_frame.header.stamp = ros::Time::now();
      temp_ros_one_frame.header.frame_id = "/world";
      this->_shape_3d_publisher->publish(temp_ros_one_frame);
      usleep(50000); // 50000 us (50 ms) of pause between point clouds
    }
  }
  else
  {
    ROS_INFO("[VisualNet::SfM] Publishing 3D coordinates of the features in each cluster");

    ClusterTrajectorySetPtr cts = ClusterTrajectorySetPtr(
                                                          new ClusterTrajectorySet(this->_fts_3d_acc_cpy,
                                                                                   this->_segmentation_last_result));
    this->_fts_3d_acc_cpy->PublishInROS(this->_shape_3d_publisher, 100);
  }
  ROS_INFO("[VisualNet::_runSfM] Spent %f seconds.",
      (ros::Time::now() - t_sfm).toSec());
}

void VisualNet::_drawSegmentationResults()
{
  cv::Mat clusters_img = this->_last_image_cpy.clone();
  vector<cv::Scalar> colors_strong;
  colors_strong.push_back(cv::Scalar(0, 255, 0, 0));//green
  colors_strong.push_back(cv::Scalar(255, 0, 0, 0));//blue
  colors_strong.push_back(cv::Scalar(0, 0, 255, 0));//red
  colors_strong.push_back(cv::Scalar(0, 0, 0, 0));//black?
  colors_strong.push_back(cv::Scalar(255, 255, 255, 0)); //white?

  for (unsigned int i = 0; i < this->_number_features; i++)
  {
    if ((!_accumulator_2d_copy.rbegin()->at(i)->isLost() && !this->_segmenting_in_3d)
        || (!_accumulator_3d_copy.rbegin()->at(i)->isLost() && this->_segmenting_in_3d))
    {
      // If I don't want to show the smallest Clusters, I should filter it here
      cv::Point p = cvPoint(cvRound(_accumulator_2d_copy.rbegin()->at(i)->getX()),
                            cvRound(_accumulator_2d_copy.rbegin()->at(i)->getY()));
      cv::circle(clusters_img, p, 2, colors_strong.at(this->_segmentation_last_result.at(i) % colors_strong.size()),
                 -1, 20, 0);

    }
    cv::Mat roi = this->_composed_image_res(cv::Rect(this->_video_width, 0, this->_video_width, this->_video_height));
    // fill the ROI with (255, 255, 255) (which is white in RGB space);
    // the original image will be modified
    clusters_img.copyTo(roi);
    //cv::imshow(CLUSTER_CONNECTIVITY_WDW, cluster_connectivity_img);
    cv::imshow("VisualNet", this->_composed_image_res);
    //cv::imshow(CLUSTERS_WDW, clusters_img);
  }
  cv::waitKey(100);
  //cv::imwrite("/home/roberto/Desktop/TestSegmentation/rigid_bodies.jpg", clusters_img);
  ROS_INFO("[VisualNet::_drawRigidBodies] Resulting clusters segmentation plotted.");
}
