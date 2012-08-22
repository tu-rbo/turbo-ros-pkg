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
 * feature_tracker.cpp
 *
 *  Created on: Nov 15th, 2011
 *      Author: Roberto Martin
 */

#include "FeatureTracker.h"
#include <iostream>
#include <fstream>

//#include <pcl_ros/point_cloud.h>
#include "pcl_ros/publisher.h"
#include "pcl/io/io.h"
#include "pcl/point_types.h"

#include "sensor_msgs/PointCloud2.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

FeatureTracker::FeatureTracker() :
  _image_transport(_node_handle), _3d_from_sensor(false), _number_of_tracked_features(-1), _quality_level(-1),
      _minimum_dist(-1), _windows_size(-1), _max_level(-1), _max_count(-1), _epsilon(-1), _deriv_lambda(-1),
      _manual_segmentation(false), _first_frame(true)
{

  // We have to delete these parameters in case they were defined before. Otherwise, if we now use another cam/video, the other nodes
  // would read the parameters before we could change them
  this->_node_handle.deleteParam("video_height");
  this->_node_handle.deleteParam("video_width");

  getROSParameter<std::string> ("/video_sensor_type", _video_sensor_type);
  if (_video_sensor_type == std::string("kinect"))
  {
    //subsrcibe to openni msgs
    _cloud_subscriber = _node_handle.subscribe("camera/depth_registered/points", 1,
                                               &FeatureTracker::PointCloudCallback, this);

    _rgb_image_subscriber = _image_transport.subscribe("camera/rgb/image_color", 1, &FeatureTracker::RGBImageCallback,
                                                       this);

    _depth_image_subscriber = _image_transport.subscribe("camera/depth/image", 1, &FeatureTracker::DepthImageCallback,
                                                         this);

    //cv::namedWindow(DEPTHWINDOW);
  }
  else if (_video_sensor_type == std::string("usb_cam"))
  {
    //subscribe to camera msgs
    _rgb_image_subscriber = _image_transport.subscribe("camera/image_raw", 1, &FeatureTracker::RGBImageCallback, this);
  }
  else
  {
    ROS_ERROR_NAMED("FeatureTracker.FeatureTracker", "Wrong parameter \"video_sensor_type\". Value: %s.",
                    _video_sensor_type.c_str());
    throw std::string("[FeatureTracker::FeatureTracker] Wrong parameter \"video_sensor_type\".");

  }
  _feature_set_2d_publisher = _node_handle.advertise<sensor_msgs::PointCloud2> ("iap/feature_set_2d", 1);

  _features_reset_publisher = _node_handle.advertise<std_msgs::Empty> ("iap/feature_set_reset", 100);

  getROSParameter<bool> ("/3d_from_sensor", _3d_from_sensor);
  if (_3d_from_sensor)
  {
    _feature_set_3d_publisher = _node_handle.advertise<sensor_msgs::PointCloud2> ("iap/feature_set_3d", 1);
    //cv::namedWindow(DEPTHWINDOW);
  }

  getROSParameter<bool> ("/manual_segmentation", this->_manual_segmentation);
  if (this->_manual_segmentation)
  {
    this->_segmentation_publisher = this->_node_handle.advertise<sensor_msgs::PointCloud2> ("iap/manual_segmentation",
                                                                                            1);
  }

  getROSParameter<int> ("/number_features", _number_of_tracked_features);
  getROSParameter<double> ("/feature_tracker/quality_level", _quality_level);
  getROSParameter<double> ("/feature_tracker/minimum_dist", _minimum_dist);
  getROSParameter<int> ("/feature_tracker/windows_size", _windows_size);
  getROSParameter<int> ("/feature_tracker/max_level", _max_level);
  getROSParameter<int> ("/feature_tracker/max_count", _max_count);
  getROSParameter<double> ("/feature_tracker/epsilon", _epsilon);
  getROSParameter<double> ("/feature_tracker/deriv_lambda", _deriv_lambda);
  getROSParameter<bool> ("/feature_tracker/manual_selection", _manual_selection);

  //  if (_manual_selection)
  //  {
  //    cv::namedWindow(SELECTWINDOW);
  //  }
  cv::namedWindow( TRACKWINDOW);
  this->reset();
}

FeatureTracker::~FeatureTracker()
{
  //cv::destroyWindow(RGBWINDOW);
  cv::destroyWindow( TRACKWINDOW);
  if (_video_sensor_type == std::string("kinect"))
  {
    //cv::destroyWindow(DEPTHWINDOW);
  }
}

void FeatureTracker::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs)
{
  //convert dataset
  pcl::PointCloud < PointT > temp_cloud;
  pcl::fromROSMsg(*cloud_msgs, temp_cloud);
  _point_cloud = boost::make_shared<const pcl::PointCloud<PointT> >(temp_cloud);
}

void FeatureTracker::RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image_msgs)
{
  _cv_ptr_rgb_bw_old = _cv_ptr_rgb_bw;
  try
  {
    _cv_ptr_rgb = cv_bridge::toCvCopy(rgb_image_msgs, sensor_msgs::image_encodings::BGR8);

    _cv_ptr_rgb_bw = cv_bridge::cvtColor(_cv_ptr_rgb, "mono8");

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[FeatureTracker::RGBImageCallback] cv_bridge exception: %s", e.what());
    return;
  }
  _cv_ptr_rgb_cpy = cv_bridge::cvtColor(_cv_ptr_rgb, "bgr8");

  this->LKTracking();

  //  cv::imshow(RGBWINDOW, _cv_ptr_rgb_cpy->image);
}

void FeatureTracker::DepthImageCallback(const sensor_msgs::ImageConstPtr &depth_image_msgs)
{
  try
  {
    _cv_ptr_depth = cv_bridge::toCvCopy(depth_image_msgs, sensor_msgs::image_encodings::TYPE_32FC1);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_NAMED("FeatureTracker.DepthImageCallback", "cv_bridge exception: %s", e.what());
    return;
  }
  //set fix min max values
  double min_range_ = 0.5;
  double max_range_ = 8.0;

  //create cv;;Mat img
  cv::Mat img(_cv_ptr_depth->image.rows, _cv_ptr_depth->image.cols, CV_8UC1);

  //convert all pixels
  for (int i = 0; i < _cv_ptr_depth->image.rows; i++)
  {
    float* Di = _cv_ptr_depth->image.ptr<float> (i);
    char* Ii = img.ptr<char> (i);
    for (int j = 0; j < _cv_ptr_depth->image.cols; j++)
    {
      Ii[j] = (char)(255 * ((Di[j] - min_range_) / (max_range_ - min_range_)));
    }
  }
  // show the image
  //  cv::imshow(DEPTHWINDOW, img);
  //  cv::waitKey(1);
}

void on_mouse(int event, int x, int y, int flags, void* param)
{
  static int number_clicked_feats = 0;
  static bool dragging_pointer = false;
  static cv::Point2f start_point(-1, -1);
  static cv::Point2f end_point(-1, -1);
  FeatureTracker* feature_tracker_ptr = (FeatureTracker*)param;
  if (feature_tracker_ptr->_cv_ptr_rgb_cpy_mouse)
  {
    if (event == CV_EVENT_LBUTTONDOWN && !dragging_pointer)
    {
      start_point.x = x;
      start_point.y = y;
      dragging_pointer = true;
    }

    /* user drag the mouse */
    if (event == CV_EVENT_MOUSEMOVE && dragging_pointer)
    {
      cv::Mat copy_img = feature_tracker_ptr->_cv_ptr_rgb_cpy_mouse->image.clone();
      cv::rectangle(copy_img, start_point, cv::Point2f(x, y), CV_RGB(255, 255, 0), 3, 8, 0);
      cv::imshow(TRACKWINDOW, copy_img);
    }

    if (event == CV_EVENT_LBUTTONUP && dragging_pointer)
    {
      int selected_points = 0;
      end_point.x = x;
      end_point.y = y;

      if (start_point.x > end_point.x)
      {
        double swap_point = start_point.x;
        start_point.x = end_point.x;
        end_point.x = swap_point;
      }

      if (start_point.y > end_point.y)
      {
        double swap_point = start_point.y;
        start_point.y = end_point.y;
        end_point.y = swap_point;
      }

      // Find the points inside the selected rectangle
      std::vector<cv::Point2f>::const_iterator feat_it = feature_tracker_ptr->_feature_set_vector_2d.begin();
      std::vector<cv::Point2f>::const_iterator feat_it_end = feature_tracker_ptr->_feature_set_vector_2d.end();
      for (; feat_it != feat_it_end; feat_it++)
      {
        // The point is in the selecte region
        if ((feat_it->x >= start_point.x && feat_it->x <= end_point.x) && (feat_it->y >= start_point.y && feat_it->y
            <= end_point.y))
        {
          bool already_selected = false;
          // The point was not selected previously
          std::vector<cv::Point2f>::const_iterator feat_it2 = feature_tracker_ptr->_manual_detector_fs_ret.begin();
          std::vector<cv::Point2f>::const_iterator feat_it2_end = feature_tracker_ptr->_manual_detector_fs_ret.end();
          for (; feat_it2 != feat_it2_end; feat_it2++)
          {
            if (feat_it->x == feat_it2->x && feat_it->y == feat_it2->y)
            {
              already_selected = true;
              break;
            }
          }
          if (!already_selected)
          {
            cv::Point p = cvPoint(cvRound(feat_it->x), cvRound(feat_it->y));
            cv::circle(feature_tracker_ptr->_cv_ptr_rgb_cpy_mouse->image, p, 3, CV_RGB(255, 255, 0), -1, 8, 0);
            feature_tracker_ptr->_manual_detector_fs_ret.push_back(*feat_it);
            cv::imshow(TRACKWINDOW, feature_tracker_ptr->_cv_ptr_rgb_cpy_mouse->image);
            number_clicked_feats++;
            selected_points++;
          }
        }
      }
      ROS_INFO_NAMED("ManualSelection", "Number of features selected in this rectangle: %d", selected_points);
      ROS_INFO_NAMED("ManualSelection", "Total number of features selected: %d", number_clicked_feats);
      dragging_pointer = false;
    }
  }
}

void FeatureTracker::reset()
{
  this->_first_frame = true;
  this->_manual_detector_fs_ret.clear();
  this->_manual_detector_fs_ret.reserve(this->_number_of_tracked_features);
  this->_tracked_feature_set_2d.clear();
  this->_tracked_feature_set_2d.reserve(this->_number_of_tracked_features);
  this->_feature_set_vector_2d.clear();
  this->_feature_set_vector_2d.resize(this->_number_of_tracked_features, cv::Point2f(-1., -1.));
  this->_lost_in_3d.clear();
  this->_lost_in_3d.resize(this->_number_of_tracked_features, false);
  this->_status_3d.clear();
  this->_status_3d.resize(this->_number_of_tracked_features, 0);
  this->_previous_points_3d.clear();
  this->_previous_points_3d.resize(this->_number_of_tracked_features, pcl::PointXYZ(-1., -1., -1.));
  this->_status_2d.clear();
  this->_status_2d.resize(this->_number_of_tracked_features, 1);
  this->_errors.clear();
  this->_errors.resize(this->_number_of_tracked_features, 0.);
}

void FeatureTracker::LKTracking()
{
  pcl::PointCloud < pcl::PointXYZ > feature_set_2d_pcl;
  // In the first loop cycle good features are searched
  if (this->_first_frame)
  {
    int num_features_to_detect = this->_number_of_tracked_features;
    this->_node_handle.setParam("video_height", this->_cv_ptr_rgb->image.rows);
    this->_node_handle.setParam("video_width", this->_cv_ptr_rgb->image.cols);

    cv::goodFeaturesToTrack(this->_cv_ptr_rgb_bw->image, this->_feature_set_vector_2d, num_features_to_detect,
                            this->_quality_level, this->_minimum_dist);

    // MANUAL SELECTION OF THE FEATURES /////////////////////////////////////////////////////////////////////////////
    if (this->_manual_selection)
    {
      this->_cv_ptr_rgb_cpy_mouse = cv_bridge::cvtColor(this->_cv_ptr_rgb, "bgr8");
      // Draw all found features in the image and set the mouse callback to select them manually
      for (unsigned int i = 0; i < this->_feature_set_vector_2d.size(); i++)
      {
        cv::Point p = cvPoint(cvRound(this->_feature_set_vector_2d.at(i).x),
                              cvRound(this->_feature_set_vector_2d.at(i).y));
        cv::circle(this->_cv_ptr_rgb_cpy_mouse->image, p, 2, CV_RGB(255, 0, 0), 1, 8, 0);
      }
      cv::namedWindow( TRACKWINDOW);
      cv::imshow(TRACKWINDOW, this->_cv_ptr_rgb_cpy_mouse->image);
      cv::setMouseCallback(TRACKWINDOW, on_mouse, (void*)this);

      if (this->_manual_segmentation)
      {
        ROS_INFO_NAMED("FeatureTracker.LKTracking", "Select the features on the image by selecting areas."
          "\nPress ENTER to mark the last features to belong to the same cluster."
          "\nPress ESC to finish. \nPress SPACE to stop the execution (bad features).");
      }
      else
      {
        ROS_INFO_NAMED("FeatureTracker.LKTracking",
                       "Select the features on the image by selecting areas. \nPress ESC to finish."
                         "\nPress SPACE to stop the execution (bad features).");
      }

      // Get the values from the image callback
      int feature_index = 0;
      int cluster_id = 1;
      std::vector<int> manual_segmentation;
      while (true)
      {
        char key = cvWaitKey(10);
        if (key == 10 && this->_manual_segmentation) // RETURN
        {
          for (int j = feature_index; j < this->_manual_detector_fs_ret.size(); j++)
          {
            manual_segmentation.push_back(cluster_id);
          }
          ROS_INFO_STREAM_NAMED(
                                "FeatureTracker.LKTracking",
                                "Number of features in cluster " << cluster_id << " : "
                                    << this->_manual_detector_fs_ret.size() - feature_index);
          feature_index = this->_manual_detector_fs_ret.size();
          cluster_id++;
        }
        if (key == 27) // ESCAPE
        {
          ROS_INFO_STREAM_NAMED("FeatureTracker.LKTracking", "Finished!");
          cv::setMouseCallback(TRACKWINDOW, NULL, (void*)this);
          break;
        }
        if (key == 32) // SPACEBAR
        {
          exit(-1);
        }
      }
      this->_feature_set_vector_2d = this->_manual_detector_fs_ret;

      // If the segmentation is manual we publish the results
      if (this->_manual_segmentation)
      {
        // Publishing the segmentation////////////////////////
        pcl::PointCloud < pcl::PointXYZ > cluster_ids_pc;
        cluster_ids_pc.header.stamp = ros::Time::now();
        for (int i = 0; i < this->_number_of_tracked_features; i++)
        {
          pcl::PointXYZ feature_id_temp;
          if (i < manual_segmentation.size())
          {
            feature_id_temp.x = (float)(manual_segmentation.at(i));
          }
          else
          {
            feature_id_temp.x = -1.f;
          }
          feature_id_temp.y = -1.f;
          feature_id_temp.z = -1.f;
          cluster_ids_pc.points.push_back(pcl::PointXYZ(feature_id_temp));
        }
        sensor_msgs::PointCloud2 feature_ids_ros;
        pcl::toROSMsg(cluster_ids_pc, feature_ids_ros);
        this->_segmentation_publisher.publish(feature_ids_ros);
      }
    }
    feature_set_2d_pcl.header.frame_id = "new_features";
    this->_features_reset_publisher.publish(_reset_signal);
    // We copy the detected features into the vector of tracked features to publish it
    this->_tracked_feature_set_2d = this->_feature_set_vector_2d;
  }
  else
  {
    // clear all vectors
    this->_tracked_feature_set_2d .clear();
    this->_status_2d.clear();
    this->_errors.clear();

    // run LK tracking
    cv::calcOpticalFlowPyrLK(this->_cv_ptr_rgb_bw_old->image, this->_cv_ptr_rgb_bw->image,
                             this->_feature_set_vector_2d, this->_tracked_feature_set_2d, this->_status_2d,
                             this->_errors, cv::Size(this->_windows_size, this->_windows_size), this->_max_level,
                             cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, this->_max_count, this->_epsilon),
                             this->_deriv_lambda);

  }

  feature_set_2d_pcl.header.stamp = ros::Time::now();

  //Publishing 2D features
  for (unsigned int i = 0; i < this->_number_of_tracked_features; i++)
  {
    pcl::PointXYZ temp_point_2d;
    temp_point_2d.x = -1.f;
    temp_point_2d.y = -1.f;
    temp_point_2d.z = -1.f;
    if (i < this->_tracked_feature_set_2d.size() && (this->_status_2d.at(i) != 0))
    {
      temp_point_2d.x = this->_tracked_feature_set_2d.at(i).x;
      temp_point_2d.y = this->_tracked_feature_set_2d.at(i).y;
      cv::Point p = cvPoint(cvRound(this->_tracked_feature_set_2d.at(i).x),
                            cvRound(this->_tracked_feature_set_2d.at(i).y));
      cv::circle(_cv_ptr_rgb->image, p, 2, CV_RGB(255, 0, 0), 1, 8, 0);
    }
    feature_set_2d_pcl.points.push_back(pcl::PointXYZ(temp_point_2d));
  }

  sensor_msgs::PointCloud2 feature_set_2d_ros;
  pcl::toROSMsg(feature_set_2d_pcl, feature_set_2d_ros);
  this->_feature_set_2d_publisher.publish(feature_set_2d_ros);

  if (this->_point_cloud && this->_3d_from_sensor)
  {
    this->TrackIn3D();
  }

  this->_feature_set_vector_2d = this->_tracked_feature_set_2d;

  cv::imshow(TRACKWINDOW, _cv_ptr_rgb->image);
  cv::waitKey(1);
  this->_first_frame = false;
}

void FeatureTracker::TrackIn3D()
{
  pcl::PointCloud < pcl::PointXYZ > feature_set_3d_pcl;
  feature_set_3d_pcl.header.stamp = ros::Time::now();
  // Run over all interest points and check if the xyz coordinate is valid and store them
  for (unsigned int i = 0; i < this->_number_of_tracked_features; i++)
  {
    pcl::PointXYZ temp_point_3d;
    temp_point_3d.x = -1.f;
    temp_point_3d.y = -1.f;
    temp_point_3d.z = -1.f;

    // If the point was NOT lost before (more than 2 consecutive frames without data in 3D)
    if (this->_status_3d.at(i) < 3)
    {
      // If the point was tracked
      if (i < this->_tracked_feature_set_2d.size() && (this->_status_2d.at(i) != 0))
      {
        // Get the pixel index
        int idx = (cvRound(this->_tracked_feature_set_2d .at(i).y) * this->_point_cloud->width
            + cvRound(this->_tracked_feature_set_2d .at(i).x));

        // If the data from Kinect for this feature is RIGHT
        if (!isnan(this->_point_cloud->points[idx].x) && !isnan(this->_point_cloud->points[idx].y)
            && !isnan(this->_point_cloud->points[idx].z))
        {
          // get the real world values
          temp_point_3d.x = _point_cloud->points[idx].x;
          temp_point_3d.y = _point_cloud->points[idx].y;
          temp_point_3d.z = _point_cloud->points[idx].z;
          this->_previous_points_3d.at(i) = pcl::PointXYZ(temp_point_3d);
          this->_status_3d.at(i) = 0;
        }
        else // The point was lost in 3D in less than 2 consecutive frames
        {
          if (!this->_first_frame)
          {
            temp_point_3d = this->_previous_points_3d.at(i);
            this->_status_3d.at(i) = this->_status_3d.at(i) + 1;
          }
          else // The point was lost and we couldn't store a previous value
          {
            temp_point_3d.x = -2.;
            temp_point_3d.y = -2.;
            temp_point_3d.z = -2.;
            this->_status_3d.at(i) = 3;
            // We don't want to track this point in 2D anymore!!!!!!!!!
            this->_status_2d.at(i) = 0;
          }
        }
      }
    }
    else // The point was lost in 3D in more than 2 consecutive frames
    {
      temp_point_3d.x = -2.;
      temp_point_3d.y = -2.;
      temp_point_3d.z = -2.;
      this->_status_2d.at(i) = 0;
    }
    feature_set_3d_pcl.points.push_back(pcl::PointXYZ(temp_point_3d));
  }

  // Publish to ROS
  sensor_msgs::PointCloud2 feature_set_3d_ros;
  pcl::toROSMsg(feature_set_3d_pcl, feature_set_3d_ros);
  feature_set_3d_ros.header.stamp = ros::Time::now();
  feature_set_3d_ros.header.frame_id = "/world";
  this->_feature_set_3d_publisher.publish(feature_set_3d_ros);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_tracker");
  FeatureTracker p;
  ros::spin();

  return (0);
}
