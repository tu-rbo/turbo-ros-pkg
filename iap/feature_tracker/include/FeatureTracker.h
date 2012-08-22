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
 * feature_tracker.h
 *
 *  Created on: Nov 15th, 2011
 *      Author: Roberto Martin
 */

#ifndef FEATURE_TRACKER_ROS_H_
#define FEATURE_TRACKER_ROS_H_

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "std_msgs/Empty.h"

#include "pcl_ros/publisher.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "pcl/io/io.h"
#include "pcl/point_types.h"

#include "FeatureSet.h"
#include "image.h"

using namespace cv_bridge;

static const char RGBWINDOW[] = "RGB Image";
static const char DEPTHWINDOW[] = "Depth Channel";
static const char TRACKWINDOW[] = "Point Features";
static const char SELECTWINDOW[] = "Select Features";

const int MAX_CORNERS = 500;

class FeatureTracker
{

  typedef pcl::PointXYZ PointT;

public:

  FeatureTracker();

  virtual
  ~FeatureTracker();

  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);

  void RGBImageCallback(const sensor_msgs::ImageConstPtr &rgb_image);

  void DepthImageCallback(const sensor_msgs::ImageConstPtr &depth_image);

  void LKTracking();

  void reset();

  void TrackIn3D();

  pcl::PointCloud<PointT>::ConstPtr     _point_cloud;
  cv_bridge::CvImagePtr                 _cv_ptr_rgb_cpy_mouse;
  std::vector<cv::Point2f>              _manual_detector_fs_ret;
  std::vector<cv::Point2f>              _feature_set_vector_2d;
  std::vector<cv::Point2f>              _tracked_feature_set_2d;

private:
  // ROS communication
  ros::NodeHandle _node_handle;
  ros::Subscriber _cloud_subscriber;
  ros::Publisher _feature_set_2d_publisher;
  ros::Publisher _feature_set_3d_publisher;
  ros::Publisher _segmentation_publisher;
  ros::Publisher _features_reset_publisher;
  image_transport::ImageTransport _image_transport;
  image_transport::Subscriber _rgb_image_subscriber;
  image_transport::Subscriber _depth_image_subscriber;
  std_msgs::Empty _reset_signal;         //Message to be sent in order to inform the other nodes that a new set of features was detected and will be tracked

  // OpenCV images
  cv_bridge::CvImagePtr _cv_ptr_rgb;
  cv_bridge::CvImagePtr _cv_ptr_rgb_cpy;
  cv_bridge::CvImagePtr _cv_ptr_rgb_bw;
  cv_bridge::CvImagePtr _cv_ptr_rgb_bw_old;
  cv_bridge::CvImagePtr _cv_ptr_depth;
  cv_bridge::CvImagePtr _cv_ptr_depth_mono;

  // Tracking variables
  bool                          _first_frame;
  std::vector<bool>             _lost_in_3d;
  std::vector<int>              _status_3d;
  std::vector<pcl::PointXYZ>    _previous_points_3d;
  std::vector<uchar>            _status_2d;
  std::vector<float>            _errors;

  // Parameters from ROS
  bool _manual_selection;
  bool _manual_segmentation;
  std::string _video_sensor_type;
  bool _3d_from_sensor;
  int _number_of_tracked_features;

  // Parameters from ROS for feature_tracker
  double _quality_level;
  double _minimum_dist;
  int _windows_size;
  int _max_level;
  int _max_count;
  double _epsilon;
  double _deriv_lambda;

  template <class T>
  bool getROSParameter(std::string param_name, T & param_container)
  {
    if (!(_node_handle.getParam(param_name, param_container)))
        {
          ROS_ERROR_NAMED(
              "FeatureTracker.getROSParameter", "The parameter %s can not be found.", param_name.c_str());
          throw (std::string(
              "[FeatureTracker::getROSParameter] The parameter can not be found. Parameter name: ") + param_name);
          return false;
        }
    else
      return true;
  }

};

#endif /* FEATURE_TRACKER_ROS_H_ */
