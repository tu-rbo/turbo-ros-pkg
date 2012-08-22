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
 * Joint.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#include "Joint.h"

#include "geometry_msgs/Point32.h"

#include <tf/transform_broadcaster.h>

Joint::Joint() :
  _min_motion(0.)
{
}

Joint::Joint(Axis axis) :
  _min_motion(0.), _axis(axis)
{
}

Joint::~Joint()
{
}

Axis Joint::getAxis() const
{
  return this->_axis;
}

std::pair<tf::Transform, tf::Transform> Joint::getTransformationsToOrigin() const
{
  tf::Transform transform_one_dir;
  tf::Transform transform_other_dir;
  transform_one_dir.setOrigin(
                              tf::Vector3(this->_axis.position->getX(), this->_axis.position->getY(),
                                          this->_axis.position->getZ()));
  transform_other_dir.setOrigin(
                                tf::Vector3(this->_axis.position->getX(), this->_axis.position->getY(),
                                            this->_axis.position->getZ()));
  tf::Vector3 x_axis(1.0, 0.0, 0.0);
  tf::Vector3 x_axis_neg(-1.0, 0.0, 0.0);
  vision::FeaturePtr orientation = this->_axis.orientation->clone();
  double ori_norm_val = orientation->norm();
  if (ori_norm_val != 0) // Undefined and Rigid joints have ori = (0,0,0)
  {
    orientation->setX(orientation->getX() / ori_norm_val);
    orientation->setY(orientation->getY() / ori_norm_val);
    orientation->setZ(orientation->getZ() / ori_norm_val);
    tf::Vector3 axis_tf_one_dir = x_axis.cross(
                                               tf::Vector3(orientation->getX(), orientation->getY(),
                                                           orientation->getZ()));
    double angle_tf_one_dir =
        acos(x_axis.dot(tf::Vector3(orientation->getX(), orientation->getY(), orientation->getZ())));
    transform_one_dir.setRotation(tf::Quaternion(axis_tf_one_dir, angle_tf_one_dir));

    tf::Vector3 axis_tf_other_dir = x_axis_neg.cross(
                                                     tf::Vector3(orientation->getX(), orientation->getY(),
                                                                 orientation->getZ()));
    double angle_tf_other_dir = acos(
                                     x_axis_neg.dot(
                                                    tf::Vector3(orientation->getX(), orientation->getY(),
                                                                orientation->getZ())));
    transform_other_dir.setRotation(tf::Quaternion(axis_tf_other_dir, angle_tf_other_dir));
  }
  else
  {
    orientation->setX(1.);
    orientation->setY(0.);
    orientation->setZ(0.);
    transform_one_dir.setRotation(tf::Quaternion(0., 0., 0., 1));
    transform_other_dir.setRotation(tf::Quaternion(0., 1., 0., 0.));
  }

  std::pair<tf::Transform, tf::Transform> ret_tf_pair;
  ret_tf_pair.first = transform_one_dir;
  ret_tf_pair.second = transform_other_dir;

  return ret_tf_pair;
}

std::pair<sensor_msgs::Range, sensor_msgs::Range> Joint::getAxisOrientationUncertaintyMarker() const
{
  sensor_msgs::Range cone_uncertainty_one_dir;
  sensor_msgs::Range cone_uncertainty_other_dir;

  cone_uncertainty_one_dir.min_range = 0.0;
  cone_uncertainty_other_dir.min_range = 0.0;
  cone_uncertainty_one_dir.max_range = 3.0;
  cone_uncertainty_other_dir.max_range = 3.0;
  cone_uncertainty_one_dir.range = 2.99;
  cone_uncertainty_other_dir.range = 2.99;
  cone_uncertainty_one_dir.field_of_view = this->getJointOrientationUncertainty();
  cone_uncertainty_other_dir.field_of_view = this->getJointOrientationUncertainty();

  std::pair<sensor_msgs::Range, sensor_msgs::Range> ret_cones_pair;
  ret_cones_pair.first = cone_uncertainty_one_dir;
  ret_cones_pair.second = cone_uncertainty_other_dir;

  return ret_cones_pair;
}

visualization_msgs::Marker Joint::getAxisMarker() const
{
  ROS_INFO_STREAM_NAMED(
                        "Joint.getAxisMarker",
                        "Axis marker:" << std::endl << "Position: " << this->_axis.position << std::endl
                            << "Orientation: " << this->_axis.orientation);
  visualization_msgs::Marker rviz_marker;
  rviz_marker.header.frame_id = "/world";
  rviz_marker.header.stamp = ros::Time();
  rviz_marker.ns = "my_namespace";
  rviz_marker.id = 0;

  vision::FeaturePtr orientation = this->_axis.orientation->clone();
  double ori_norm_val = orientation->norm();
  rviz_marker.type = visualization_msgs::Marker::ARROW;
  rviz_marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  if (ori_norm_val != 0) // Undefined and Rigid joints have ori = (0,0,0)
  {
    vision::FeaturePtr orientation_norm = orientation / ori_norm_val;
    start_point.x = this->_axis.position->getX() - 10 * orientation_norm->getX();
    start_point.y = this->_axis.position->getY() - 10 * orientation_norm->getY();
    start_point.z = this->_axis.position->getZ() - 10 * orientation_norm->getZ();
    end_point.x = this->_axis.position->getX() + 10 * orientation_norm->getX();
    end_point.y = this->_axis.position->getY() + 10 * orientation_norm->getY();
    end_point.z = this->_axis.position->getZ() + 10 * orientation_norm->getZ();
  }
  else
  {
    start_point.x = 0.;
    start_point.y = 0.;
    start_point.z = 0.;
    end_point.x = 0.;
    end_point.y = 0.;
    end_point.z = 0.;
  }
  rviz_marker.scale.x = 0.03; //Using start and end points, scale.x is the radius of the array body
  rviz_marker.scale.y = 0.03; //Using start and end points, scale.y is the radius of the array head
  rviz_marker.points.push_back(start_point);
  rviz_marker.points.push_back(end_point);
  //marker.scale.z = 0.1;
  rviz_marker.color.a = 1.0;
  rviz_marker.color.r = 1.0;
  rviz_marker.color.g = 0.0;
  rviz_marker.color.b = 0.0;
  return rviz_marker;
}

visualization_msgs::Marker Joint::getAxisPositionUncertaintyMarker() const
{
  visualization_msgs::Marker rviz_marker;
  rviz_marker.header.frame_id = "/world";
  rviz_marker.header.stamp = ros::Time();
  rviz_marker.ns = "my_namespace";
  rviz_marker.id = 0;

  rviz_marker.type = visualization_msgs::Marker::SPHERE;
  rviz_marker.action = visualization_msgs::Marker::ADD;
  rviz_marker.pose.position.x = this->_axis.position->getX();
  rviz_marker.pose.position.y = this->_axis.position->getY();
  rviz_marker.pose.position.z = this->_axis.position->getZ();
  rviz_marker.scale.x = this->getJointPositionUncertainty();
  rviz_marker.scale.y = this->getJointPositionUncertainty();
  rviz_marker.scale.z = this->getJointPositionUncertainty();

  //marker.scale.z = 0.1;
  rviz_marker.color.a = 1.0;
  rviz_marker.color.r = 1.0;
  rviz_marker.color.g = 0.0;
  rviz_marker.color.b = 0.0;
  return rviz_marker;
}

JointPtr Joint::clone() const
{
  return JointPtr(doClone());
}

iap_common::JointMsg Joint::toROSMsg()
{
  iap_common::JointMsg msg;
  msg.axis = _axis.toROSMsg();
  msg.goodnessOfFit = getGoodnessOfFit();
  return msg;
}

void Joint::set(iap_common::JointMsg &)
{
  // FIXME not implemented
}

std::string Joint::JointTypeToString(JointType type)
{
  std::string s;

  switch (type)
  {
    case PRISMATIC_JOINT:
    {
      s = "PRISMATIC";
      break;
    }
    case REVOLUTE_JOINT:
    {
      s = "REVOLUTE";
      break;
    }
    case RIGID_JOINT:
    {
      s = "RIGID";
      break;
    }
    case UNDEFINED_JOINT:
    {
      s = "UNDEFINED";
      break;
    }
    case UNCERTAIN_JOINT:
    {
      s = "UNCERTAIN";
      break;
    }
  }
  return s;
}

void Joint::setMinimumMotion(double min_motion)
{
  this->_min_motion = min_motion;
}
