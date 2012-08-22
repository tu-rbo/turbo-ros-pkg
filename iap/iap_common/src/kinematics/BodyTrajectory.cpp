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
 * BodyTrajectory.cpp
 *
 *  Created on: Apr 12, 2012
 *      Author: roberto
 */

#include "BodyTrajectory.h"

#include <iostream>
#include <fstream>

#define ROT_ANGLE_THRESHOLD 0.01        // Radians
#define TRANS_DIST_THRESHOLD 0.01       // Meters
BodyTrajectory::BodyTrajectory()
{
  // TODO Auto-generated constructor stub

}

BodyTrajectory::BodyTrajectory(const BodyTrajectory& bt)
{
  this->_time_start = bt._time_start;
  this->_transformations = bt._transformations;
  this->_body_id = bt._body_id;
  this->_rot_params_estimated = bt._rot_params_estimated;
  this->_acc_rot_angle = bt._acc_rot_angle;
  this->_max_rot_angle = bt._max_rot_angle;
  this->_rot_axis_parall = bt._rot_axis_parall;
  this-> _trans_params_estimated = bt._trans_params_estimated;
  this->_acc_trans_dist = bt._acc_trans_dist;
  this->_max_trans_dist = bt._max_trans_dist;
  this->_trans_axis_parall = bt._trans_axis_parall;
  this->_rot_axis_distance = bt._rot_axis_distance;
}

BodyTrajectory::BodyTrajectory(int time_start, std::vector<RigidTransformationPtr> transformations, int body_id) :
  _time_start(time_start), _transformations(transformations), _body_id(body_id), _rot_params_estimated(false),
      _acc_rot_angle(0), _max_rot_angle(0), _rot_axis_parall(0), _trans_params_estimated(false), _acc_trans_dist(0),
      _max_trans_dist(0), _trans_axis_parall(0), _max_rot_frame(0), _max_trans_frame(0), _rot_axis_distance(0)
{

}

BodyTrajectory::~BodyTrajectory()
{
  // TODO Auto-generated destructor stub
}

int BodyTrajectory::getTimeStart() const
{
  return this->_time_start;
}

int BodyTrajectory::getTimeEnd() const
{
  int time_end = this->_time_start + this->_transformations.size() - 1;
  return time_end;
}

int BodyTrajectory::getTrajectoryLength() const
{
  return this->_transformations.size();
}

int BodyTrajectory::getBodyId() const
{
  return this->_body_id;
}

void BodyTrajectory::estimateRotationParameters()
{
  std::vector<RigidTransformationPtr>::iterator rt_it = this->_transformations.begin() + 1;
  std::vector<RigidTransformationPtr>::iterator rt_end = this->_transformations.end();

  /* First we estimate the mean orientation of the axis of rotation:
   * To do this, we estimate the orientation of the rotation at every time step and we weight this value by the angle of rotation.
   * That means, large rotation will have higher weight in the mean value
   */
  Eigen::Vector3d mean_rot_axis = Eigen::Vector3d(0., 0., 0.);
  for (; rt_it != rt_end; rt_it++)
  {
    double rot_angle = fabs((*rt_it)->getRotationAngle());
    if (rot_angle != 0.0)
    {
      Eigen::Vector3d rot_axis = (*rt_it)->getRotationAxis();
      mean_rot_axis += (rot_angle * rot_axis);
    }
  }
  // Re-normalize the mean rotation axis:
  mean_rot_axis.normalize();

  /*
   * Then, we estimate the deviation of each rotation axis wrt the mean rotation axis, also weighted by the amount of rotation (angle)
   * and normalized at the end -> The result will be between 0 and 1
   */
  rt_it = this->_transformations.begin() + 1;
  this->_rot_axis_parall = 0;
  int frames_count = 1;
  double accumula_rot = 0.0;
  for (; rt_it != rt_end; rt_it++)
  {
    double rot_angle = fabs((*rt_it)->getRotationAngle());
    if (this->_max_rot_angle < rot_angle)
    {
      this->_max_rot_angle = rot_angle;
      this->_max_rot_frame = frames_count;
    }
    if (rot_angle != 0.0)
    {
      Eigen::Vector3d rot_axis = (*rt_it)->getRotationAxis();
      this->_rot_axis_parall += rot_angle * fabs(mean_rot_axis.dot(rot_axis));
      accumula_rot += rot_angle;
    }
    frames_count++;
  }
  this->_rot_axis_parall /= accumula_rot;

  /*
   * Second, we estimate the mean position of the axis of rotation:
   * To do this, we estimate the position of the axis at every time step and we weight this value by the angle of rotation.
   * That means, large rotation will have higher weight in the mean value
   */
  rt_it = this->_transformations.begin() + 1;
  Eigen::Vector3d mean_rot_pos = Eigen::Vector3d(0., 0., 0.);
  frames_count = 1;
  for (; rt_it != rt_end; rt_it++)
  {
    double rot_angle = fabs((*rt_it)->getRotationAngle());
    if (rot_angle != 0.0)
    {
      Eigen::Vector3d rot_pos = (*rt_it)->getRotationAxisPosition();
      mean_rot_pos += (rot_angle * rot_pos);
      frames_count++;
    }
  }
  // Re-normalize the mean rotation axis:
  mean_rot_pos /= frames_count;

  /*
   * Then, we estimate the distance between the position of each rotation axis and the mean value, also weighted by the amount of rotation (angle)
   * and finally divided by the number of frames
   */
  rt_it = this->_transformations.begin() + 1;
  this->_rot_axis_distance = 0.;
  frames_count = 1;
  accumula_rot = 0.0;
  for (; rt_it != rt_end; rt_it++)
  {
    double rot_angle = fabs((*rt_it)->getRotationAngle());
    if (rot_angle != 0.0)
    {
      this->_rot_axis_distance += rot_angle*(mean_rot_pos - (*rt_it)->getRotationAxisPosition()).norm();
      std::cout << (*rt_it)->getRotationAxisPosition().x() << " " << (*rt_it)->getRotationAxisPosition().y() << " " << (*rt_it)->getRotationAxisPosition().z() << std::endl;
      accumula_rot += rot_angle;
    }
  }
  this->_rot_axis_distance /= accumula_rot;

  this->_rot_params_estimated = true;
  std::cout << "----------------------------------------------- \n rot para: " << this->_rot_axis_parall << "\n rot transla: " << this->_rot_axis_distance << std::endl ;
}

void BodyTrajectory::estimateTranslationParameters()
{
  Eigen::Vector3d previous_translation(0., 0., 0.);
  Eigen::Vector3d accumulated_translation(0., 0., 0.);
  std::vector<Eigen::Vector3d> translation_axes;
  std::vector<RigidTransformationPtr>::iterator rt_it = this->_transformations.begin() + 1;
  std::vector<RigidTransformationPtr>::iterator rt_end = this->_transformations.end();

  Eigen::Vector3d mean_translation_ori(0., 0., 0.);
  int frame = 1;
  double normi = 0.0;
  for (; rt_it != rt_end; rt_it++)
  {
    Eigen::Vector3d current_translation = (*rt_it)->getTranslation();
    ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters", "Translation: " << current_translation.x() << " "
        << current_translation.y() << " " << current_translation.z() );;
    double translation_distance = current_translation.norm();

    if (this->_max_trans_dist < translation_distance)
    {
      this->_max_trans_dist = translation_distance;
      this->_max_trans_frame = frame;
    }

    mean_translation_ori += current_translation;

    Eigen::Vector3d prev_current_diff = current_translation - previous_translation;
    // If the cluster remains in the same position, consecutive frames will have the same translation, the difference will be the
    // vector zero, and the norm of it is nan!
    if (prev_current_diff != Eigen::Vector3d(0., 0., 0.))
    {
      this->_acc_trans_dist += prev_current_diff.norm();
      translation_axes.push_back(prev_current_diff);
      previous_translation = current_translation;
      accumulated_translation += current_translation;
    }
    frame++;
  }

  mean_translation_ori.normalize();

  rt_it = this->_transformations.begin() + 1;
  this->_trans_axis_parall = 0.;
  int frames_count = 1;
  double acc_transli = 0.0;
  for (; rt_it != rt_end; rt_it++)
  {
    double translation_value = (*rt_it)->getTranslation().norm();

    Eigen::Vector3d current_translation = (*rt_it)->getTranslation();
    current_translation.normalize();
    this->_trans_axis_parall += translation_value * fabs(mean_translation_ori.dot(current_translation));
    frames_count++;
    acc_transli += translation_value;

  }
  this->_trans_axis_parall /= acc_transli;

  std::cout << "----------------------------------------------- \n trans para: " << this->_trans_axis_parall <<  std::endl ;

  //  if (translation_axes.size() > 2)
  //  {
  //    double normalizing_factor = 0;
  //    accumulated_translation /= translation_axes.size();
  //    double accumulated_trans_norm = accumulated_translation.norm();
  //    Eigen::Vector3d acc_trans_norm = accumulated_translation / accumulated_trans_norm;
  //    ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //        "mean_dir (norm): " << acc_trans_norm);
  //    std::vector<Eigen::Vector3d>::iterator translation_axes_it = translation_axes.begin() + 1;
  //    std::vector<Eigen::Vector3d>::iterator translation_axes_it_end = translation_axes.end();
  //    for (; translation_axes_it != translation_axes_it_end; translation_axes_it++)
  //    {
  //      double min_distance = (accumulated_trans_norm < translation_axes_it->norm() ? accumulated_trans_norm
  //          : translation_axes_it->norm());
  //
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "v_i " << *(translation_axes_it));
  //
  //      Eigen::Vector3d v_i_norm = *translation_axes_it / (translation_axes_it)->norm();
  //
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "v_i (norm): " << v_i_norm);
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "v_i * mean_dir (norm): " << (acc_trans_norm.dot(v_i_norm)));
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "fabs(v_i * mean_dir (norm): " << fabs(acc_trans_norm.dot(v_i_norm)));
  //
  //      double current_trans_axis_dot = fabs(acc_trans_norm.dot(v_i_norm)) * min_distance; // Sum of the angles of deviation between axis of rotation
  //
  //      normalizing_factor += min_distance;
  //      this->_trans_axis_parall += current_trans_axis_dot;
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "min_distance: " << min_distance);
  //      ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
  //          "current_trans_axis_dot: " << current_trans_axis_dot);
  //    }
  //    // Normalize the axis deviation value
  //    this->_trans_axis_parall /= normalizing_factor;
  //  }
  //  else
  //  {
  //    ROS_WARN_STREAM("[BodyTrajectory::estimateTranslationParameters] Not enough frames for estimating the parallelism value for the axis of translation");
  //    this->_trans_axis_parall = 0.0;
  //  }


  this->_trans_params_estimated = true;
}

double BodyTrajectory::getAccRotationAngle()
{
  if (!this->_rot_params_estimated)
  {
    this->estimateRotationParameters();
  }
  return this->_acc_rot_angle;
}

double BodyTrajectory::getMaxRotationAngle()
{
  if (!this->_rot_params_estimated)
  {
    this->estimateRotationParameters();
  }
  return this->_max_rot_angle;
}

double BodyTrajectory::getRotationAxisParallelism()
{
  if (!this->_rot_params_estimated)
  {
    this->estimateRotationParameters();
  }
  return this->_rot_axis_parall;
}

double BodyTrajectory::getAccTranslationDistance()
{
  if (!this->_trans_params_estimated)
  {
    this->estimateTranslationParameters();
  }
  return this->_acc_trans_dist;
}

double BodyTrajectory::getMaxTranslationDistance()
{
  if (!this->_trans_params_estimated)
  {
    this->estimateTranslationParameters();
  }
  return this->_max_trans_dist;
}

double BodyTrajectory::getTranslationAxisParallelism()
{
  if (!this->_trans_params_estimated)
  {
    this->estimateTranslationParameters();
  }
  return this->_trans_axis_parall;
}

RigidTransformationPtr BodyTrajectory::getRigidTransformation(int frame) const
{
  if (frame < this->_time_start || frame > this->getTimeEnd())
  {
    ROS_ERROR_STREAM_NAMED ("BodyTrajectory.getRigidTransformation",
        "Frame " << frame << " is either under " << this->_time_start << " or over " <<this->getTimeEnd());
  }

  return this->_transformations.at(frame - this->_time_start);
}

std::vector<RigidTransformationPtr> BodyTrajectory::getRigidTransformations() const
{
  return this->_transformations;
}

BodyTrajectoryPtr BodyTrajectory::clone() const
{
  return BodyTrajectoryPtr(doClone());
}

void BodyTrajectory::writeToFileBT(std::string file_name) const
{
  std::vector<RigidTransformationPtr>::const_iterator bt_it = this->_transformations.begin();
  if (bt_it != this->_transformations.end())
  {
    // The first time it is written, the rest of the times it is appended
    (*bt_it)->writeToFileRT(file_name);
    bt_it++;
    for (; bt_it != this->_transformations.end(); bt_it++)
    {
      (*bt_it)->appendToFileRT(file_name);
    }
  }
  else
  {
    ROS_ERROR_NAMED("BodyTrajectory.writeToFileBT","Nothing wrote to file. BodyTrajectory is empty.");
  }
}

iap_common::BodyTrajectoryMsg BodyTrajectory::toROSMsg()
{
  iap_common::BodyTrajectoryMsg msg;

  msg.time_start = getTimeStart();
  msg.time_end = getTimeEnd();
  msg.body_id = getBodyId();

  msg.acc_rotation_angle = getAccRotationAngle();
  msg.max_rotation_angle = getMaxRotationAngle();
  msg.rotation_axis_parall = getRotationAxisParallelism();

  msg.acc_translation_dist = getAccTranslationDistance();
  msg.max_translation_dist = getMaxTranslationDistance();
  msg.translation_axis_parall = getTranslationAxisParallelism();

  return msg;
}

void BodyTrajectory::set(iap_common::BodyTrajectoryMsg&)
{
  // FIXME not implemented
}

RigidTransformationPtr BodyTrajectory::getRTwithMaxRotationAngle()
{
  if (!this->_rot_params_estimated)
  {
    this->estimateRotationParameters();
  }
  ROS_INFO_STREAM_NAMED("BodyTrajectory.getRTwithMaxRotationAngle", "Maximum rotation in frame " << this->_max_rot_frame);
  return this->getRigidTransformation(this->_max_rot_frame);
}

RigidTransformationPtr BodyTrajectory::getRTwithMaxTranslationDistance()
{
  if (!this->_trans_params_estimated)
  {
    this->estimateTranslationParameters();
  }
  ROS_INFO_STREAM_NAMED("BodyTrajectory.getRTwithMaxTranslationDistance", "Maximum translation in frame " << this->_max_trans_frame);
  return this->getRigidTransformation(this->_max_trans_frame);
}

std::vector<double> BodyTrajectory::getSimilaritiesToI() const
{
  std::vector<RigidTransformationPtr>::const_iterator rt_it = this->_transformations.begin();
  std::vector<RigidTransformationPtr>::const_iterator rt_end = this->_transformations.end();
  int acc_T_is_I = 0;
  int acc_R_is_I = 0;
  int acc_t_is_0 = 0;
  int number_frames = 0;
  int number_moving_frames = 0;
  for (; rt_it != rt_end; rt_it++)
  {
    Eigen::Matrix4f transf_matrix = (*rt_it)->getTransformationMatrix().matrix().cast<float> ();
    int T_is_I = transf_matrix.isIdentity(2 * 1e-2);
    acc_T_is_I += T_is_I;
    if (!T_is_I)
    {
      Eigen::Matrix3f rot_matrix = transf_matrix.block<3, 3> (0, 0);
      acc_R_is_I += rot_matrix.isIdentity(3 * 1e-2);
      Eigen::Vector3d current_translation = (*rt_it)->getTranslation();
      acc_t_is_0 += current_translation.isZero(1e-2);
      number_moving_frames++;
    }
    number_frames++;
  }
  std::vector<double> ret_vector;
  ret_vector.push_back((double)acc_T_is_I / (double)number_frames);
  if (number_moving_frames)
  {
    ret_vector.push_back((double)acc_R_is_I / (double)number_moving_frames);
    ret_vector.push_back((double)acc_t_is_0 / (double)number_moving_frames);
  }
  else
  {
    ret_vector.push_back(-1.);
    ret_vector.push_back(-1.);
  }
  return ret_vector;
}

/* Estimate Rotation Parameters old code:
 *
 *
 std::vector<Eigen::AngleAxisd> rotation_angle_axes;
 Eigen::Vector3d mean_rot_axis(0., 0., 0.);
 std::vector<RigidTransformationPtr>::iterator rt_it = this->_transformations.begin() + 1;
 std::vector<RigidTransformationPtr>::iterator rt_end = this->_transformations.end();

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "this->_transformations " << this->_transformations.size());

 // stores the prev rt_it transformation
 RigidTransformationPtr rt_prev;

 int frame = 1;
 for (; rt_it != rt_end; rt_it++)
 {
 // transformation between t-1 and t
 RigidTransformationPtr rt_tm1_t;

 if (rt_prev)
 {
 rt_tm1_t = *rt_it - rt_prev;
 }
 else
 {
 rt_tm1_t = *rt_it;
 }

 // angle from 0 to t
 double rot_angle_0_t = (*rt_it)->getRotationAngle();
 // angle from t-1 to t
 double rot_angle_tm1_t = fabs(rt_tm1_t->getRotationAngle());
 // don't allow the angle to be higher than pi (assume small motions)
 if (rot_angle_tm1_t > M_PI)
 {
 // FIXME what if rot_angle_tm1_t > 2 pi?
 rot_angle_tm1_t = 2 * M_PI - rot_angle_tm1_t;
 }

 // this captures the current angle configuration
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "---------");

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "rt_it:  axis.z --> " << (*rt_it)->getRotationAxis()[2]);
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "rt_it:  angle  --> " << (*rt_it)->getRotationAngle());

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "cur_rt:  axis.z --> " << rt_tm1_t->getRotationAxis()[2]);
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "cur_rt:  angle  --> " << rt_tm1_t->getRotationAngle());

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateRotationParameters",
 "---------");

 //    this->_max_rot_angle = (this->_max_rot_angle < fabs(rot_angle)) ? fabs(rot_angle) : this->_max_rot_angle;
 if (this->_max_rot_angle < fabs(rot_angle_0_t))
 {
 this->_max_rot_angle = fabs(rot_angle_0_t);
 this->_max_rot_frame = frame;
 }

 this->_acc_rot_angle += rot_angle_tm1_t;

 // FIXME do we keep the rot axis of the accumulated rot?
 Eigen::AngleAxisd angle_axis = (*rt_it)->getRotationAngleAxis();
 rotation_angle_axes.push_back(angle_axis);
 mean_rot_axis += angle_axis.axis();

 rt_prev = *rt_it;

 frame++;
 }
 if (rotation_angle_axes.size() > 2)
 {
 mean_rot_axis /= mean_rot_axis.norm();
 double normalizing_factor = 0;
 std::vector<Eigen::AngleAxisd>::iterator angle_axis_it = rotation_angle_axes.begin();
 std::vector<Eigen::AngleAxisd>::iterator angle_axis_it_end = rotation_angle_axes.end();
 for (; angle_axis_it != angle_axis_it_end; angle_axis_it++)
 {
 double min_angle = angle_axis_it->angle();
 double current_rot_axis_dot = fabs(mean_rot_axis.dot(angle_axis_it->axis())) * min_angle; // Sum of the angles of deviation between axis of rotation
 normalizing_factor += min_angle;
 this->_rot_axis_parall += current_rot_axis_dot;
 }
 // Normalize the axis deviation value
 this->_rot_axis_parall /= normalizing_factor;
 }
 else
 {
 ROS_WARN_STREAM("[BodyTrajectory::estimateRotationParameters] Not enough frames for estimating the parallelism value for the axis of rotation");
 this->_rot_axis_parall = 0.0;
 }
 this->_rot_params_estimated = true;

 */

/* OLD CODE ESTIMATE TRANSLATION PARAMETERS
 *
 Eigen::Vector3d previous_translation(0., 0., 0.);
 Eigen::Vector3d accumulated_translation(0., 0., 0.);
 std::vector<Eigen::Vector3d> translation_axes;
 std::vector<RigidTransformationPtr>::iterator rt_it = this->_transformations.begin() + 1;
 std::vector<RigidTransformationPtr>::iterator rt_end = this->_transformations.end();

 int frame = 1;
 for (; rt_it != rt_end; rt_it++)
 {
 Eigen::Vector3d current_translation = (*rt_it)->getTranslation();
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters", "Translation: " << current_translation.x() << " "
 << current_translation.y() << " " << current_translation.z() );;
 double translation_distance = current_translation.norm();

 if (this->_max_trans_dist < translation_distance)
 {
 this->_max_trans_dist = translation_distance;
 this->_max_trans_frame = frame;
 }
 Eigen::Vector3d prev_current_diff = current_translation - previous_translation;
 // If the cluster remains in the same position, consecutive frames will have the same translation, the difference will be the
 // vector zero, and the norm of it is nan!
 if (prev_current_diff != Eigen::Vector3d(0., 0., 0.))
 {
 this->_acc_trans_dist += prev_current_diff.norm();
 translation_axes.push_back(prev_current_diff);
 previous_translation = current_translation;
 accumulated_translation += current_translation;
 }
 frame++;
 }
 if (translation_axes.size() > 2)
 {
 double normalizing_factor = 0;
 accumulated_translation /= translation_axes.size();
 double accumulated_trans_norm = accumulated_translation.norm();
 Eigen::Vector3d acc_trans_norm = accumulated_translation / accumulated_trans_norm;
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "mean_dir (norm): " << acc_trans_norm);
 std::vector<Eigen::Vector3d>::iterator translation_axes_it = translation_axes.begin() + 1;
 std::vector<Eigen::Vector3d>::iterator translation_axes_it_end = translation_axes.end();
 for (; translation_axes_it != translation_axes_it_end; translation_axes_it++)
 {
 double min_distance = (accumulated_trans_norm < translation_axes_it->norm() ? accumulated_trans_norm
 : translation_axes_it->norm());

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "v_i " << *(translation_axes_it));

 Eigen::Vector3d v_i_norm = *translation_axes_it / (translation_axes_it)->norm();

 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "v_i (norm): " << v_i_norm);
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "v_i * mean_dir (norm): " << (acc_trans_norm.dot(v_i_norm)));
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "fabs(v_i * mean_dir (norm): " << fabs(acc_trans_norm.dot(v_i_norm)));

 double current_trans_axis_dot = fabs(acc_trans_norm.dot(v_i_norm)) * min_distance; // Sum of the angles of deviation between axis of rotation

 normalizing_factor += min_distance;
 this->_trans_axis_parall += current_trans_axis_dot;
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "min_distance: " << min_distance);
 ROS_DEBUG_STREAM_NAMED("BodyTrajectory.estimateTranslationParameters",
 "current_trans_axis_dot: " << current_trans_axis_dot);
 }
 // Normalize the axis deviation value
 this->_trans_axis_parall /= normalizing_factor;
 }
 else
 {
 ROS_WARN_STREAM("[BodyTrajectory::estimateTranslationParameters] Not enough frames for estimating the parallelism value for the axis of translation");
 this->_trans_axis_parall = 0.0;
 }
 this->_trans_params_estimated = true;
 */

