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
 * Joint.h
 *
 *  Created on: Apr 19, 2012
 *      Author: roberto
 */

#ifndef JOINT_H_
#define JOINT_H_

#include "feature.h"
#include "ClusterTrajectory.h"
#include "BodyTrajectory.h"
#include "BodyTrajectoryPair.h"

#include "iap_common/JointMsg.h"
#include "iap_common/AxisMsg.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Range.h"

#define MINIMUM_STD_DEV 0.001

typedef enum JointType
{
  PRISMATIC_JOINT = 0, REVOLUTE_JOINT = 1, RIGID_JOINT = 2, UNDEFINED_JOINT = 3, UNCERTAIN_JOINT = 4
} JointType;

struct Axis
{
  vision::FeaturePtr orientation;
  vision::FeaturePtr position;
  Axis(vision::FeaturePtr orientation_in=vision::FeaturePtr(new vision::Feature()),
       vision::FeaturePtr position_in=vision::FeaturePtr(new vision::Feature()))
  {
    this->orientation = orientation_in;
    this->position = position_in;
  }

  virtual iap_common::AxisMsg toROSMsg() const {
    iap_common::AxisMsg msg;

    msg.position.x = position->getX();
    msg.position.y = position->getY();
    msg.position.z = position->getZ();
    msg.orientation.x = orientation->getX();
    msg.orientation.y = orientation->getY();
    msg.orientation.z = orientation->getZ();

    return msg;
  }
};

class Joint;
typedef boost::shared_ptr<Joint> JointPtr;

class Joint
{
public:
  /**
   * Default constructor
   */
  Joint();

  /**
   * Constructor
   * @param axis - Axis of this joint
   */
  Joint(Axis axis);

  /**
   * Destructor
   */
  virtual ~Joint();

  /**
   * Get the type of Joint
   * @return - Type of joint
   */
  virtual JointType getType() const = 0;

  /**
   * Get the type of Joint as an std::string
   * @return - String with the name of the joint type
   */
  virtual std::string getTypeStr() const = 0;

  /**
   * Get the axis of motion of this joint
   * @return - Axis of motion of the joint
   */
  virtual Axis getAxis() const;

  /**
   * Estimate the transformation between the x axis of the world coordinate frame and the
   * main axis of motion of this joint
   * @return - Pair (both directions) of transformation between the axis of the joint and the x-axis of the world coordinate
   */
  virtual std::pair<tf::Transform, tf::Transform> getTransformationsToOrigin() const;

  /**
   * Get the axis of motion of this joint as an Rviz marker
   * @return - Axis of motion of the joint as Rviz marker
   */
  virtual visualization_msgs::Marker getAxisMarker() const;

  /**
   * Uses the last measurement to characterize the uncertainty in the orientation of the axis of motion and returns two
   * ros-rviz Range messages (cones) with the suitable radius that represent this uncertainty
   * @return - Pair (both directions) of Range markers for rviz
   */
  virtual std::pair<sensor_msgs::Range, sensor_msgs::Range> getAxisOrientationUncertaintyMarker() const;

  /**
   * Uses the last measurement to characterize the uncertainty in the position of the axis of motion and returns
   * an sphere centered in the believed position and with radius that represents this uncertainty
   * @return - Pair (both directions) of Range markers for rviz
   */
  virtual visualization_msgs::Marker getAxisPositionUncertaintyMarker() const;

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointOrientationUncertainty() const = 0;

  /**
   * Get the maximum deviation in radians from the orientation, based on the covariance of the measurement
   * @return - Standard deviation in radians
   */
  virtual double getJointPositionUncertainty() const = 0;

  /**
   * Clone the whole object and return a copy
   * @return - Clone of the Joint
   */
  JointPtr clone() const;

  /**
   * Estimate the axis of the motion of this joint
   *
   * The method should trigger the calculation of a goodness of fit, too
   * @see getGoodnessOfFit()
   */
  virtual void estimateAxis() = 0;

  /**
   * Get the goodness of fit
   */
  virtual double getGoodnessOfFit() = 0;

  /**
   * Transform this object into a ROS message
   */
  virtual iap_common::JointMsg toROSMsg();

  /**
   * Set the class from a ROS message
   *
   * (currently not implemented)
   */
  virtual void set(iap_common::JointMsg &);

  /**
   * @brief To string method for joint types
   */
  static std::string JointTypeToString(JointType type);

  /**
   * Set the minimum motion of the features, used to trigger the segmentation. It could be relevant to characterize the
   * uncertainty of the measurement
   * @param - min_motion - Minimum motion that triggers the segmentation
   */
  virtual void setMinimumMotion(double min_motion);

protected:
  Axis _axis;

  double _min_motion;

  virtual Joint* doClone() const = 0;
};

#endif /* JOINT_H_ */
