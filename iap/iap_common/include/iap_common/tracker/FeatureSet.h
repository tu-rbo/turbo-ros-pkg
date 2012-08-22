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
 * FeatureSet.h
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer & rmartin
 */

#ifndef FEATURESET_H_
#define FEATURESET_H_

#include "feature.h"
#include "RigidTransformation.h"
#include "sensor_msgs/PointCloud2.h"
#include <vector>
#include <boost/shared_ptr.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

namespace vision
{

class FeatureSet;
typedef boost::shared_ptr<FeatureSet> FeatureSetPtr;

/**
 * @brief A set a features at a certain frame in time
 *
 * This represents a set of Cartesian feature points during
 * a certain time frame.
 *
 * Usually we will construct the feature set from a measurement
 * from a FeatureTracker, i.e. you convert a PointCloud2 ROS message.
 */
class FeatureSet
{
public:

  /**
   * Constructor
   * @param frame - Number of frame the features of this FeatureSet belong to
   */
  FeatureSet(unsigned int frame);

  /**
   * Constructor
   * @param feature_set - ROS message that provides the values of the Features of this FeatureSet
   */
  FeatureSet(const sensor_msgs::PointCloud2 &feature_set);

  /**
   * Copy constructor
   * @param fs - FeatureSet to be copied
   */
  FeatureSet(const FeatureSet &fs);

  /**
   * Destructor
   */
  virtual ~FeatureSet();

  /**
   * Add another Feature to the FeatureSet
   * @param f - Feature to be added
   * @return - True if the add operation was successful
   */
  virtual bool addFeature(const FeaturePtr & f);

  /**
   * Clone the Feature contained in the FeatureSet with the passed Id
   * @param feature_id - Id of the Feature to be cloned
   * @return - Clone of the Feature
   */
  virtual FeaturePtr getFeature(const int feature_id) const;

  /**
   * Get the number of Features contained in this FeatureSet
   * @return - Number of Features in this FeatureSet
   */
  virtual unsigned int getNumberOfFeatures() const;

  /**
   * Getter of the frame the features of this FeatureSet belong to
   * @return - The frame number
   */
  virtual unsigned int getFrame() const;

  /**
   * Setter of the frame the features of this FeatureSet belong to
   * @param frame - The new frame number
   */
  virtual void setFrame(unsigned int frame);

  /**
   * Clone function
   * @return - Clone of this FeatureSet
   */
  FeatureSetPtr clone() const;

  /**
   * Delete all contained Features of this FeatureSet
   */
  virtual void clear();

  /**
   * Convert the Features in this FeatureSet to a PCL PointCloud
   * @param pcl_out - The PCL PointCloud structure that will contain all the Features
   */
  virtual void toPCL(pcl::PointCloud<pcl::PointXYZ>& pcl_out) const;

  /**
   * @brief Finds the center of the set of Features
   * @return FeaturePtr to the center of the set of Features
   */
  virtual FeaturePtr findCenter() const;

  /**
   * Check the number of contained Features that are not lost
   * @return - Number of not lost features
   */
  virtual int getNumberNonLostFeatures() const;

  /**
   * Check if this FeatureSet contains a Feature with the given Id
   * @param feature_id - Id of the searched Feature
   * @return - True if the Feature is in this FeatureSet
   */
  virtual bool isContained(const int feature_id) const;

  /**
   * Change the Id of one of the contained Features
   * @param old_feature_id - Id of the Feature to be changed
   * @param new_feature_id - New Id of the Feature
   */
  void changeFeatureId(int old_feature_id, int new_feature_id);

  /**
   * Creates a vector < FeaturePtr > from this FeatureSet
   * @return - vector of Features with a COPY of the contents of this FeatureSet
   */
  virtual std::vector<FeaturePtr> FeatureSet2Vector() const;

  /**
   * Change the positions of the Features in this FeatureSet with the information contained in the ROS message
   * @param new_positions - ROS message containing the new positions of the features of the FeatureSet
   * @return - True if the update was successful. If the message and the FeatureSet have different number of elements, return false.
   */
  virtual bool updateFromROS(const sensor_msgs::PointCloud2 &new_positions);

  // Geometric Functions /////////////////////////////////////////////////////////////////////////
  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  typedef std::pair<FeaturePtr, Eigen::Quaterniond > MotionDefinition;

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  //virtual FeatureSetPtr move(FeaturePtr translation, std::vector<double> quaternion_rot) const;
  virtual FeatureSetPtr move(FeaturePtr translation, Eigen::Quaterniond quaternion_rot) const;

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  //virtual FeatureSetPtr move(FeaturePtr translation, std::vector<double> quaternion_rot) const;
  virtual FeatureSetPtr move(Eigen::Vector3d translation, Eigen::Quaterniond quaternion_rot) const;

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  virtual FeatureSetPtr move(FeatureSet::MotionDefinition motion) const;

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  virtual FeatureSetPtr moveInverse(FeatureSet::MotionDefinition motion) const;

  virtual FeatureSetPtr scale(double scale_value) const;

  virtual MotionDefinition estimateMotion(FeatureSetPtr second_set) const;

  RigidTransformationPtr estimateRigidTransformation(FeatureSetPtr second_set);

  /**
   * Estimate the transformation that explains the motion between this FeatureSet and the FeatureSet passed as variable.
   * The second parameter is an initial guess of the transformation between FeatureSets
   * @param second_set - Second position of the FeatureSet. Destination to reach using the estimated transformation
   * @param initial_guess - Initial guess of the transformation between FeatureSets
   * @return - RigidTransformation object containing the transformation estimated
   */
  RigidTransformationPtr estimateRigidTransformation(FeatureSetPtr second_set, RigidTransformationPtr initial_guess);

  virtual FeatureSetPtr applyRigidTransformation(RigidTransformationPtr rt) const;

  virtual FeatureSetPtr applyInverseRigidTransformation(RigidTransformationPtr rt) const;

  virtual void PublishInROS(ros::Publisher* publisher) const;

protected:
  unsigned int _frame;
  std::vector<FeaturePtr>       _features_vector;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> *_icp;

  virtual FeatureSet* doClone() const
  {
    return (new FeatureSet(*this));
  }

public:
  // Iterators ///////////////////////////////////////////////////////////////////
  typedef std::vector<FeaturePtr>::iterator iterator;
  typedef std::vector<FeaturePtr>::const_iterator const_iterator;

  iterator begin()
  {
    return _features_vector.begin();
  }

  iterator end()
  {
    return _features_vector.end();
  }

  const_iterator begin() const
  {
    return _features_vector.begin();
  }

  const_iterator end() const
  {
    return _features_vector.end();
  }

  // Printing Functions ///////////////////////////////////////////////////////////
  virtual void printFS() const;
  virtual void printF(const int feature_id) const;

  // Writing and Reading Functions
  virtual void writeToFileFS(std::string file_name) const;
  virtual void appendToFileFS(std::string file_name) const;
  virtual void readFromFileFS(std::string file_name);
  virtual void readFromFileFS(std::ifstream &file_to_read);
};

void FeatureSet2PCL(FeatureSetPtr fs_in, pcl::PointCloud<pcl::PointXYZ>& pcl_out);
void PCL2FeatureSet(const pcl::PointCloud<pcl::PointXYZ>& pcl_in, FeatureSetPtr fs_out);

}

#endif /* FEATURESET_H_ */
