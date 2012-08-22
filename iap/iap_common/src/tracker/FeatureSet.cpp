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
 * FeatureSet.cpp
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#include "FeatureSet.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/ros/conversions.h>

#ifdef ROS_ELECTRIC
    using pcl::registration::CorrespondencesPtr;
    using pcl::registration::Correspondence;
    using pcl::registration::Correspondences;
#elif defined(ROS_FUERTE)
    #include <pcl/correspondence.h>
    using pcl::CorrespondencesPtr;
    using pcl::Correspondence;
    using pcl::Correspondences;
#else
#error The ROS version is not defined
#endif



#include <vector>

#include <iostream>

#include <boost/algorithm/string.hpp>

namespace vision
{

FeatureSet::FeatureSet(unsigned int frame) :
  _frame(frame), _icp(NULL)
{
}

FeatureSet::~FeatureSet()
{
}

FeatureSet::FeatureSet(const sensor_msgs::PointCloud2 &feature_set)
{
  // TODO
  // instantiate _features array
  // convert feature_set to PCL, create features, add to array
  // update _length
}

FeatureSet::FeatureSet(const FeatureSet &fs)
{
  this->_frame = fs.getFrame();
  FeatureSet::const_iterator fs_it = fs.begin();
  FeatureSet::const_iterator fs_it_end = fs.end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    this->addFeature((*fs_it)->clone());
  }
  this->_icp = fs._icp;
}

bool FeatureSet::addFeature(const FeaturePtr& f)
{
  _features_vector.push_back(f);
  //_features_map[f->getId()] = f;
  return true;
}

FeaturePtr FeatureSet::getFeature(const int feature_id) const
{
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    if ((*fs_it)->getId() == feature_id)
    {
      return (*fs_it)->clone();
    }
  }
  ROS_ERROR("[FeatureSet::getFeature] This FeatureSet does NOT contain a Feature with Id %d!", feature_id);
  return FeaturePtr(new Feature(-1, -1, -1));
}

unsigned int FeatureSet::getNumberOfFeatures() const
{
  //return _features_map.size();
  return _features_vector.size();
}

unsigned int FeatureSet::getFrame() const
{
  return _frame;
}

void FeatureSet::setFrame(unsigned int frame)
{
  this->_frame = frame;
}

FeatureSetPtr FeatureSet::clone() const
{
  return (FeatureSetPtr(doClone()));
}

void FeatureSet::clear()
{
  _frame = -1;
  _features_vector.clear();
  //_features_map.clear();
}

void FeatureSet::printFS() const
{
  std::cout << "Feature Set in frame " << _frame << " :" << std::endl;
  int index = 0;
  for (FeatureSet::const_iterator it = _features_vector.begin(); it != _features_vector.end(); it++)
  {
    std::cout << "Feature " << (*it)->getId() << " : " << *it << std::endl;
    index++;
  }
}

void FeatureSet::printF(const int feature_id) const
{
  FeaturePtr feature = this->getFeature(feature_id);
  if (feature)
  {
    std::cout << "Feature " << feature_id << " in frame " << _frame << " : " << feature << std::endl;
  }
  else
  {
    std::cout << "ERROR[FeatureSet::print(const int feature_id)]: This feature is not in the Feature Set." << std::endl;
  }
}

void FeatureSet::writeToFileFS(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios_base::trunc);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[FeatureSet::writeToFile(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }
  for (FeatureSet::const_iterator it = _features_vector.begin(); it != _features_vector.end(); it++)
  {
    //file_to_write << *it << std::endl;
    file_to_write << (*it)->getX() << " " << (*it)->getY() << " " << (*it)->getZ() << " ";
  }
  //file_to_write << "--" << std::endl;
  file_to_write << std::endl;

  file_to_write.close();
}

void FeatureSet::appendToFileFS(std::string file_name) const
{
  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios::out | std::ios::app);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[FeatureSet::appendToFile(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }
  // We add a first end-of-line to the file to separate the previous content
  //  file_to_write << std::endl;
  for (FeatureSet::const_iterator it = _features_vector.begin(); it != _features_vector.end(); it++)
  {
    //TODO: CHANGE it to be as before
    //file_to_write << *it << std::endl;
    file_to_write << (*it)->getX() << " " << (*it)->getY() << " " << (*it)->getZ() << " ";
  }
  //file_to_write << "--" << std::endl;
  file_to_write << std::endl;

  file_to_write.close();
}

void FeatureSet::readFromFileFS(std::string file_name)
{
  std::ifstream file_to_read;
  file_to_read.open(file_name.c_str(), std::ios::in);

  readFromFileFS(file_to_read);
}

void FeatureSet::readFromFileFS(std::ifstream &file_to_read)
{
  _features_vector.clear();

  if (!file_to_read.is_open())
  {
    std::cout << "ERROR[FeatureSet::readFromFileFS(std::fstream file_to_read)]: The file couldn't be opened."
        << std::endl;
    return;
  }
  std::string line;
  while (getline(file_to_read, line))//file_to_read.eof())
  {
    // read until eof or first empty line
    boost::algorithm::trim(line);
    if (line == "--" || line == "==")
    {
      break;
    }

    FeaturePtr f(new Feature);
    line >> f;
    addFeature(f);
    //    std::cout << (*f) << std::endl;
  }
}

//FeatureSetPtr FeatureSet::move(FeaturePtr translation, std::vector<double> quaternion_rot) const
FeatureSetPtr FeatureSet::move(FeaturePtr translation, Eigen::Quaterniond quaternion_rot) const
{
  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  Eigen::Vector3d offset(translation->getX(), translation->getY(), translation->getZ());

  pcl::PointCloud < pcl::PointXYZ > cloud_in;
  pcl::PointCloud < pcl::PointXYZ > cloud_out;
  FeatureSetPtr moved_fs(new FeatureSet(0));

  this->toPCL(cloud_in);
  cloud_out = cloud_in;
  pcl::transformPointCloud(cloud_in, cloud_out, offset.cast<float> (), quaternion_rot.cast<float> ());

  int pcl_index = 0;
  for (FeatureSet::const_iterator it = this->begin(); it != this->end(); it++)
  {
    moved_fs->addFeature(
                         (*it)->cloneAndUpdate(cloud_out.points[pcl_index].x, cloud_out.points[pcl_index].y,
                                               cloud_out.points[pcl_index].z));
    pcl_index++;
  }
  return moved_fs;
}

FeatureSetPtr FeatureSet::move(Eigen::Vector3d translation, Eigen::Quaterniond quaternion_rot) const
{
  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z

  pcl::PointCloud < pcl::PointXYZ > cloud_in;
  pcl::PointCloud < pcl::PointXYZ > cloud_out;
  FeatureSetPtr moved_fs(new FeatureSet(0));

  this->toPCL(cloud_in);
  cloud_out = cloud_in;
  pcl::transformPointCloud(cloud_in, cloud_out, translation.cast<float> (), quaternion_rot.cast<float> ());

  int pcl_index = 0;
  for (FeatureSet::const_iterator it = this->begin(); it != this->end(); it++)
  {
    moved_fs->addFeature(
                         (*it)->cloneAndUpdate(cloud_out.points[pcl_index].x, cloud_out.points[pcl_index].y,
                                               cloud_out.points[pcl_index].z));
    pcl_index++;
  }
  return moved_fs;
}

FeatureSetPtr FeatureSet::move(FeatureSet::MotionDefinition motion) const
{
  std::cout << motion.first->getX() << " " << motion.first->getY() << " " << motion.first->getZ() << " " << std::endl;
  return this->move(motion.first, motion.second);
}

FeatureSetPtr FeatureSet::moveInverse(FeatureSet::MotionDefinition motion) const
{
  /*Inverse:
   The transform is
   v'=T+R*v
   using some simple algebra on it to solve for v:
   v'-T=R*v
   inverse(R)*(v' - T) = inverse(R)*R*v = v
   v = -inverse(R)*T +inverse(R)*(v')
   */
  Eigen::Vector3d offset(motion.first->getX(), motion.first->getY(), motion.first->getZ());
  Eigen::Quaterniond inverse = motion.second.conjugate(); // Inverse and conjugate are the same when working with normalized quats
  Eigen::Vector3d offset_inv = inverse * offset;
  FeaturePtr inverse_translation = FeaturePtr(new Feature(-offset_inv[0], -offset_inv[1], -offset_inv[2]));
  return this->move(inverse_translation, inverse);
}

FeatureSetPtr FeatureSet::scale(double scale_value) const
{
  FeatureSetPtr ret_val = FeatureSetPtr(new FeatureSet(this->getFrame()));
  for (FeatureSet::const_iterator it = this->begin(); it != this->end(); it++)
  {
    FeaturePtr temp_feat = (*it)->cloneAndUpdate((*it)->getX() / scale_value, (*it)->getY() / scale_value,
                                                 (*it)->getZ() / scale_value);

    ret_val->addFeature(temp_feat);
  }
  return ret_val;
}

FeatureSet::MotionDefinition FeatureSet::estimateMotion(FeatureSetPtr second_set) const
{
  pcl::PointCloud < pcl::PointXYZ > first_cloud;
  pcl::PointCloud < pcl::PointXYZ > last_cloud;

  this->toPCL(first_cloud);
  second_set->toPCL(last_cloud);

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z

  // Reject outliers!!
  pcl::registration::CorrespondenceRejectorSampleConsensus < pcl::PointXYZ > rejector;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr in_ptr(*first_cloud);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in_const(&first_cloud);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr out_ptr(*last_cloud);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_out_const(&last_cloud);
  rejector.setInputCloud(cloud_in_const);
  rejector.setTargetCloud(cloud_out_const);
  rejector.setInlierThreshold(0.001); // 5 cm
  rejector.setMaxIterations(1000);
  //  pcl::registration::CorrespondencesPtr correspondences_in(new pcl::registration::Correspondences);
  //  pcl::registration::CorrespondencesPtr correspondences_out(new pcl::registration::Correspondences);
  //  int index = 0;
  //  for(int index = 0; index<this->getNumberOfFeatures(); index++)
  //  {
  //    double distance = sqrt((first_cloud.points[index].x - last_cloud.points[index].x)*(first_cloud.points[index].x - last_cloud.points[index].x)+
  //        (first_cloud.points[index].y - last_cloud.points[index].y)*(first_cloud.points[index].y - last_cloud.points[index].y)+
  //        (first_cloud.points[index].z - last_cloud.points[index].z)*(first_cloud.points[index].z - last_cloud.points[index].z));
  //    pcl::registration::Correspondence* corr_temp = new pcl::registration::Correspondence(index, index, distance);
  //    correspondences_in->push_back(*corr_temp);
  //  }
  //  rejector.getCorrespondences(*correspondences_in, *correspondences_out);


  //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator;
  Eigen::Matrix4f T; // the result transformation
  T = rejector.getBestTransformation();
  //estimator.estimateRigidTransformation(first_cloud, last_cloud, *correspondences_out,  T);
  ROS_DEBUG(
            "\nTransformation: %.3f %.3f %.3f %.3f  \n \
                %.3f %.3f %.3f %.3f  \n \
                %.3f %.3f %.3f %.3f  \n \
                %.3f %.3f %.3f %.3f",
            T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2), T(2, 3),
            T(3, 0), T(3, 1), T(3, 2), T(3, 3));

  Eigen::Matrix3d rot_matrix = T.topLeftCorner(3, 3).cast<double> ();
  Eigen::Quaterniond rotation(rot_matrix);
  Eigen::Vector3d offset = T.topRightCorner(3, 1).cast<double> ();
  FeaturePtr ret_offset = FeaturePtr(new Feature(offset(0), offset(1), offset(2)));

  return FeatureSet::MotionDefinition(ret_offset, rotation);
}

RigidTransformationPtr FeatureSet::estimateRigidTransformation(FeatureSetPtr second_set)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

  this->toPCL(*cloud_in);
  second_set->toPCL(*cloud_out);

  //NOTE: Be careful!!!!!
  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z

  pcl::registration::CorrespondenceRejectorSampleConsensus < pcl::PointXYZ > rejector;
  rejector.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr(cloud_in));
  rejector.setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr(cloud_out));
  rejector.setInlierThreshold(0.005); // 0.5 cm
  rejector.setMaxIterations(1000);
  Correspondences correspondences_in;

  Correspondence* corr_temp = NULL;
  for (int index_corresp = 0; index_corresp < cloud_in->points.size(); index_corresp++)
  {
    double distance_corresp = sqrt(
                                   (cloud_in->points[index_corresp].x - cloud_out->points[index_corresp].x)
                                       * (cloud_in->points[index_corresp].x - cloud_out->points[index_corresp].x)
                                       + (cloud_in->points[index_corresp].y - cloud_out->points[index_corresp].y)
                                           * (cloud_in->points[index_corresp].y - cloud_out->points[index_corresp].y)
                                       + (cloud_in->points[index_corresp].z - cloud_out->points[index_corresp].z)
                                           * (cloud_in->points[index_corresp].z - cloud_out->points[index_corresp].z));
    corr_temp = new Correspondence(index_corresp, index_corresp, distance_corresp);
    correspondences_in.push_back(*corr_temp);
  }
  Correspondences correspondences_out;


#ifdef ROS_ELECTRIC
  rejector.setInputCorrespondences(CorrespondencesPtr(&correspondences_in));
  rejector.getCorrespondeces(correspondences_out);
#elif defined(ROS_FUERTE)
  rejector.getRemainingCorrespondences(correspondences_in, correspondences_out);
#endif

  Eigen::Matrix4f initial_T = rejector.getBestTransformation();

  pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > estimator;
  estimator.estimateRigidTransformation(*cloud_in, *cloud_out, correspondences_out, initial_T);

  if (!this->_icp)
  {
    this->_icp = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
  }
  _icp->setInputCloud(cloud_in);
  _icp->setInputTarget(cloud_out);
  _icp->setTransformationEpsilon(1e-6);
  pcl::PointCloud < pcl::PointXYZ > Final;
  _icp->align(Final, initial_T);
  Eigen::Matrix4f T = _icp->getFinalTransformation();
  ROS_DEBUG_NAMED(
                  "FeatureSet.estimateRigidTransformation",
                  "\n \ Transformation: %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f",
                  T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2),
                  T(2, 3), T(3, 0), T(3, 1), T(3, 2), T(3, 3));
  return RigidTransformationPtr(new RigidTransformation(T));
  ROS_DEBUG_NAMED(
                  "FeatureSet.estimateRigidTransformation",
                  "\n \ Transformation: %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f  \n \
                  %.3f %.3f %.3f %.3f",
                  T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2),
                  T(2, 3), T(3, 0), T(3, 1), T(3, 2), T(3, 3));
  double signo = (T(2, 1) - T(1, 2)) * T(0, 3) + (T(0, 2) - T(2, 0)) * T(1, 3) + (T(1, 0) - T(0, 1)) * T(2, 3);
  int signo_int = 0;
  if (signo < 0)
  {
    signo_int = -1;
  }
  else if (signo > 0)
  {
    signo_int = 1;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("FeatureSet.estimateRigidTransformation", "Signo of the angle estimation is zero");
  }
  double thetas = (double)signo_int * (fabs(acos((T(0, 0) + T(1, 1) + T(2, 2) - 1) / 2)));
  double hache = signo / (2 * thetas * sin(thetas));
  Eigen::Matrix3d rotaciones;
  rotaciones << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2);
  Eigen::Vector3d roro = (Eigen::Matrix3d::Identity() - rotaciones) * Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3)) / (2
      * (1 - cos(thetas)));
  Eigen::Vector3d omegas((T(2, 1) - T(1, 2)) / (2 * sin(thetas)), (T(0, 2) - T(2, 0)) / (2 * sin(thetas)),
                         (T(1, 0) - T(0, 1)) / (2 * sin(thetas)));
  return RigidTransformationPtr(new RigidTransformation(T));
}

RigidTransformationPtr FeatureSet::estimateRigidTransformation(FeatureSetPtr second_set,
                                                               RigidTransformationPtr initial_guess)
{
  // Convert both FeatureSets in PCL point clouds to estimate the transformation using it
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  this->toPCL(*cloud_in);
  second_set->toPCL(*cloud_out);

  if (cloud_in->points.size() != cloud_out->points.size())
  {
    ROS_ERROR_STREAM_NAMED("FeatureSet.estimateRigidTransformation",
                           "Source and destination to estimate the transformationa have different"
                             " number of Features.");
  }

  // Use PCL to reject wrong tracked Features between both FeatureSets
  pcl::registration::CorrespondenceRejectorSampleConsensus < pcl::PointXYZ > rejector;
  rejector.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr(cloud_in));
  rejector.setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr(cloud_out));
  rejector.setInlierThreshold(0.005); // 0.5 cm
  rejector.setMaxIterations(1000);

#ifdef ROS_ELECTRIC
// Initialize the correspondences and the distance between points
  pcl::registration::CorrespondencesPtr correspondences_in(new pcl::registration::Correspondences);
  pcl::registration::Correspondence* corr_temp = NULL;
  for (int index_corresp = 0; index_corresp < cloud_in->points.size(); index_corresp++)
  {
    double distance_corresp =
        sqrt(
             pow(cloud_in->points[index_corresp].x - cloud_out->points[index_corresp].x, 2)
                 + pow(cloud_in->points[index_corresp].y - cloud_out->points[index_corresp].y, 2)
                 + pow(cloud_in->points[index_corresp].z - cloud_out->points[index_corresp].z, 2));
    corr_temp = new pcl::registration::Correspondence(index_corresp, index_corresp, distance_corresp);
    correspondences_in->push_back(*corr_temp);
  }
  pcl::registration::Correspondences correspondences_out;
  rejector.setInputCorrespondences((correspondences_in));
  rejector.getCorrespondeces(correspondences_out);
#elif defined(ROS_FUERTE)
  // Initialize the correspondences and the distance between points
  Correspondences correspondences_in;
  Correspondence* corr_temp = NULL;
  for (int index_corresp = 0; index_corresp < cloud_in->points.size(); index_corresp++)
  {
    double distance_corresp =
        sqrt(
             pow(cloud_in->points[index_corresp].x - cloud_out->points[index_corresp].x, 2)
                 + pow(cloud_in->points[index_corresp].y - cloud_out->points[index_corresp].y, 2)
                 + pow(cloud_in->points[index_corresp].z - cloud_out->points[index_corresp].z, 2));
    corr_temp = new Correspondence(index_corresp, index_corresp, distance_corresp);
    correspondences_in.push_back(*corr_temp);
  }
  Correspondences correspondences_out;
  rejector.getRemainingCorrespondences(correspondences_in, correspondences_out);
#endif

  // Use ICP to estimate the transformation, without the outliers
  Eigen::Matrix4f initial_guess_eigen = initial_guess->getTransformationMatrix().matrix().cast<float> ();
  if (!this->_icp)
  {
    this->_icp = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
  }
  _icp->setInputCloud(cloud_in);
  _icp->setInputTarget(cloud_out);
  _icp->setTransformationEpsilon(1e-6);
  pcl::PointCloud < pcl::PointXYZ > cloud_aligned = pcl::PointCloud<pcl::PointXYZ>();
  _icp->align(cloud_aligned, initial_guess_eigen);
  Eigen::Matrix4f T = _icp->getFinalTransformation();

  return RigidTransformationPtr(new RigidTransformation(T));
}

FeatureSetPtr FeatureSet::applyRigidTransformation(RigidTransformationPtr rt) const
{
  return this->move(rt->getTranslation(), rt->getRotationQuaternion());
}

FeatureSetPtr FeatureSet::applyInverseRigidTransformation(RigidTransformationPtr rt) const
{
  Eigen::Vector3d offset = rt->getTranslation();
  Eigen::Quaterniond inverse = rt->getRotationQuaternion().conjugate(); // Inverse and conjugate are the same when working with normalized quats
  Eigen::Vector3d offset_inv = inverse * offset;
  Eigen::Vector3d inverse_translation = -rt->getTranslation();
  return this->move(inverse_translation, inverse);
}

void FeatureSet::changeFeatureId(int old_feature_id, int new_feature_id)
{
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    if ((*fs_it)->getId() == old_feature_id)
    {
      (*fs_it)->setId(new_feature_id);
      return;
    }
  }
  ROS_ERROR("[FeatureSet::changeFeatureId]: The Feature with Id %d does NOT belong to this FeatureSet", old_feature_id);
  return;
}

std::vector<FeaturePtr> FeatureSet::FeatureSet2Vector() const
{
  std::vector < FeaturePtr > ret_val;
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    ret_val.push_back((*fs_it)->clone());
  }
  return ret_val;
}

bool FeatureSet::updateFromROS(const sensor_msgs::PointCloud2 &new_positions)
{
  pcl::PointCloud < pcl::PointXYZ > new_positions_pcl;
  pcl::fromROSMsg(new_positions, new_positions_pcl);

  if (new_positions_pcl.size() != this->getNumberOfFeatures())
  {
    ROS_ERROR(
              "[FeatureSet::updateFromROS]: The ROS message (%d) and the FeatureSet (%d) have different number of elements.",
              new_positions_pcl.size(), this->getNumberOfFeatures());
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::iterator np_it = new_positions_pcl.begin();
  pcl::PointCloud<pcl::PointXYZ>::iterator np_it_end = new_positions_pcl.end();
  FeatureSet::iterator fs_it = this->begin();
  FeatureSet::iterator fs_it_end = this->end();
  for (; np_it != np_it_end && fs_it != fs_it_end; np_it++, fs_it++)
  {
    (*fs_it)->setPos(np_it->x, np_it->y, np_it->z);
    if ((np_it->x == -1. && np_it->y == -1. && np_it->z == -1.) || (np_it->x == -2. && np_it->y == -2. && np_it->z
        == -2.))
    {
      (*fs_it)->setLost(true);
    }
  }
  return true;
}

void FeatureSet::toPCL(pcl::PointCloud<pcl::PointXYZ>& pcl_out) const
{
  //TODO: Why is clear() not declared? The web documentation says it should be
  //pcl_out.clear();
  pcl_out.width = this->getNumberOfFeatures();
  pcl_out.height = 1; //Unorganized point cloud
  pcl_out.is_dense = false;
  pcl_out.points.resize(pcl_out.width * pcl_out.height);

  int pcl_index = 0;
  for (FeatureSet::const_iterator it = this->begin(); it != this->end(); it++)
  {
    pcl_out.points[pcl_index].x = (*it)->getX();
    pcl_out.points[pcl_index].y = (*it)->getY();
    pcl_out.points[pcl_index].z = (*it)->getZ();
    pcl_index++;
  }
}

FeaturePtr FeatureSet::findCenter() const
{
  double x_center = 0;
  double y_center = 0;
  double z_center = 0;
  for (std::vector<FeaturePtr>::const_iterator it = _features_vector.begin(); it != _features_vector.end(); it++)
  {
    x_center += (*it)->getX();
    y_center += (*it)->getY();
    z_center += (*it)->getZ();
  }
  return FeaturePtr(
                    new Feature(x_center / (double)_features_vector.size(), y_center / (double)_features_vector.size(),
                                z_center / (double)_features_vector.size()));
}

int FeatureSet::getNumberNonLostFeatures() const
{
  int ret_val = 0;
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    if (!(*fs_it)->isLost())
    {
      ret_val++;
    }
  }
  return ret_val;
}

bool FeatureSet::isContained(const int feature_id) const
{
  if (this->getNumberOfFeatures() == 0)
  {
    return false;
  }
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    if ((*fs_it)->getId() == feature_id)
    {
      return true;
    }
  }
  return false;
}

void FeatureSet2PCL(FeatureSetPtr fs_in, pcl::PointCloud<pcl::PointXYZ>& pcl_out)
{
  //TODO: Why is clear() not declared? The web documentation says it should be
  //pcl_out.clear();
  pcl_out.width = fs_in->getNumberOfFeatures();
  pcl_out.height = 1; //Unorganized point cloud
  pcl_out.is_dense = false;
  pcl_out.points.resize(pcl_out.width * pcl_out.height);

  int pcl_index = 0;
  for (FeatureSet::const_iterator it = fs_in->begin(); it != fs_in->end(); it++)
  {
    pcl_out.points[pcl_index].x = (*it)->getX();
    pcl_out.points[pcl_index].y = (*it)->getY();
    pcl_out.points[pcl_index].z = (*it)->getZ();
    pcl_index++;
  }
}
void PCL2FeatureSet(const pcl::PointCloud<pcl::PointXYZ>& pcl_in, FeatureSetPtr fs_out)
{
  fs_out->clear();
  for (int i = 0; i < pcl_in.width; i++)
  {
    FeaturePtr temp(new Feature(pcl_in.points[i].x, pcl_in.points[i].y, pcl_in.points[i].z));
    fs_out->addFeature(temp);
  }
}

void FeatureSet::PublishInROS(ros::Publisher* publisher) const
{
  FeatureSet::const_iterator fs_it = this->begin();
  FeatureSet::const_iterator fs_it_end = this->end();
  pcl::PointCloud < pcl::PointXYZ > temp_pcl_one_frame;
  for (; fs_it != fs_it_end; fs_it++)
  {
    if (!(*fs_it)->isLost())
    {
      pcl::PointXYZ temp_point_3d;
      temp_point_3d.x = (*fs_it)->getX();
      temp_point_3d.y = (*fs_it)->getY();
      temp_point_3d.z = (*fs_it)->getZ();
      //ROS_INFO_STREAM("[VisualNet::SfM] " << (*fs_it)->getX() << " " << (*fs_it)->getY()<<" " << (*fs_it)->getZ());
      temp_pcl_one_frame.points.push_back(pcl::PointXYZ(temp_point_3d));
    }
    else
    {
      pcl::PointXYZ temp_point_3d(-1.f, -1.f, -1.f);
      temp_pcl_one_frame.points.push_back(pcl::PointXYZ(temp_point_3d));
    }
  }
  sensor_msgs::PointCloud2 temp_ros_one_frame;
  pcl::toROSMsg(temp_pcl_one_frame, temp_ros_one_frame);
  temp_ros_one_frame.header.stamp = ros::Time::now();
  temp_ros_one_frame.header.frame_id = "/world";
  publisher->publish(temp_ros_one_frame);
}

}

/*
 // DEBUG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 // Use the estimated transformation to find a point on the axis of rotation  (Screw theory -> Handbook of robotics p. 16)
 Eigen::Vector3d translation_eigen(T(0, 3), T(1, 3), T(2, 3));
 Eigen::Vector3d axis_rot_eigen(T(2, 1) - T(1, 2), T(0, 2) - T(2, 0), T(1, 0) - T(0, 1));
 double axis_rot_dot_translation = axis_rot_eigen.dot(translation_eigen);
 int axis_rot_dot_translation_signus = 0;
 if (axis_rot_dot_translation < 0)
 {
 axis_rot_dot_translation_signus = -1;
 }
 else if (axis_rot_dot_translation > 0)
 {
 axis_rot_dot_translation_signus = 1;
 }
 else
 {
 ROS_ERROR_STREAM_NAMED("FeatureSet.estimateRigidTransformation",
 "Dot product between line vector and translation vector is 0");
 }
 double rot_angle = (double)axis_rot_dot_translation_signus * (fabs(acos((T(0, 0) + T(1, 1) + T(2, 2) - 1) / 2)));
 double pitch = axis_rot_dot_translation / (2 * rot_angle * sin(rot_angle));
 Eigen::Matrix3d rotation_matrix_eigen;
 rotation_matrix_eigen << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2);
 Eigen::Vector3d point_on_rot_axis = (Eigen::Matrix3d::Identity() - rotation_matrix_eigen.transpose()) * translation_eigen / (2
 * (1 - cos(rot_angle)));
 Eigen::Vector3d angular_velocity = axis_rot_eigen / (2 * sin(rot_angle));
 // TEST: Transform the first point of this FeatureSet using the estimated transformation
 std::ofstream number_of_features;
 number_of_features.open("number_of_features.txt", std::ios_base::trunc);
 number_of_features << this->getNumberOfFeatures();
 number_of_features.close();

 int counting_features = 0;
 for (int i = 0; i < this->getNumberOfFeatures(); i++)
 {
 std::ofstream transformed_pc_file;
 std::stringstream transformed_pc_ss;
 transformed_pc_ss << std::string("transformed_pc_") << i << std::string(".txt");
 transformed_pc_file.open(transformed_pc_ss.str().c_str(), std::ios_base::app);

 FeaturePtr feature_to_transform = (this->_features_vector[i])->clone();
 FeaturePtr feature_destination = (*(second_set->begin() + i))->clone();
 Eigen::Vector4f feature_to_tranform_eigen(feature_to_transform->getX(), feature_to_transform->getY(),
 feature_to_transform->getZ(), 1);
 Eigen::Vector4f feature_transformed_eigen = T * feature_to_tranform_eigen;

 transformed_pc_file << feature_to_tranform_eigen(0) << " " << feature_to_tranform_eigen(1) << " "
 << feature_to_tranform_eigen(2) << std::endl;
 transformed_pc_file << feature_transformed_eigen(0) << " " << feature_transformed_eigen(1) << " "
 << feature_transformed_eigen(2) << std::endl;
 transformed_pc_file << feature_destination->getX() << " " << feature_destination->getY() << " "
 << feature_destination->getZ() << std::endl;
 transformed_pc_file.close();
 }
 // END OF DEBUG ///////////////////////////////////////////////////////////////////////////////////////////////////////
 */
