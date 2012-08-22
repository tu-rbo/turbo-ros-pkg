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
 * FeatureTrajectorySet.cpp
 *
 *  Created on: Nov 28, 2011
 *      Author: shoefer
 */

#include "FeatureTrajectorySet.h"

#include "MathFunctions.h"

namespace vision
{

FeatureTrajectorySet::FeatureTrajectorySet()
{
}

FeatureTrajectorySet::FeatureTrajectorySet(const FeatureTrajectorySet & fts)
{
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it = fts.begin();
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it_end = fts.end();
  for (; fts_it != fts_it_end; fts_it++)
  {
    this->addFeatureSet((*fts_it));
  }
}

FeatureTrajectorySet::~FeatureTrajectorySet()
{
}

bool FeatureTrajectorySet::addFeatureSet(const FeatureSetPtr &fs)
{
  if (this->_feature_sets.size() != 0)
  {
    if (this->getNumberOfFeatures() != fs->getNumberOfFeatures())
    {
      // cannot add feature sets
      ROS_DEBUG("[FeatureTrajectorySet::add] Passed feature set has not the correct amount features");
      return false;
    }
  }
  // TODO make sure feature set has a frame number which is not
  // yet contained in _feature_sets
  _feature_sets.push_back(fs);
  return true;
}

FeatureSetPtr FeatureTrajectorySet::getFeatureSet(const unsigned int frame_nr) const
{
  if (frame_nr >= _feature_sets.size())
  {
    return FeatureSetPtr();
  }
  return _feature_sets[frame_nr]->clone();
}

FeatureTrajectoryPtr FeatureTrajectorySet::getFeatureTrajectory(const int feature_id) const
{
  FeatureTrajectoryPtr out;

  if (_feature_sets.size() == 0)
  {
    return out;
  }

  out.reset(new FeatureTrajectory);

  int t = 0;
  for (std::vector<FeatureSetPtr>::const_iterator it = _feature_sets.begin(); it != _feature_sets.end(); it++)
  {
    FeaturePtr f = (*it)->getFeature(feature_id);
    if (!f)
    {
      ROS_ERROR("No feature with id=%d in set at t=%d", feature_id, t);
      return FeatureTrajectoryPtr();
    }
    out->addFeature(f);
    t++;
  }
  return out;
}

FeaturePtr FeatureTrajectorySet::getFeature(const int unsigned frame_nr, const int feature_id) const
{
  if (frame_nr >= _feature_sets.size() || _feature_sets.size() == 0)
  {
    return FeaturePtr();
  }
  return _feature_sets[frame_nr]->getFeature(feature_id)->clone();
}

int FeatureTrajectorySet::getNumberOfFeatures() const
{
  return _feature_sets.size() == 0 ? 0 : _feature_sets.front()->getNumberOfFeatures();
}

unsigned int FeatureTrajectorySet::getTrajectoryLength() const
{
  return _feature_sets.size();
}

bool FeatureTrajectorySet::isConsistent() const
{
  int sz = -1;
  // check trajectory length per timestep
  for (std::vector<FeatureSetPtr>::const_iterator it = _feature_sets.begin(); it != _feature_sets.end(); it++)
  {
    int cur_sz = (*it)->getNumberOfFeatures();
    if (sz == -1)
    {
      sz = cur_sz;
      continue;
    }
    if (sz != cur_sz)
    {
      return false;
    }
  }

  return true;
}

void FeatureTrajectorySet::printFTS() const
{
  std::vector<FeatureSetPtr>::const_iterator fts_it = _feature_sets.begin();
  std::cout << "Feature Trajectory Set: " << std::endl;
  for (; fts_it != _feature_sets.end(); fts_it++)
  {
    (*fts_it)->printFS();
  }
}

void FeatureTrajectorySet::printFT(int feature_id) const
{
  this->getFeatureTrajectory(feature_id)->printFT();
}

void FeatureTrajectorySet::printFS(int frame_nr) const
{
  this->getFeatureSet(frame_nr)->printFS();
}

void FeatureTrajectorySet::printF(int feature_id, int frame_nr) const
{
  std::cout << "Feature " << feature_id << " in frame " << frame_nr << " : "
      << this->getFeatureSet(frame_nr)->getFeature(feature_id) << std::endl;
}

void FeatureTrajectorySet::writeToFileFTS(std::string file_name) const
{
  std::vector<FeatureSetPtr>::const_iterator fts_it = _feature_sets.begin();
  if (fts_it != _feature_sets.end())
  {
    // The first time it is written, the rest of the times it is appended
    (*fts_it)->writeToFileFS(file_name);
    fts_it++;
    for (; fts_it != _feature_sets.end(); fts_it++)
    {
      (*fts_it)->appendToFileFS(file_name);
    }
  }
  else
  {
    ROS_ERROR("Nothing wrote to file. FeatureTrajectorySet is empty.");
  }
}

void FeatureTrajectorySet::appendToFileFTS(std::string file_name) const
{
  std::vector<FeatureSetPtr>::const_iterator fts_it = _feature_sets.begin();
  if (fts_it == _feature_sets.end())
  {
    ROS_ERROR("Nothing wrote to file. FeatureTrajectorySet is empty.");
    return;
  }

  std::ofstream file_to_write;
  file_to_write.open(file_name.c_str(), std::ios::out | std::ios::app);
  if (!file_to_write.is_open())
  {
    std::cout << "ERROR[FeatureTrajectorySet::appendToFileCT(std::string file_name)]: The file couldn't be opened." << std::endl;
    return;
  }

  (*fts_it)->appendToFileFS(file_name);
  fts_it++;
  for (; fts_it != _feature_sets.end(); fts_it++)
  {
    (*fts_it)->appendToFileFS(file_name);
  }

  file_to_write << "==" << std::endl;

}


void FeatureTrajectorySet::readFromFileFTS(std::string file_name)
{
  std::ifstream file_to_read;
  file_to_read.open(file_name.c_str(), std::ios::in);

  readFromFileFTS(file_to_read);
}

void FeatureTrajectorySet::readFromFileFTS(std::ifstream &file_to_read)
{
  int frame = 0;
  do
  {
    FeatureSetPtr fs(new FeatureSet(frame++));
    fs->readFromFileFS(file_to_read);

    if (fs->getNumberOfFeatures() == 0)
      // this means we stumbled upon a ==
      return;

    addFeatureSet(fs);

    // there should be a last line containing the ==
//    getline(file_to_read, line);
//    if (line == "==")
//      break;

    // file eof or empty line
  } while (!file_to_read.eof());
}

double FeatureTrajectorySet::estimateAmountOfMotion(int first_frame, int last_frame) const
{
  int real_last_frame = (last_frame == 0 ? (this->_feature_sets.size() - 1) : last_frame);
  double sum = 0;
  FeaturePtr center_init = this->_feature_sets.at(first_frame)->findCenter();
  for (int i = first_frame + 1; i < real_last_frame; i++)
  {
    FeaturePtr center_second = this->_feature_sets.at(i)->findCenter();
    sum += math::abs(center_init, center_second);
    center_init = center_second;
  }
  return (sum / (double)this->_feature_sets.size());
}

std::vector<double> FeatureTrajectorySet::estimateFeaturesMotion(int first_frame, int last_frame) const
{
  std::vector<double> ret_val;
  if (this->_feature_sets.size() <= 1)
  {
    //this->printFTS();
    return ret_val;
  }
  int real_last_frame = (last_frame == 0 ? (this->_feature_sets.size() - 1) : last_frame);
  vision::FeatureSet::const_iterator first_frame_it = this->_feature_sets.at(first_frame)->begin();
  vision::FeatureSet::const_iterator first_frame_it_end = this->_feature_sets.at(first_frame)->end();
  vision::FeatureSet::const_iterator last_frame_it = this->_feature_sets.at(real_last_frame)->begin();
  vision::FeatureSet::const_iterator last_frame_it_end = this->_feature_sets.at(real_last_frame)->end();

  for (; first_frame_it != first_frame_it_end && last_frame_it != last_frame_it_end; first_frame_it++, last_frame_it++)
  {
    ret_val.push_back(math::abs((*first_frame_it), (*last_frame_it)));
  }
  return ret_val;
}

int FeatureTrajectorySet::findMostMovingFeature() const
{
  std::vector<double> motion;
  std::vector<int> feat_ids;
  FeatureSet::const_iterator fs_it = (*(this->begin()))->begin();
  FeatureSet::const_iterator fs_it_end = (*(this->begin()))->end();
  for (; fs_it != fs_it_end; fs_it++)
  {
    motion.push_back(this->getFeatureTrajectory((*fs_it)->getId())->estimateAmountOfMotion());
    feat_ids.push_back((*fs_it)->getId());
  }
  int index = 0;
  math::max(motion, index);
  return feat_ids.at(index);
}

FeatureSet::MotionDefinition FeatureTrajectorySet::estimateMotion(int first_frame, int last_frame) const
{
  //int real_last_frame = (last_frame == 0 ? (_feature_sets.size() - 1) : last_frame);

  return (this->getFeatureSet(first_frame)->estimateMotion(this->getFeatureSet(last_frame)));

  //  pcl::PointCloud<pcl::PointXYZ> first_cloud;
  //  pcl::PointCloud<pcl::PointXYZ> last_cloud;
  //
  //  this->getFeatureSet(first_frame)->toPCL(first_cloud);
  //  this->getFeatureSet(real_last_frame)->toPCL(last_cloud);
  //
  //  //NOTE: Be careful!!!!!
  //  //pcl_ros uses qtQuaternion which is constructed in the order of x, y, z, w
  //  //pcl uses Eigen::Quaternion which is constructed in the order of w, x, y, z
  //
  //  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> estimator;
  //  Eigen::Matrix4f T; // the result transformation
  //  estimator.estimateRigidTransformation( first_cloud, last_cloud, T );
  //
  //  ROS_DEBUG("\nTransformation: %.3f %.3f %.3f %.3f  \n \
//              %.3f %.3f %.3f %.3f  \n \
//              %.3f %.3f %.3f %.3f  \n \
//              %.3f %.3f %.3f %.3f",
  //          T(0,0),T(0,1),T(0,2),T(0,3),
  //          T(1,0),T(1,1),T(1,2),T(1,3),
  //          T(2,0),T(2,1),T(2,2),T(2,3),
  //          T(3,0),T(3,1),T(3,2),T(3,3));
  //
  //  Eigen::Matrix3f rot_matrix = T.corner(Eigen::TopLeft, 3, 3);
  //  Eigen::Quaterniond rotation(rot_matrix);
  //  std::vector<double> ret_rot;
  //  ret_rot.push_back(rotation.x());
  //  ret_rot.push_back(rotation.y());
  //  ret_rot.push_back(rotation.z());
  //  ret_rot.push_back(rotation.w());
  //
  //
  //  Eigen::Vector3f offset = T.corner(Eigen::TopRight, 3, 1);
  //  FeaturePtr ret_offset = FeaturePtr(new Feature(offset(0), offset(1), offset(2)));
  //
  //  return FeatureSet::MotionDefinition(ret_offset, ret_rot);
}

void FeatureTrajectorySet::clear()
{
  _feature_sets.clear();
}

FeatureTrajectorySetPtr FeatureTrajectorySet::clone() const
{
  return FeatureTrajectorySetPtr(doClone());
}

bool FeatureTrajectorySet::isLost(const int feature_id) const
{
  std::vector<FeatureSetPtr>::const_reverse_iterator it = _feature_sets.rbegin();
  FeaturePtr feat_in_last_frame = (*it)->getFeature(feature_id);
  if (feat_in_last_frame == 0)
  {
    ROS_ERROR("[FeatureTrajectorySet::isLost] This FeatureTrajectorySet does NOT contain a Feature with this Id (%d)", feature_id);
    return true;
  }
  else
  {
    return feat_in_last_frame->isLost();
  }
}

bool FeatureTrajectorySet::isContained(const int feature_id) const
{
  if (this->getTrajectoryLength() == 0)
  {
    return false;
  }
  return (*(this->begin()))->isContained(feature_id);
}

std::vector<std::vector<FeaturePtr> > FeatureTrajectorySet::FeatureTrajectorySet2Vector() const
{
  std::vector<std::vector<FeaturePtr> > ret_val;
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it = this->begin();
  FeatureTrajectorySet::const_trajectory_set_iterator fts_it_end = this->end();
  for (; fts_it != fts_it_end; fts_it++)
  {
    ret_val.push_back((*fts_it)->FeatureSet2Vector());
  }
  return ret_val;
}

void FeatureTrajectorySet::PublishInROS(ros::Publisher* publisher, int msec_delay) const
{
  FeatureTrajectorySet::const_trajectory_set_iterator ts_it = this->begin();
  FeatureTrajectorySet::const_trajectory_set_iterator ts_it_end = this->end();
  for (; ts_it != ts_it_end; ts_it++)
  {
    (*ts_it)->PublishInROS(publisher);
    usleep(msec_delay * 1000);
  }
}

}
