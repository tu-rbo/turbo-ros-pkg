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
#include "KinematicAnalyzer.h"

#include "BodyTrajectory.h"

#include "ros/ros.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "recursive/RecursiveJoint.h"

using namespace vision;

#define MINIMUM_NUM_FEATS 1

iap_common::KinematicStructureMsg KinematicStructure::toROSMsg()
{
  iap_common::KinematicStructureMsg msg;
  msg.links = this->size();

  for (const_iterator it = this->begin(); it != this->end(); it++)
  {
    msg.joint_estimator_results.push_back(it->second->toROSMsg());
  }

  return msg;
}

KinematicAnalyzer::KinematicAnalyzer() :
  _interval_selection(ONA), _interval_selection_overlap(10), _frame_counter(0)
{
  this->_accumulator_3d = FeatureTrajectorySetPtr(new FeatureTrajectorySet());
}

KinematicAnalyzer::~KinematicAnalyzer()
{
}

void KinematicAnalyzer::setLastRGBImage(cv::Mat last_image)
{
  this->_last_image = last_image;
}

void KinematicAnalyzer::addFeatureSet3D(FeatureSetPtr feature_set)
{
  this->_accumulator_3d->addFeatureSet(feature_set->clone());
  this->_frame_counter++;
}

void KinematicAnalyzer::setSegmentation(std::vector<int> &new_segmentation)
{
  // If a Feature has or had (or both, had and has) clusterId=-1, it will get clusterId=-1 again

  if (this->_segmentation.size() == 0) // If there is not a previous segmentation we just copy the new one
  {
    this->_segmentation = new_segmentation;
    return;
  }
  else // If there is a previous segmentation we merge both
  {
    int merged_cluster_id = 1; // New ClusterId for the resulting clusters
    std::map<std::pair<int, int>, int> merged_segments;
    std::vector<int> merged_segmentation;

    std::vector<int>::iterator int_it_new = new_segmentation.begin();
    std::vector<int>::iterator int_it_new_end = new_segmentation.end();
    std::vector<int>::iterator int_it_previous = this->_segmentation.begin();
    std::vector<int>::iterator int_it_previous_end = this->_segmentation.end();
    for (; int_it_new != int_it_new_end && int_it_previous != int_it_previous_end; int_it_new++, int_it_previous++)
    {
      std::pair<int, int> new_and_previous_ids = std::make_pair((*int_it_new), (*int_it_previous));
      std::map<std::pair<int, int>, int>::iterator it = merged_segments.find(new_and_previous_ids);
      if ((*int_it_new) == -1 || (*int_it_previous) == -1)
      {
        merged_segmentation.push_back(-1);
      }
      else if (it == merged_segments.end()) // This combination of new and previous ClusterIDs wasn't seen before
      {
        merged_segments.insert(std::make_pair(new_and_previous_ids, merged_cluster_id));
        merged_segmentation.push_back(merged_cluster_id);
        merged_cluster_id++;
      }
      else // This combination of new and previous ClusterIDs was seen before
      {
        merged_segmentation.push_back(it->second);
      }
    }
    this->_segmentation = merged_segmentation;
  }
}

void KinematicAnalyzer::estimateKinematicStructure()
{
  ROS_INFO_STREAM_NAMED("KinematicAnalyzer.estimateKinematicStructure",
      "Number of frames to analyze: " << this->_accumulator_3d->getTrajectoryLength());
  if(this->_segmentation.size() == 0)
  {
  ROS_ERROR_STREAM_NAMED("KinematicAnalyzer.estimateKinematicStructure",
      "No segmentation received. We can't run the analysis of the kinematic structure.");
  }
  // Clean the previous results
  this->_kinematic_structure.clear();
  if (!this->_joint_estimator)
  {
    ROS_ERROR("[KinematicAnalyzer::estimateKinematicStructure()] The JointEstimator object was not set!");
    return;
  }

  // Make a copy of the last received image before starting the analysis
  this->_last_image_cpy = this->_last_image.clone();

  // Check if any point got lost after we received the segmentation, and update the segmentation to have -1 for them
  FeatureTrajectorySet::const_trajectory_set_rev_iterator ftsrev_it = this->_accumulator_3d->rbegin();
  FeatureSet::const_iterator fs_it = (*ftsrev_it)->begin();
  FeatureSet::const_iterator fs_it_end = (*ftsrev_it)->end();
  int segm_index = 0;
  for (; fs_it != fs_it_end; fs_it++)
  {
    if (((*fs_it)->getX() == -1. && (*fs_it)->getY() == -1. && (*fs_it)->getZ() == -1.) || ((*fs_it)->getX() == -2.
        && (*fs_it)->getY() == -2. && (*fs_it)->getZ() == -2.))
    {
      this->_segmentation.at(segm_index) = -1;
    }

    segm_index++;
  }
  // Generate a ClusterTrajectorySet from the accumulated 3D points and the segmentation
  ClusterTrajectorySetPtr cts = ClusterTrajectorySetPtr(
                                                        new ClusterTrajectorySet(this->_accumulator_3d,
                                                                                 this->_segmentation));
  // Pass every pair of ClusterTrajectories to the joint estimator to generate a joint hypothesis
  // Store the result in a map of int(clusterid1)-int(clusterid2) to JointEstimatorResultPtr
  ClusterTrajectorySet::const_iterator ct_it1 = cts->begin();
  ClusterTrajectorySet::const_iterator ct_it_end = cts->end();
  for (; ct_it1 != ct_it_end; ct_it1++)
  {
    ClusterTrajectorySet::const_iterator ct_it2 = ct_it1;
    while (++ct_it2 != ct_it_end)
    {
      this->_kinematic_structure[std::pair<int, int>((*ct_it1)->getClusterId(), (*ct_it2)->getClusterId())]
          = this->_joint_estimator->execute(*ct_it1, *ct_it2);
    }
  }

  this->drawAxes();

  switch (this->_interval_selection)
  {
    case ONA: // Not cleaning the accumulated 3D feature sets
      break;
    case NOA: // Cleaning all accumulated 3D feature sets
      this->_accumulator_3d->clear();
      break;
    case OA: // Storing the last OVERLAPPING_WINDOW frames and cleaning the rest
    {
      FeatureTrajectorySetPtr acc_copy_temp = this->_accumulator_3d->clone();
      this->_accumulator_3d->clear();
      for (int i = this->_interval_selection_overlap; i > 0; i--)
      {
        FeatureSetPtr temp_fs = acc_copy_temp->getFeatureSet(acc_copy_temp->getTrajectoryLength() - i);
        this->_accumulator_3d->addFeatureSet(temp_fs);
      }
      break;
    }
    default:
    {
      break;
    }
  }

}

#define FOCAL_LENGTH 533.301244

void KinematicAnalyzer::drawAxes()
{
//  std::vector<cv::Scalar> colors_strong;
//  colors_strong.push_back(cv::Scalar(0, 255, 0, 0));//green
//  colors_strong.push_back(cv::Scalar(255, 0, 0, 0));//blue
//  colors_strong.push_back(cv::Scalar(0, 0, 255, 0));//red
//  colors_strong.push_back(cv::Scalar(0, 0, 0, 0));//black?
//  colors_strong.push_back(cv::Scalar(255, 255, 255, 0)); //white?
//
//  KinematicStructure::iterator ks_it = this->_kinematic_structure.begin();
//  KinematicStructure::iterator ks_it_end = this->_kinematic_structure.end();
//
//  cv::Scalar zero_scalar = cvScalar(0., 0., 0., 0.);
//  std::vector<cv::Point3f> points_3d;
//  std::vector<cv::Point2f> points_2d;
//  cv::Mat rotation = cv::Mat(3, 1, CV_32FC1, zero_scalar);
//  cv::Mat frame;
//  cv::Mat distorsion = cv::Mat(4, 1, CV_32FC1, zero_scalar);
//
//  distorsion.at<float> (0, 0) = 0.182848;
//  distorsion.at<float> (1, 0) = -0.289798;
//  distorsion.at<float> (2, 0) = 0.005950;
//  distorsion.at<float> (3, 0) = 0.002582;
//
//  cv::Mat cam_mat = cv::Mat(3, 3, CV_32FC1, zero_scalar);
//  cam_mat.at<float> (0, 0) = FOCAL_LENGTH;
//  cam_mat.at<float> (1, 1) = FOCAL_LENGTH;
//  cam_mat.at<float> (2, 2) = 1.;
//  cam_mat.at<float> (0, 2) = 314.41;//(double)this->_last_image_cpy.rows / 2.;
//  cam_mat.at<float> (1, 2) = 256.; //(double)this->_last_image_cpy.cols / 2.;
//  cv::Mat translation = cv::Mat(3, 1, CV_32FC1, zero_scalar);
//  translation.at<float> (0, 0) = 0.;
//  translation.at<float> (1, 0) = 0.;
//  translation.at<float> (2, 0) = 0.;
//  int color_idx = 0;
//  for (; ks_it != ks_it_end; ks_it++)
//  {
//    points_2d.clear();
//    points_3d.clear();
//    JointPtr most_probable_joint = ks_it->second->getMostProbableJoint();
//    FeaturePtr posi = most_probable_joint->getAxis().position;
//    if (most_probable_joint->getType() == PRISMATIC_JOINT || most_probable_joint->getType() == REVOLUTE_JOINT)
//    {
//      FeaturePtr ori = most_probable_joint->getAxis().orientation;
//      FeaturePtr point1 = posi - ori;
//      FeaturePtr point2 = posi + ori;
//      points_3d.push_back(cv::Point3f(point1->getX(), point1->getY(), point1->getZ()));
//      points_3d.push_back(cv::Point3f(point2->getX(), point2->getY(), point2->getZ()));
//
//      cv::projectPoints(cv::Mat(points_3d), rotation, translation, cam_mat, distorsion, points_2d);
//
//      cv::circle(this->_last_image_cpy, points_2d.at(0), 2, colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//      cv::circle(this->_last_image_cpy, points_2d.at(1), 2, colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//      cv::line(this->_last_image_cpy, points_2d.at(0), points_2d.at(1),
//               colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//    }
//    else
//    {
//      points_3d.push_back(cv::Point3f(posi->getX(), posi->getY(), posi->getZ()));
//      cv::projectPoints(cv::Mat(points_3d), rotation, translation, cam_mat, distorsion, points_2d);
//      cv::circle(this->_last_image_cpy, points_2d.at(0), 4, colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//    }
//
//    JointPtr prism_joint = ks_it->second->getJointHypothesis(PRISMATIC_JOINT);
//    FeaturePtr prism_posi = prism_joint->getAxis().position;
//    FeaturePtr prism_ori = prism_joint->getAxis().orientation;
//    //    FeaturePtr point1 = prism_posi - prism_ori;
//    //          FeaturePtr point2 = prism_posi + prism_ori;
//    //          points_3d.push_back(cv::Point3f(point1->getX(), point1->getY(), point1->getZ()));
//    //          points_3d.push_back(cv::Point3f(point2->getX(), point2->getY(), point2->getZ()));
//    //
//    //          cv::projectPoints(cv::Mat(points_3d), rotation, translation, cam_mat, distorsion, points_2d);
//    //
//    //          cv::circle(this->_last_image_cpy, points_2d.at(0), 2, colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//    //          cv::circle(this->_last_image_cpy, points_2d.at(1), 2, colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//    //          cv::line(this->_last_image_cpy, points_2d.at(0), points_2d.at(1),
//    //                   colors_strong.at(color_idx % (colors_strong.size() - 1)), 2);
//
//    JointPtr rev_joint = ks_it->second->getJointHypothesis(REVOLUTE_JOINT);
//    FeaturePtr rev_posi = rev_joint->getAxis().position;
//    FeaturePtr rev_ori = rev_joint->getAxis().orientation;
//
//    if (!this->_last_image_cpy.empty())
//    {
//      cv::imshow("Resulting Axes", this->_last_image_cpy);
//      cv::waitKey(-1);
//    }
//
//    color_idx++;
//  }

  //cv::imwrite("resulting_axis.jpg", this->_last_image_cpy);
}

void KinematicAnalyzer::setJointEstimator(JointEstimatorPtr je)
{
  this->_joint_estimator = je->clone();
}

JointEstimatorResultPtr KinematicAnalyzer::getJointEstimatorResult(int cluster_id1, int cluster_id2) const
{
  if (this->_kinematic_structure.find(std::pair<int, int>(cluster_id1, cluster_id2))
      != this->_kinematic_structure.end())
  {
    return this->_kinematic_structure.at(std::pair<int, int>(cluster_id1, cluster_id2));
  }
  else
  {
    return JointEstimatorResultPtr(new JointEstimatorResult());
  }
}

KinematicStructure KinematicAnalyzer::getKinematicStructure() const
{
  return this->_kinematic_structure;
}

void KinematicAnalyzer::reset()
{
  this->_kinematic_structure.clear();
  this->_accumulator_3d->clear();
  this->_segmentation.clear();
  this->_joint_estimator->reset();
}

void KinematicAnalyzer::setIntervalSelectionCriterion(IntervalSelection interval_selection)
{
  this->_interval_selection = interval_selection;
}

void KinematicAnalyzer::setIntervalSelectionOverlap(int overlap) {
  this->_interval_selection_overlap = overlap;
}
