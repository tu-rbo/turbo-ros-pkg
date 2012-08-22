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
 * FeatureBasedPrismaticJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "feature_based/FeatureBasedPrismaticJoint.h"

using namespace vision;

#include "MathFunctions.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std;

FeatureBasedPrismaticJoint::FeatureBasedPrismaticJoint() :
  Joint(), FeatureBasedJoint(), PrismaticJoint(), _line_goodness(0), _max_translation(0)
{
}

FeatureBasedPrismaticJoint::FeatureBasedPrismaticJoint(const FeatureBasedPrismaticJoint& tbpj) :
  Joint(tbpj), FeatureBasedJoint(tbpj), PrismaticJoint(tbpj), _line_goodness(tbpj._line_goodness),
      _max_translation(tbpj._max_translation)
{

}

FeatureBasedPrismaticJoint::~FeatureBasedPrismaticJoint()
{
}

void FeatureBasedPrismaticJoint::estimateAxis()
{
  unsigned int Nframes = this->_ct2->getTrajectoryLength();
  unsigned int Npoints = this->_ct2->getNumberOfFeatures();

  ClusterTrajectoryPtr body_local_ct = ClusterTrajectoryPtr(new ClusterTrajectory());

  for (int i = 0; i < Nframes; i++)
  {
    body_local_ct->addFeatureSet(
                                 this->_ct2->getCluster(i)->applyInverseRigidTransformation(
                                                                                            this->_btp->getBody1Trajectory()->getRigidTransformation(
                                                                                                                                                     i)));
  }

  std::vector<std::vector<FeaturePtr> > body_local = body_local_ct->FeatureTrajectorySet2Vector();

  std::vector<double> lineLengths;
  ROS_INFO_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis",
                        "Number of Features: " << Npoints << " Number of Frames: " << Nframes);

  std::vector<double> probabilityFramesInInterval;

  std::vector<std::vector<double> > distancesForEachFeature;

  int bestFeatureIndex = 0;

  this->_max_translation = math::calculateOverallMotion(body_local, bestFeatureIndex);

  int bestFrame = 0;
  int best_feat_best_frame = 0;
  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Best Feature: " << bestFeatureIndex);

  std::vector<vision::FeaturePtr> featureBestOverTime;

  // Extract the trajectory of the best feature
  for (unsigned int k = 0; k < Nframes; k++)
  {
    featureBestOverTime.push_back(body_local.at(k).at(bestFeatureIndex));
  }
  math::highestDistanceFeature(featureBestOverTime.at(0), featureBestOverTime, best_feat_best_frame);

  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Best Frame: " << best_feat_best_frame);

  vision::FeaturePtr lineStart = body_local.at(0).at(bestFeatureIndex);
  vision::FeaturePtr lineEnd = body_local.at(best_feat_best_frame).at(bestFeatureIndex);

  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Line start: " << lineStart);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Line end: " << lineEnd);

  double lineDistance = math::abs(lineStart, lineEnd);
  //double confidenceTubeInterval = math::abs(lineStart,lineEnd)*0.13;
  double confidenceTubeInterval = math::abs(lineStart, lineEnd) * 0.1;

  ROS_INFO_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Overall Motion: " << lineDistance);
  ROS_INFO_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "Interval: " << confidenceTubeInterval);

  std::vector<std::vector<double> > distanceVector;

  for (unsigned int i = 0; i < Npoints; i++)
  {

    // Extract the trajectory of the i Feature
    std::vector<vision::FeaturePtr> current_feature_trajectory;

    // Displace all Feature trajectories to start in the same position as the best Feature
    // Subtract the best Feature to the i-Feature at frame 0
    vision::FeaturePtr diffVector = math::sub(body_local.at(0).at(bestFeatureIndex), body_local.at(0).at(i));
    // Add the estimated difference of the frame 0 to all other frames
    for (unsigned int k = 0; k < Nframes; k++)
    {
      current_feature_trajectory.push_back(math::add(body_local.at(k).at(i), diffVector));
    }

    // Find the Feature in the trajectory that has the largest distance to starting position and store this maximal distance
    lineLengths.push_back(
                          math::highestDistanceFeature(current_feature_trajectory.at(0), current_feature_trajectory,
                                                       bestFrame));

    std::vector<vision::FeaturePtr> linePoints;

    std::vector<double> internalStructureAnalyse;

    std::vector<double> distanceStructure;
    double divisionStepSize = 0.2;

    unsigned int lowerBound = 0;
    unsigned int frameStepSize = int(ceil(divisionStepSize * Nframes));
    unsigned int upperBound = frameStepSize;

    while (upperBound < Nframes)
    {
      std::vector<double> distances;
      int numberOfFramesPerSection = 0;
      for (unsigned int k = lowerBound; k < upperBound; k++)
      {
        double current_distance_to_line = math::distanceFromLine(current_feature_trajectory.at(k), lineStart, lineEnd);
        distances.push_back(current_distance_to_line);
        distanceStructure.push_back(current_distance_to_line);
        numberOfFramesPerSection++;
      }
      internalStructureAnalyse.push_back(
                                         (double)math::numberSmallerThan(distances, confidenceTubeInterval)
                                             / (double)numberOfFramesPerSection);
      lowerBound += frameStepSize;
      upperBound += frameStepSize;
    }

    probabilityFramesInInterval.push_back(math::min(internalStructureAnalyse));
    distancesForEachFeature.push_back(distanceStructure);
  }

  double lineLengthMax = math::max(lineLengths);
  for (unsigned int i = 0; i < lineLengths.size(); i++)
  {
    lineLengths.at(i) /= lineLengthMax;
  }
  double probabilityEqualLines = 1.0 - math::qerf(math::mean(lineLengths), 0.930, 40.0); //0.930,40.0
  double pFeaturesInInterval = (double)math::sum(probabilityFramesInInterval)
      / (double)(probabilityFramesInInterval.size());

  ROS_INFO_STREAM_NAMED(
                        "FeatureBasedPrismaticJoint.estimateAxis",
                        "Prismatic Fit: Quality: " << pFeaturesInInterval << " EqualLines: " << probabilityEqualLines
                            << " Complete: " << pFeaturesInInterval * probabilityEqualLines);
  this->_line_goodness = pFeaturesInInterval * probabilityEqualLines;

  // In the prismatic case, the best axis is probably the one oriented between the
  // mean vision::FeaturePtrs of the first and the last frame (or the two frames which have the highest distance
  // between them)
  vision::FeaturePtr center_body_first_frame = math::mean(body_local.at(0));
  vision::FeaturePtr center_body_last_frame = math::mean(body_local.at(bestFrame));

  this->_axis.position = this->_ct2->getCluster(Nframes - 1)->findCenter();
  this->_axis.orientation = math::sub(center_body_last_frame, center_body_first_frame);
  this->_axis.orientation->normalize();

  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "AxisOfTranslationPVec: " << this->_axis.position);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis",
                         "AxisOfTranslationDirVec: " << this->_axis.orientation);

  if (this->_line_goodness > 1.0 || this->_line_goodness < 0.0)
  {
    ROS_ERROR_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis", "The goodness of the line fit is out of bounds!");
    ROS_ERROR_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis",
                           "math::mean(lineLengths): " << math::mean(lineLengths));
    ROS_ERROR_STREAM_NAMED("FeatureBasedPrismaticJoint.estimateAxis",
                           "math::qerf: " << math::qerf(math::mean(lineLengths), 0.930, 40.0));
  }
}

double FeatureBasedPrismaticJoint::getGoodnessOfFit()
{
  // TODO
  return this->_line_goodness;
}

::iap_common::JointMsg FeatureBasedPrismaticJoint::toROSMsg()
{
  ::iap_common::JointMsg msg = PrismaticJoint::toROSMsg();
  FeatureBasedJoint::toROSMsg(msg);
  return msg;
}

double FeatureBasedPrismaticJoint::getJointOrientationUncertainty() const
{
  double ori_uncertainty = 1.;
  if (this->_max_translation < this->_min_motion / 2.)
  {
    ori_uncertainty = 1.;
  }
  else if (this->_max_translation > 3.0 * this->_min_motion / 2.0)
  {
    ori_uncertainty = 0.01;
  }
  else
  {
    ori_uncertainty = 1.0 - (this->_max_translation - this->_min_motion / 2.0) * 0.9 / this->_min_motion;
  }
  return (ori_uncertainty*(1 - this->_line_goodness));
}
