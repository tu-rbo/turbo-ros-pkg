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
/**
 * The test is currently "inspect only", there are
 * no assertions
 */

#include "gtest/gtest.h"

#include "kinematic_analyzer/KinematicAnalyzer.h"
#include "kinematic_analyzer/joint_estimation/transformation_based/TransformationBasedJointEstimator.h"

#include "feature.h"
#include "FeatureSet.h"
#include "FeatureTrajectory.h"
#include "FeatureTrajectorySet.h"
#include "ClusterTrajectorySet.h"
#include "BodyTrajectoryPair.h"
#include "BodyTrajectory.h"

#include "../../test/feature_cluster_helpers.h"

#include "kinematic_analyzer/KinematicStructureMsg.h"
#include "iap_common/BodyTrajectoryMsg.h"
#include "iap_common/BodyTrajectoryPairMsg.h"

#include <vector>
#include <iostream>

using namespace std;
using namespace vision;
using namespace iap_common;
using namespace kinematic_analyzer;

TEST(RosMessagesTest, KinematicStructureMsg)
{
  int num_features = 4;
  int num_frames = 10;

  // static cluster
  ClusterTrajectoryPtr cluster0
   = generateRandomClusterTrajectory(1, num_features, num_frames, 0,0,0,0);

  // prismatic cluster
  ClusterTrajectoryPtr cluster1
   = generateStraightClusterTrajectory(2, num_features, num_frames);

  // merge the two clusters into one FeatureSetTrajectory with all
  // the features
  ClusterTrajectoryPtr merged =
      mergeClusterTrajectoriesSeparately(cluster0, cluster1, 0);

  merged->writeToFileCT("KinematicStructureMsg_CT.log");

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);
  BodyTrajectoryPtr bt1 = cluster1->estimateBodyTrajectory(0, num_frames-1);

  BodyTrajectoryPairPtr btpair(new BodyTrajectoryPair(bt0, bt1));
  BodyTrajectoryPtr lt = btpair->getLocalTrajectory();

  BodyTrajectoryPairMsg msg = btpair->toROSMsg();
  BodyTrajectoryMsg bt0_msg = msg.body1_trajectory;
  BodyTrajectoryMsg bt1_msg = msg.body2_trajectory;
  BodyTrajectoryMsg lt_msg = msg.local_trajectory;

  ////////////////

  KinematicAnalyzer* kinematic_analyzer = new KinematicAnalyzer;
  JointEstimatorPtr je(new TransformationBasedJointEstimator);
  kinematic_analyzer->setJointEstimator(je);


  for (int frame_nr = 0; frame_nr < num_frames; frame_nr++) {
    kinematic_analyzer->addFeatureSet3D(merged->getCluster(frame_nr));
  }

  std::vector<int> segmentation;
  // half to 0, half to 1
  for (int i = 0; i < num_features; i++) {
    segmentation.push_back(0);
  }
  for (int i = 0; i < num_features; i++) {
    segmentation.push_back(1);
  }

  kinematic_analyzer->setSegmentation(segmentation);

  kinematic_analyzer->estimateKinematicStructure();
  KinematicStructure ks = kinematic_analyzer->getKinematicStructure();

  KinematicStructureMsg ks_msg = ks.toROSMsg();

  ROS_INFO_STREAM(ks_msg);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
