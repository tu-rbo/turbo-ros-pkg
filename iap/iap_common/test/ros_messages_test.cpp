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
#include "gtest/gtest.h"
#include "feature.h"
#include "FeatureSet.h"
#include "FeatureTrajectory.h"
#include "FeatureTrajectorySet.h"
#include "ClusterTrajectorySet.h"
#include "BodyTrajectoryPair.h"
#include "BodyTrajectory.h"

#include "feature_cluster_helpers.h"

#include <vector>
#include <iostream>

using namespace std;
using namespace vision;
using namespace iap_common;

TEST(RosMessagesTest, ReadWriteFeature)
{
  int num_frames = 5;

  ClusterTrajectoryPtr cluster0
   = generateRandomClusterTrajectory(1, 5, num_frames);

  ClusterTrajectoryPtr cluster1
   = generateRandomClusterTrajectory(2, 5, num_frames);

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);
  BodyTrajectoryPtr bt1 = cluster1->estimateBodyTrajectory(0, num_frames-1);

  BodyTrajectoryPairPtr btpair(new BodyTrajectoryPair(bt0, bt1));
  BodyTrajectoryPtr lt = btpair->getLocalTrajectory();

  BodyTrajectoryPairMsg msg = btpair->toROSMsg();
  BodyTrajectoryMsg bt0_msg = msg.body1_trajectory;
  BodyTrajectoryMsg bt1_msg = msg.body2_trajectory;
  BodyTrajectoryMsg lt_msg = msg.local_trajectory;

  EXPECT_EQ(bt0_msg.body_id, 1);
  EXPECT_EQ(bt0_msg.time_start, 0);
  EXPECT_EQ(bt0_msg.time_end, num_frames-1);
  EXPECT_EQ(bt0_msg.acc_rotation_angle, bt0->getAccRotationAngle());
  EXPECT_EQ(bt0_msg.max_rotation_angle, bt0->getMaxRotationAngle());
  EXPECT_EQ(bt0_msg.acc_translation_dist, bt0->getAccTranslationDistance());
  EXPECT_EQ(bt0_msg.max_translation_dist, bt0->getMaxTranslationDistance());

  EXPECT_EQ(bt1_msg.body_id, 2);
  EXPECT_EQ(bt1_msg.time_start, 0);
  EXPECT_EQ(bt1_msg.time_end, num_frames-1);
  EXPECT_EQ(bt1_msg.acc_rotation_angle, bt1->getAccRotationAngle());
  EXPECT_EQ(bt1_msg.max_rotation_angle, bt1->getMaxRotationAngle());
  EXPECT_EQ(bt1_msg.acc_translation_dist, bt1->getAccTranslationDistance());
  EXPECT_EQ(bt1_msg.max_translation_dist, bt1->getMaxTranslationDistance());

  EXPECT_EQ(lt_msg.body_id, lt->getBodyId()); // undefined
  EXPECT_EQ(lt_msg.time_start, 0);
  EXPECT_EQ(lt_msg.time_end, num_frames-1);
  EXPECT_EQ(lt_msg.acc_rotation_angle, lt->getAccRotationAngle());
  EXPECT_EQ(lt_msg.max_rotation_angle, lt->getMaxRotationAngle());
  EXPECT_EQ(lt_msg.acc_translation_dist, lt->getAccTranslationDistance());
  EXPECT_EQ(lt_msg.max_translation_dist, lt->getMaxTranslationDistance());


}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
