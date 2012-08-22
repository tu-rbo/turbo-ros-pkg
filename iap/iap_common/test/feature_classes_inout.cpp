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

#include "feature_cluster_helpers.h"

#include <vector>
#include <iostream>

using namespace std;
using namespace vision;


TEST(FeatureClassesInOutTest, ReadWriteFeature)
{
  FeaturePtr f1(new Feature(4.425,2.2,5.1));
  f1->setId(3);

  std::stringstream ss;
  ss << f1;
//  ROS_INFO_STREAM("f1 " << f1);

  FeaturePtr f2(new Feature);
  std::string s = ss.str();
  s >> f2;
//  ROS_INFO_STREAM("f2 " << f2);

  EXPECT_TRUE( *f1 == *f2 );
  EXPECT_TRUE( f1->getId() == f2->getId() );
}


TEST(FeatureClassesInOutTest, ReadWriteFeatureSet)
{
  std::string filename("_FeatureClassesInOutTest_FS.log");

  FeaturePtr f1(new Feature(4.425,2.2,5.1));
  f1->setId(3);
  FeaturePtr f2(new Feature(1.325,2.5,1.4));
  f2->setId(4);

  FeatureSetPtr fs1(new FeatureSet(1));
  fs1->addFeature(f1);
  fs1->addFeature(f2);
  fs1->writeToFileFS(filename);

  FeatureSetPtr fs2(new FeatureSet(1));
  fs2->readFromFileFS(filename);

  EXPECT_EQ(fs1->getNumberOfFeatures(), fs2->getNumberOfFeatures());

  FeatureSet::iterator it1=fs1->begin(), it2=fs2->begin();
  for (; it1 != fs1->end() && it2 != fs2->end(); it1++, it2++)
  {
//    ROS_INFO_STREAM("f1 " << (*it1));
//    ROS_INFO_STREAM("f2 " << (*it2));
    EXPECT_TRUE((**it1) == (**it2));
    EXPECT_TRUE((*it1)->getId() == (*it2)->getId());
  }

  // delete tmp file
  remove(filename.c_str());
}

TEST(FeatureClassesInOutTest, ReadWriteFeatureTrajectorySet)
{
  int num_features = 30;
  int num_frames = 10;

  std::string filename("_FeatureClassesInOutTest_FTS.log");

  ClusterTrajectoryPtr c1 = generateRandomClusterTrajectory(3, num_features, num_frames);
  c1->writeToFileFTS(filename);

  ClusterTrajectoryPtr c2(new ClusterTrajectory());
  c2->readFromFileFTS(filename);

  // compare c1 and c2
  for (int t = 1; t < num_frames; t++){
    FeatureSetPtr fs1 = c1->getFeatureSet(t);
    FeatureSetPtr fs2 = c2->getFeatureSet(t);
    EXPECT_EQ(fs1->getNumberOfFeatures(), fs2->getNumberOfFeatures());

    FeatureSet::iterator it1=fs1->begin(), it2=fs2->begin();
    for (; it1 != fs1->end() && it2 != fs2->end(); it1++, it2++)
    {
//      ROS_INFO_STREAM("f1 " << (*it1));
//      ROS_INFO_STREAM("f2 " << (*it2));
//      EXPECT_TRUE((**it1) == (**it2));
      EXPECT_TRUE((*it1)->getId() == (*it2)->getId());
      EXPECT_NEAR((*it1)->getX(), (*it2)->getX(), 0.001);
      EXPECT_NEAR((*it1)->getY(), (*it2)->getY(), 0.001);
      EXPECT_NEAR((*it1)->getZ(), (*it2)->getZ(), 0.001);
    }
  }

  // delete tmp file
  remove(filename.c_str());



  // check the same thing for writeToFileCluster!
  filename = "_FeatureClassesInOutTest_CT.log";
  c1->writeToFileCT(filename);

  c2.reset(new ClusterTrajectory());
  c2->readFromFileCT(filename);

  EXPECT_EQ(c1->getClusterId(), c2->getClusterId());

  // compare c1 and c2
  for (int t = 1; t < num_frames; t++){
    FeatureSetPtr fs1 = c1->getFeatureSet(t);
    FeatureSetPtr fs2 = c2->getFeatureSet(t);
    EXPECT_EQ(fs1->getNumberOfFeatures(), fs2->getNumberOfFeatures());

    FeatureSet::iterator it1=fs1->begin(), it2=fs2->begin();
    for (; it1 != fs1->end() && it2 != fs2->end(); it1++, it2++)
    {
//      ROS_INFO_STREAM("f1 " << (*it1));
//      ROS_INFO_STREAM("f2 " << (*it2));
//      EXPECT_TRUE((**it1) == (**it2));
      EXPECT_TRUE((*it1)->getId() == (*it2)->getId());
      EXPECT_NEAR((*it1)->getX(), (*it2)->getX(), 0.001);
      EXPECT_NEAR((*it1)->getY(), (*it2)->getY(), 0.001);
      EXPECT_NEAR((*it1)->getZ(), (*it2)->getZ(), 0.001);
    }
  }

  // delete tmp file
  remove(filename.c_str());

}

TEST(FeatureClassesInOutTest, ReadWriteClusterTrajectorySet)
{
  int num_features = 30;
  int num_frames = 10;

  std::string filename("_FeatureClassesInOutTest_CTS.log");

  std::vector<int> ids;
  ids.push_back(2);
  ids.push_back(3);

  ClusterTrajectoryPtr c1 = generateRandomClusterTrajectory(ids[0], num_features, num_frames);
  ClusterTrajectoryPtr c2 = generateRandomClusterTrajectory(ids[1], num_features, num_frames);

  ClusterTrajectorySetPtr cts1(new ClusterTrajectorySet());
  cts1->addClusterTrajectory(c1);
  cts1->addClusterTrajectory(c2);
  cts1->writeToFileCTS(filename);

  ClusterTrajectorySetPtr cts2(new ClusterTrajectorySet());
  cts2->readFromFileCTS(filename);

  for (int i = 0; i < 2; i++) {
    ClusterTrajectoryPtr c1 = cts1->getClusterTrajectory(ids[i]);
    ClusterTrajectoryPtr c2 = cts2->getClusterTrajectory(ids[i]);

    // compare c1 and c2
    for (int t = 1; t < num_frames; t++){
      FeatureSetPtr fs1 = c1->getFeatureSet(t);
      FeatureSetPtr fs2 = c2->getFeatureSet(t);
      EXPECT_EQ(fs1->getNumberOfFeatures(), fs2->getNumberOfFeatures());

      FeatureSet::iterator it1=fs1->begin(), it2=fs2->begin();
      for (; it1 != fs1->end() && it2 != fs2->end(); it1++, it2++)
      {
  //      ROS_INFO_STREAM("f1 " << (*it1));
  //      ROS_INFO_STREAM("f2 " << (*it2));
  //      EXPECT_TRUE((**it1) == (**it2));
        EXPECT_TRUE((*it1)->getId() == (*it2)->getId());
        EXPECT_NEAR((*it1)->getX(), (*it2)->getX(), 0.001);
        EXPECT_NEAR((*it1)->getY(), (*it2)->getY(), 0.001);
        EXPECT_NEAR((*it1)->getZ(), (*it2)->getZ(), 0.001);
      }
    }
  }

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
