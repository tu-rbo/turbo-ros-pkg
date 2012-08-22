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

#include <vector>
#include <iostream>

using namespace std;
using namespace vision;

TEST(FeatureClassesTest, Feature)
{
  FeaturePtr f1(new Feature(1, 2, 3));
  FeaturePtr f2;

  f2 = f1->clone();
  EXPECT_TRUE(f2 != NULL);
  EXPECT_TRUE(*f1 == *f1);
  EXPECT_TRUE(*f1 == *f2);

  FeatureSet set1(0);
  set1.addFeature(f1);
  //  set1.printFS();
  //  set1.printF(f1->getId());
  f2.reset();
  f2 = set1.getFeature(f1->getId());
  EXPECT_TRUE(f2);
  EXPECT_TRUE(f1 != f2); // different instantiations
  EXPECT_TRUE(*f1 == *f2); // same feature
}

TEST(FeatureClassesTest, BasicInOut)
{
  // feature 0 translates one step
  FeaturePtr f0t0(new Feature(1, 2, 3));
  int f0_id = f0t0->getId();
  FeaturePtr f0t1 = f0t0->clone();
  f0t1->setPos(2, 3, 4);
  // feature 1 does not move
  FeaturePtr f1t0(new Feature(10, 10, 10));
  int f1_id = f1t0->getId();
  FeaturePtr f1t1 = f1t0->clone();

  // Init feature set for feature 1
  FeatureSetPtr set1(new FeatureSet(0));
  EXPECT_TRUE(set1->getNumberOfFeatures() == 0);
  // Init feature set for feature 2
  FeatureSetPtr set2(new FeatureSet(1));
  EXPECT_TRUE(set2->getNumberOfFeatures() == 0);

  // add features to sets
  EXPECT_TRUE(set1->addFeature(f0t0));
  EXPECT_TRUE(set1->addFeature(f1t0));
  EXPECT_TRUE(set1->getNumberOfFeatures() == 2);

  EXPECT_TRUE(set2->addFeature(f0t1));
  EXPECT_TRUE(set2->addFeature(f1t1));
  EXPECT_TRUE(set2->getNumberOfFeatures() == 2);

  // get a feature 1 at t=0 from set 1
  FeaturePtr f0t0_b = set1->getFeature(f0_id);
  EXPECT_TRUE(f0t0_b);
  EXPECT_TRUE(*f0t0 == *f0t0_b);

  // probe for non-existing feature
  EXPECT_FALSE (set1->getFeature(-1));

  ///////////////////////////////////////////
  // create a feature trajectory set
  FeatureTrajectorySetPtr fts(new FeatureTrajectorySet());
  EXPECT_TRUE(fts->addFeatureSet(set1));
  EXPECT_TRUE(fts->addFeatureSet(set2));

  fts->printFTS();

  // Test a wrong initialization of FeatureTrajectorySet
  FeatureTrajectorySetPtr fts_false(new FeatureTrajectorySet());
  EXPECT_FALSE(fts_false->addFeatureSet(set1));
  // /test

  EXPECT_EQ(fts->getNumberOfFeatures(), 2);
  EXPECT_EQ(fts->getTrajectoryLength(), 2);

  // get a FeatureTrajectory
  // probe for non-existing trajectory
  FeatureTrajectoryPtr trajectory1 = fts->getFeatureTrajectory(-1);
  EXPECT_FALSE(trajectory1);

  // trajectory for feature f0_id
  trajectory1 = fts->getFeatureTrajectory(f0_id);
  EXPECT_TRUE (trajectory1);
  EXPECT_EQ (trajectory1->getTrajectoryLength(), 2);

  // get feature f0_id at t=0
  FeaturePtr f0t0_c = trajectory1->getFeature(0);
  EXPECT_TRUE (f0t0_c);
  EXPECT_TRUE (*f0t0_c == *f0t0);

  // get feature f0_id at t=1
  FeaturePtr f0t1_c = trajectory1->getFeature(1);
  EXPECT_TRUE (f0t1_c);
  EXPECT_TRUE (*f0t1_c == *f0t1);

  // probe for non existing feature
  EXPECT_FALSE (trajectory1->getFeature(2));

  // get a FeatureSet from FeatureTrajectorySet
  FeatureSetPtr set0_c = fts->getFeatureSet(0);
  // get feature f0_id at t=0
  FeaturePtr f0t0_d = set0_c->getFeature(f0_id);
  EXPECT_TRUE (f0t0_d);
  EXPECT_TRUE (*f0t0_d == *f0t0);

  // probe for non existing feature
  EXPECT_FALSE (set0_c->getFeature(-1));
}

////////////////////////////////////////////////////
TEST(FeatureClassesTest, SetIterator)
{
  FeaturePtr f0t0(new Feature(1, 2, 3));
  FeaturePtr f1t0(new Feature(10, 10, 10));
  FeaturePtr f2t0(new Feature(5, 3, 3));

  FeatureSetPtr set1(new FeatureSet(0));
  EXPECT_TRUE(set1->addFeature(f0t0));
  EXPECT_TRUE(set1->addFeature(f1t0));
  EXPECT_TRUE(set1->addFeature(f2t0));
  EXPECT_TRUE(set1->getNumberOfFeatures() == 3);

  FeaturePtr feat;
  // first
  FeatureSet::iterator it = set1->begin();
  feat = *it;
  EXPECT_TRUE(*feat == *f0t0);
  it++;
  // second
  feat = *it;
  EXPECT_TRUE(*feat == *f1t0);
  it++;
  // third
  feat = *it;
  EXPECT_TRUE(*feat == *f2t0);
  it++;
  // end
  EXPECT_TRUE(it == set1->end());

}

////////////////////////////////////////////////////
TEST(FeatureClassesTest, TrajectoryIterator)
{
  FeaturePtr f0t0(new Feature(1, 2, 3));

  //NO!! We have to clone the old Feature and modify the position to get the same Id
  //A second option would be to use setId(f0t0->getId()) after creating new Features, but is less elegant :)
  //Maybe we want 2 new functions: Feature::cloneAndUpdatePos(float x_new, float y_new)
  // and Feature::cloneAndUpdatePos(float x_new, float y_new, float z_new);

  //FeaturePtr f1t0(new Feature(10, 10, 10));
  //FeaturePtr f2t0(new Feature(5, 3, 3));
  FeaturePtr f0t1 = f0t0->clone();
  f0t1->setPos(10, 10, 10);
  FeaturePtr f0t2 = f0t0->clone();
  f0t2->setPos(5, 3, 3);

  FeaturePtr f1t0(new Feature(5, 3, 3));

  FeatureTrajectoryPtr set1(new FeatureTrajectory());
  EXPECT_TRUE(set1->addFeature(f0t0));
  EXPECT_TRUE(set1->addFeature(f0t1));
  EXPECT_TRUE(set1->addFeature(f0t2));
  EXPECT_FALSE(set1->addFeature(f1t0));
  EXPECT_TRUE(set1->getTrajectoryLength() == 3);

  FeaturePtr feat;
  // first
  FeatureTrajectory::iterator it = set1->begin();
  feat = *it;
  EXPECT_TRUE(*feat == *f0t0);
  it++;
  // second
  feat = *it;
  EXPECT_TRUE(*feat == *f0t1);
  it++;
  // third
  feat = *it;
  EXPECT_TRUE(*feat == *f0t2);
  it++;
  // end
  EXPECT_TRUE(it == set1->end());

}

////////////////////////////////////////////////////
TEST(FeatureClassesTest, Clustering)
{
  // feature 0 translates one step
  FeaturePtr f0t0(new Feature(1, 2, 3));
  int f0_id = f0t0->getId();
  FeaturePtr f0t1 = f0t0->clone();
  f0t1->setPos(2,3,4);
  // feature 1 does not move
  FeaturePtr f1t0(new Feature(10, 10, 10));
  int f1_id = f1t0->getId();
  FeaturePtr f1t1 = f1t0->clone();

  // Feature 2 moves mininally
  FeaturePtr f2t0(new Feature(5, 3, 3));
  int f2_id = f2t0->getId();
  FeaturePtr f2t1 = f2t0->clone();
  f2t1->setPos(5,3.1,3.1);

  // Init feature set for feature 1
  FeatureSetPtr set1(new FeatureSet(0));
  EXPECT_TRUE(set1->getNumberOfFeatures() == 0);
  // Init feature set for feature 2
  FeatureSetPtr set2(new FeatureSet(1));
  EXPECT_TRUE(set2->getNumberOfFeatures() == 0);

  // add features to sets
  EXPECT_TRUE(set1->addFeature(f0t0));
  EXPECT_TRUE(set1->addFeature(f1t0));
  EXPECT_TRUE(set1->addFeature(f2t0));
  EXPECT_TRUE(set1->getNumberOfFeatures() == 3);

  EXPECT_TRUE(set2->addFeature(f0t1));
  EXPECT_TRUE(set2->addFeature(f1t1));
  EXPECT_TRUE(set2->addFeature(f2t1));
  EXPECT_TRUE(set2->getNumberOfFeatures() == 3);

  // create a new feature trajectory set
  FeatureTrajectorySetPtr fts_cl(new FeatureTrajectorySet());
  EXPECT_TRUE(fts_cl->addFeatureSet(set1));
  EXPECT_TRUE(fts_cl->addFeatureSet(set2));
  EXPECT_EQ(fts_cl->getNumberOfFeatures(), 3);

  // Check the FeatureTrajectorySet
  EXPECT_TRUE(fts_cl->isConsistent());

  // let's cluster as follows: {0}, {1,2}
  vector<int> clustering1;
  clustering1.push_back(0);
  clustering1.push_back(1);
  clustering1.push_back(1);

  ClusterTrajectorySetPtr clustered_fts;

  EXPECT_NO_THROW(clustered_fts.reset(new ClusterTrajectorySet(fts_cl, clustering1)));

  EXPECT_EQ(clustered_fts->getTrajectoryLength(), 3);
  EXPECT_EQ(clustered_fts->getNumberOfClusters(), 2);
  EXPECT_EQ(clustered_fts->getClusterId(f0_id), 0);
  EXPECT_EQ(clustered_fts->getClusterId(f1_id), 1);
  EXPECT_EQ(clustered_fts->getClusterId(f2_id), 1);

  // TODO
  // clustered_fts->getFeatureSet(0, 0)
  // clustered_fts->getFeatureTrajectorySet(1)

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
