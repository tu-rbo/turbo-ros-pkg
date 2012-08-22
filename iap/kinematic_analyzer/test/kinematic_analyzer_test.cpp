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
 * kinematic_analizer_test.cpp
 *
 *  Created on: Jan 4, 2012
 *      Author: roberto
 */

#include "gtest/gtest.h"

#include "../../iap_common/include/iap_common/cluster_generator/cluster_generator.h"
#include "../../iap_common/include/iap_common/tracker/feature.h"
#include "../../iap_common/include/iap_common/tracker/FeatureSet.h"
#include "../../iap_common/include/iap_common/cluster/ClusterSet.h"
#include "KinematicAnalyzer.h"

using namespace std;
using namespace vision;

#define NUMBER_FEATS_PER_CLUSTER0 50
#define NUMBER_FEATS_PER_CLUSTER1 40
#define NUMBER_FEATS_PER_CLUSTER2 60
#define NUMBER_FRAMES 20

TEST(KinematicAnalyzerTests, KinematicAnalyse)
{
  // init random seed
  srand(time(NULL));

  FeaturePtr center_c0(new Feature(1, 1, 1));
  FeaturePtr center_c1(new Feature(2, 2, 2));
  FeaturePtr center_c2(new Feature(3, 3, 3));

  FeatureSetPtr fs(new FeatureSet(0));

  FeatureSetPtr fs0(new FeatureSet(0));
  FeatureSetPtr fs1(new FeatureSet(0));
  FeatureSetPtr fs2(new FeatureSet(0));

  std::vector<int> cluster_ids;

  for (int i = 0; i < NUMBER_FEATS_PER_CLUSTER0; i++)
  {
    FeaturePtr random = generateRandFeature(center_c0);
    fs->add(random);
    cluster_ids.push_back(0);
    fs0->add(random);
  }

  for (int i = 0; i < NUMBER_FEATS_PER_CLUSTER1; i++)
  {
    FeaturePtr random = generateRandFeature(center_c1);
    fs->add(random);
    cluster_ids.push_back(1);
    fs1->add(random);
  }

  for (int i = 0; i < NUMBER_FEATS_PER_CLUSTER2; i++)
  {
    FeaturePtr random = generateRandFeature(center_c2);
    fs->add(random);
    cluster_ids.push_back(2);
    fs2->add(random);
  }

  EXPECT_EQ(NUMBER_FEATS_PER_CLUSTER0 + NUMBER_FEATS_PER_CLUSTER1 + NUMBER_FEATS_PER_CLUSTER2, fs->size());

  FeatureTrajectorySetPtr fts(new FeatureTrajectorySet(fs->size()));

  fts->add(fs);
  //FeaturePtr translation(new Feature(0.05,0.05,0.05));
  FeaturePtr translation(new Feature(0,0,0));
  FeaturePtr translation_acc(new Feature(0,0,0));
  FeaturePtr translation2(new Feature(0,0,0));
  FeaturePtr translation_acc2(new Feature(0,0,0));
  Eigen::Quaterniond q(1,0,0,0);
  Eigen::Quaterniond q2(sqrt(0.5), 0,sqrt(0.5),  0);
  Eigen::Quaterniond q_acc(1,0,0,0);
  for(int j=1; j<NUMBER_FRAMES; j++)
  {
    FeatureSetPtr fs0_moved = fs0->move(translation_acc, q_acc);
    FeatureSetPtr temp = FeatureSetPtr(new FeatureSet(j));

    FeatureSetPtr fs0_movedg = fs0_moved->move(translation_acc2, q);

    for(FeatureSet::iterator it0 = fs0_movedg->begin(); it0!=fs0_movedg->end(); it0++)
    {
      temp->add((*it0));
    }

    FeatureSetPtr fs1_movedg = fs1->move(translation_acc2, q);
    for(FeatureSet::iterator it1 = fs1_movedg->begin(); it1!=fs1_movedg->end(); it1++)
    {
      temp->add((*it1));
    }

    FeatureSetPtr fs2ovedg = fs2->move(translation_acc2, q);
    for(FeatureSet::iterator it2 = fs2ovedg->begin(); it2!=fs2ovedg->end(); it2++)
    {
      temp->add((*it2));
    }
    fts->add(temp);
    translation_acc->setX(translation_acc->getX()+translation->getX());
    translation_acc->setY(translation_acc->getY()+translation->getY());
    translation_acc->setZ(translation_acc->getZ()+translation->getZ());

    translation_acc2->setX(translation_acc2->getX()+translation2->getX());
    translation_acc2->setY(translation_acc2->getY()+translation2->getY());
    translation_acc2->setZ(translation_acc2->getZ()+translation2->getZ());

    q_acc*=q2;
    std::cout << "quaternion: " << q_acc.w() << " "<< q_acc.x() << " "<< q_acc.y() << " "<< q_acc.z() << std::endl;

  }

  ClusterTrajectorySetPtr cts(new ClusterTrajectorySet(fts, cluster_ids));

  int innn = 0;
  for(ClusterTrajectorySet::iterator it = cts->begin(); it!=cts->end(); it++)
  {
    std::stringstream sst;
    sst<<innn;
    std::string ll = sst.str();

    it->second->writeToFileFTS(std::string("cts_") + ll + std::string(".txt"));
    innn++;

  }

  KinematicAnalyzer ka;

  ka.run(cts);

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

