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
#include "kinematics/BodyTrajectory.h"
#include "kinematics/BodyTrajectoryPair.h"

//#include "MathFunctions.h"
#include "FeatureTrajectorySet.h"
#include "FeatureSet.h"
#include "ClusterTrajectory.h"
#include "Cluster.h"

#include "feature_cluster_helpers.h"

#include <vector>
#include <iostream>

//using namespace std;
using namespace vision;


TEST(BodyTrajectoryTest, mergeClusterWithZero)
{

  int num_features = 1;
  int num_frames = 2;

  ClusterTrajectoryPtr cluster_empty
   = generateRandomClusterTrajectory(0, num_features, num_frames, 0,0,0,0);
  ClusterTrajectoryPtr cluster1
   = generateRandomClusterTrajectory(1, num_features, num_frames);

  ClusterTrajectoryPtr cmerged =
      mergeClusterTrajectories(cluster_empty, cluster1, 10);

  EXPECT_EQ (cluster1->getTrajectoryLength(), cmerged->getTrajectoryLength());


  // check that cmerged == cluster1
  for (int t = 0; t < cmerged->getTrajectoryLength(); t++)
  {
    ClusterPtr cm = cmerged->getCluster(t);
    ClusterPtr c1 = cluster1->getCluster(t);

    Cluster::iterator itm = cm->begin(),
        it1 = c1->begin();
    for (; itm != cm->end() && it1 != c1->end();
          (itm++), (it1++)) {
      EXPECT_TRUE( **itm == **it1);
    }
  }
}

TEST(BodyTrajectoryTest, localMotionTranslation)
{
  // idea:
  //  create two random (consistent) cluster trajectories c1, c2
  //  calculate the sum of both: cm = c0 + c1
  //  then calculate the relative motion of c1 to cm
  //    which should result in (c0+c1) - c1 = c0

  int num_features = 10;
  int num_frames = 2;

  ClusterTrajectoryPtr cluster0
   = generateRandomClusterTrajectory(0, num_features, num_frames);
  ClusterTrajectoryPtr cluster1
   = generateRandomClusterTrajectory(1, num_features, num_frames);

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);
  BodyTrajectoryPtr bt1 = cluster1->estimateBodyTrajectory(0, num_frames-1);

  RigidTransformationPtr rt0 = bt0->getRigidTransformation(1);
  RigidTransformationPtr rt1 = bt1->getRigidTransformation(1);

  FeaturePtr fc0_t0_0 = cluster0->getCluster(0)->getFeature(0);
  FeaturePtr fc0_t1_0 = cluster0->getCluster(1)->getFeature(0);

  // Translation of Feature 0 of cluster 0 from t=0 -> t=1
  FeaturePtr trans_fc0_0 = fc0_t1_0 - fc0_t0_0;
//  ROS_INFO_STREAM("feature 0 at t0 " << fc0_t0_0);
//  ROS_INFO_STREAM("feature 0 at t1 " << fc0_t1_0);
//  ROS_INFO_STREAM("trans 0 at t0->t1 " << trans_fc0_0);

  Eigen::Transform<double, 3, Eigen::Affine> t0 = rt0->getTransformationMatrix();
//  ROS_INFO_STREAM("t0 " << t0.matrix());
//  ROS_INFO_STREAM(t0.matrix().col(3));

  // check that translation appear in the matrix of the
  // calculated rigid transformation
  EXPECT_NEAR(t0.matrix().col(3)[0], trans_fc0_0->getX(), 0.001);
  EXPECT_NEAR(t0.matrix().col(3)[1], trans_fc0_0->getY(), 0.001);
  EXPECT_NEAR(t0.matrix().col(3)[2], trans_fc0_0->getZ(), 0.001);

  // merge clusters
  ClusterTrajectoryPtr cmerged = mergeClusterTrajectories(cluster0, cluster1, 10);
  BodyTrajectoryPtr btm = cmerged->estimateBodyTrajectory(0, num_frames-1);

  // compute local motion c0 - cm
  BodyTrajectoryPairPtr btpair_plus(new BodyTrajectoryPair(bt1, btm));
  BodyTrajectoryPtr bt1_2_plus = btpair_plus->getLocalTrajectory();
  RigidTransformationPtr rt1_2_plus = bt1_2_plus->getRigidTransformation(1);

  // get the transformation matrix
  Eigen::Transform<double, 3, Eigen::Affine> tpair_plus = rt1_2_plus->getTransformationMatrix();

  // check that translation of local trajectory and c1 is the same
  for (int idx = 0; idx < 3; idx++) {
    EXPECT_NEAR(t0.matrix().col(3)[idx], tpair_plus.matrix().col(3)[idx], 0.001);
  }

  // compute local motion cm - c0
  // we expected it to be reversed in sign
  BodyTrajectoryPairPtr btpair_minus(new BodyTrajectoryPair(btm, bt1));
  BodyTrajectoryPtr bt1_2_minus = btpair_minus->getLocalTrajectory();
  RigidTransformationPtr rt1_2_minus = bt1_2_minus->getRigidTransformation(1);

  // get the transformation matrix
  Eigen::Transform<double, 3, Eigen::Affine> tpair_minus = rt1_2_minus->getTransformationMatrix();

  // check that translation of local trajectory and c1 has reversed sign
  for (int idx = 0; idx < 3; idx++) {
    EXPECT_NEAR(t0.matrix().col(3)[idx], - tpair_minus.matrix().col(3)[idx], 0.001);
  }

//  ROS_INFO_STREAM("t0 " << t0.matrix());
//  ROS_INFO_STREAM("tpair_plus " << tpair_plus.matrix());
//  ROS_INFO_STREAM("tpair_minus " << - tpair_minus.matrix());

  EXPECT_TRUE(true);
}

TEST(BodyTrajectoryTest, translationParametersStraight)
{
  int num_features = 50;
  int num_frames = 5;

  int steps = 300;

  for (int i = 0; i < steps; i++) {

    ClusterTrajectoryPtr cluster0
     = generateStraightClusterTrajectory(0, num_features, num_frames);

    // print trajectory
//  printClusterTrajectory(cluster0);
    BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);

    // check that the translational parts of the consecutive matrices
    // are multiples of the first movement

//    ROS_INFO_STREAM (" bt0(1) matrix: " <<
//        bt0.getRigidTransformation(1).getTransformationMatrix().matrix());

    for (int t = 2; t < num_frames; t++){
//      ROS_INFO_STREAM (" bt0(" << t << ") matrix: " <<
//          bt0.getRigidTransformation(t).getTransformationMatrix().matrix());

      for (int idx = 0; idx < 3; idx++) {
        EXPECT_NEAR(
            t*bt0->getRigidTransformation(1)->getTransformationMatrix().matrix().col(3)[idx],
            bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(3)[idx],
            0.001);
      }
    }

    // we assume the axis to be almost 1 since it moves in an absolutely straight line
//    ROS_INFO_STREAM ("TranslationAxisParallelism " << bt0.getTranslationAxisParallelism());

    if (bt0->getTranslationAxisParallelism() < 0.99999) {
      ROS_WARN("TEST(BodyTrajectoryTest, translationParametersStraight): \
          translation axis is %.10f", bt0->getTranslationAxisParallelism());
      EXPECT_NEAR(bt0->getTranslationAxisParallelism(), 1.0, 1.0 - 0.99999);
      break;
    }

    // we assume the accumulated translation distance to be the
    //

  }

  ROS_WARN("TEST(BodyTrajectoryTest, translationParametersStraight): everything fine!");
}

TEST(BodyTrajectoryTest, translationParametersRandom)
  {

  int num_features = 50;
  int num_frames = 5;

  // check the parameters for a completely random trajectory
//  ROS_INFO("RANDOM cluster trajectory" );

  double meanParallelism = 0.0;
  int steps = 300;
  for (int i = 0; i < steps; i++) {
    ClusterTrajectoryPtr cluster1
     = generateRandomClusterTrajectory(1, num_features, num_frames,
                                       50,10,50,-50);
    BodyTrajectoryPtr bt1 = cluster1->estimateBodyTrajectory(0, num_frames-1);

    // print trajectory
//  printClusterTrajectory(cluster0);
//    ROS_INFO_STREAM("rnd - TranslationAxisParallelism: " << bt1.getTranslationAxisParallelism());

    meanParallelism += bt1->getTranslationAxisParallelism();

  }

  // expected mean value is approximately 0.5, as angles should be
  // uniformly distributed
  EXPECT_TRUE(meanParallelism / steps < 0.6);
  EXPECT_TRUE(meanParallelism / steps > 0.4);

}

/**
 * Perform 4 (identical) random degree rotations of a point cloud and verify
 * that each Body Transformation matrix reflects the angle
 * of rotation
 *
 * then verify the motion parameters
 */
TEST(BodyTrajectoryTest, randomRotation)
{
  int num_features = 10;
  int num_frames = 4;
  // axis parallelism will only be calculated if num_frames > 3

  double angle = 2.45543;
//  double angle = uniform(M_PI, M_PI_4); // 45 - 180 degrees
  // if we make it higher (e.g. PI) the test might not work
  // anymore as the body trajectory estimator will take the smaller angle

  ClusterTrajectoryPtr cluster0
   = generateCurvedClusterTrajectory(0, num_features, num_frames, angle);

  // print trajectory
//  printClusterTrajectory(cluster0);

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);

  ROS_ERROR_STREAM("TrajectoryLength " << cluster0->getTrajectoryLength());

//  ROS_INFO_STREAM (" bt0(1) matrix: \n" <<
//      bt0->getRigidTransformation(1)->getTransformationMatrix().matrix());

  for (int t = 1; t < num_frames; t++){
    // check
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[0],
        cos(t*angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[1],
        sin(t*angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[0],
        -sin(t*angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[1],
        cos(t*angle),
        0.00001);
  }

  ROS_WARN_STREAM("angle: " << (angle));
  ROS_WARN_STREAM("num_frames-1 * angle: " << ((num_frames-1) * angle));
  ROS_WARN_STREAM("bt0.getAccRotationAngle(): " << bt0->getAccRotationAngle());
  ROS_WARN_STREAM("bt0.getMaxRotationAngle(): " << bt0->getMaxRotationAngle());

  // The accumulated rotation angle should be (number of rotations) * angle
  EXPECT_NEAR ((num_frames-1) * angle,
               bt0->getAccRotationAngle(), 0.00001);

  // The max rotation angle cannot get higher than pi.
  // It also only looks at each configuration (0, t), which
  // maximal at 2.45543
  // because 2.45543+2.45543=4.91
  // and 2pi - 4.91 = 1.37 < 2.45
  EXPECT_NEAR (2.45543,
               bt0->getMaxRotationAngle(), 0.0001);

  // The Rotation Axis Parallelism should be almost 1.0
  ROS_WARN_STREAM("rotAxisParall: " << bt0->getRotationAxisParallelism());
  EXPECT_NEAR (1.0,
               bt0->getRotationAxisParallelism(), 0.000001);

}

/**
 * Perform 4 (different) random degree rotations of a point cloud and verify
 * that each Body Transformation matrix reflects the angle of rotation
 *
 * The angles are going to be + and -, so that max will differ from acc angle
 * then verify the motion parameters
 */
TEST(BodyTrajectoryTest, randomRotationWithDifferentAngles)
{

  int num_features = 10;
  int num_frames = 4;

  std::vector<double> angles;

//  double angle = 1.0 * M_PI; // 90 degrees
//  for (int t = 0; t < num_frames - 1; t++) {
//    int sign = t % 2 == 0 ? 1 : -1;
//    angles.push_back(sign * uniform(M_PI, M_PI_2)); // 45 - 90 degrees
//  }

  // motions must be smaller than pi otherwise they are not recognize
  angles.push_back(2.5);
  angles.push_back(-2.7);
  angles.push_back(1.2);

  ClusterTrajectoryPtr cluster0
   = generateCurvedClusterTrajectory(0, num_features, num_frames, angles);

  // print trajectory
//  printClusterTrajectory(cluster0);

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);

//  ROS_INFO_STREAM (" bt0(1) matrix: \n" <<
//      bt0->getRigidTransformation(1)->getTransformationMatrix().matrix());

  double acc_angle = 0;
  for (int t = 1; t < num_frames; t++){
//    ROS_WARN_STREAM("angle[" << (t-1) << "]: " << (angles[t-1]));
//    ROS_WARN_STREAM("mtx: \n" <<
//       bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().topLeftCorner(3,3)
//    );

    double angle = 0;

    // Eigen always gives back a value between [0, 2 pi]
    // and switches the direction of the rotation axis instead
    acc_angle += angles[t-1];
    if (acc_angle < 0)
      angle = 2*M_PI + acc_angle;
    else
      angle = acc_angle;

    // check
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[0],
        cos(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[1],
        sin(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[0],
        -sin(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[1],
        cos(angle),
        0.00001);
  }

  // The accumulated rotation angle should be the sum of the abs of
  // all rotation angles
  double angle_sum = 0;
  double angle_conf = 0;
  double angle_max = 0;
  for (int i = 0; i < angles.size(); i++)
  {
    angle_sum += fabs(angles[i]);
    angle_conf += angles[i];
    angle_max = fabs(angle_conf) > angle_max ? fabs(angle_conf) : angle_max;
  }

//  ROS_WARN_STREAM("angle_sum: " << angle_sum);
//  ROS_WARN_STREAM("bt0.getAccRotationAngle(): " << bt0->getAccRotationAngle());
//  ROS_WARN_STREAM("angle_max: " << angle_max);
//  ROS_WARN_STREAM("bt0.getMaxRotationAngle(): " << bt0->getMaxRotationAngle());

  EXPECT_NEAR (angle_sum,
               bt0->getAccRotationAngle(), 0.00001);

  // The max rotation angle should be the different from the all rotation
  // we just look
  EXPECT_NEAR (angle_max,
               bt0->getMaxRotationAngle(), 0.00001);

  // The Rotation Axis Parallelism should be almost 1.0
//  ROS_WARN_STREAM("rotAxisParall: " << bt0->getRotationAxisParallelism());
  EXPECT_NEAR (1.0,
               bt0->getRotationAxisParallelism(), 0.0001);

}

/**
 * Perform 4 (different) random degree rotations of a point cloud and verify
 * that each Body Transformation matrix reflects the angle of rotation
 *
 * The angles are going to be + and -, so that max will differ from acc angle
 * then verify the motion parameters
 *
 * The difference to the previous test is that there is an additional
 * translational offset added to all the features
 */
TEST(BodyTrajectoryTest, shiftedRandomRotationWithDifferentAngles)
{

  int num_features = 10;
  int num_frames = 4;

  std::vector<double> angles;

  // motions must be smaller than pi otherwise they are not recognize
  angles.push_back(2.5);
  angles.push_back(-2.7);
  angles.push_back(1.2);

  ClusterTrajectoryPtr cluster0
   = generateCurvedClusterTrajectory(0, num_features, num_frames, angles,
                                     FeaturePtr(new Feature(30.0,-5.,8)));

  // print trajectory
//  printClusterTrajectory(cluster0);

  BodyTrajectoryPtr bt0 = cluster0->estimateBodyTrajectory(0, num_frames-1);

//  ROS_INFO_STREAM (" bt0(1) matrix: \n" <<
//      bt0->getRigidTransformation(1)->getTransformationMatrix().matrix());

  double acc_angle = 0;
  for (int t = 1; t < num_frames; t++){
    ROS_DEBUG_STREAM("angle[" << (t-1) << "]: " << (angles[t-1]));
    ROS_DEBUG_STREAM("mtx: \n" <<
       bt0->getRigidTransformation(t)->getTransformationMatrix().matrix()//.topLeftCorner(3,3)
    );

    double angle = 0;

    // Eigen always gives back a value between [0, 2 pi]
    // and switches the direction of the rotation axis instead
    acc_angle += angles[t-1];
    if (acc_angle < 0)
      angle = 2*M_PI + acc_angle;
    else
      angle = acc_angle;

    // check
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[0],
        cos(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(0)[1],
        sin(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[0],
        -sin(angle),
        0.00001);
    EXPECT_NEAR(
        bt0->getRigidTransformation(t)->getTransformationMatrix().matrix().col(1)[1],
        cos(angle),
        0.00001);
  }

  // The accumulated rotation angle should be the sum of the abs of
  // all rotation angles
  double angle_sum = 0;
  double angle_conf = 0;
  double angle_max = 0;
  for (int i = 0; i < angles.size(); i++)
  {
    angle_sum += fabs(angles[i]);
    angle_conf += angles[i];
    angle_max = fabs(angle_conf) > angle_max ? fabs(angle_conf) : angle_max;
  }

//  ROS_WARN_STREAM("angle_sum: " << angle_sum);
//  ROS_WARN_STREAM("bt0.getAccRotationAngle(): " << bt0->getAccRotationAngle());
//  ROS_WARN_STREAM("angle_max: " << angle_max);
//  ROS_WARN_STREAM("bt0.getMaxRotationAngle(): " << bt0->getMaxRotationAngle());

  EXPECT_NEAR (angle_sum,
               bt0->getAccRotationAngle(), 0.00001);

  // The max rotation angle should be the different from the all rotation
  // we just look
  EXPECT_NEAR (angle_max,
               bt0->getMaxRotationAngle(), 0.00001);

  // The Rotation Axis Parallelism should be almost 1.0
  ROS_WARN_STREAM("rotAxisParall: " << bt0->getRotationAxisParallelism());
  EXPECT_NEAR (1.0,
               bt0->getRotationAxisParallelism(), 0.0001);


  ROS_WARN_STREAM("transAxisParall: " << bt0->getTranslationAxisParallelism());
  // the translation axis parallelism should be low
  EXPECT_TRUE (0.9 > bt0->getTranslationAxisParallelism());


}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
