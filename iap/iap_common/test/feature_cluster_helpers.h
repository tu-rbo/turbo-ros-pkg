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
#include "FeatureTrajectorySet.h"
#include "FeatureSet.h"
#include "ClusterTrajectory.h"
#include "Cluster.h"


double uniform(double max, double min) {
  static bool seeded = false;

  if(!seeded) {
    srand ( time(NULL) );
    seeded = true;
  }

  return (max - min) * ((double) rand()) / RAND_MAX + min;
}

void printClusterTrajectory(vision::ClusterTrajectoryPtr cluster0) {
  for (int t = 0; t < cluster0->getTrajectoryLength(); t++) {
    vision::ClusterPtr c1 = cluster0->getCluster(t);
    vision::Cluster::iterator it1 = c1->begin();
    int i = 0;
    for (; it1 != c1->end(); it1++)
    {
      ROS_INFO_STREAM( (i++) << ": " << (*it1));
    }
  }
}

vision::FeaturePtr generateUniformRandomFeature(double x_min, double x_max,
                                        double y_min, double y_max,
                                        double z_min, double z_max)
{
  vision::FeaturePtr f(new vision::Feature(uniform(x_max, x_min),
                            uniform(y_max, y_min),
                            uniform(z_max, z_min)));
  return f;
}

vision::FeaturePtr generateUniformRandomFeature(double _min, double _max)
{
  return generateUniformRandomFeature(_min, _max, _min, _max, _min, _max);
}


vision::ClusterTrajectoryPtr generateStraightClusterTrajectory(int id, int num_features,
                                               int num_frames) {
  // Create cluster at time 0
  vision::ClusterPtr cluster_0 (new vision::Cluster(0, id));

  for (int i = 0; i < num_features; i++)
  {
    // random start
    vision::FeaturePtr f = generateUniformRandomFeature(4.0, 10.0);
    f->setId(i);
    cluster_0->addFeature(f);
  }

  vision::FeaturePtr move_by = generateUniformRandomFeature(0.5, 20.0);
//  ROS_INFO_STREAM("move by: " << *move_by );

  vision::ClusterTrajectoryPtr cluster_0_traj(new vision::ClusterTrajectory);
  cluster_0_traj->setClusterId(id);
  cluster_0_traj->addCluster(cluster_0);

  // generate trajectory
  vision::ClusterPtr last_cluster = cluster_0;
  for (int t = 1; t < num_frames; t++) {
    vision::ClusterPtr cluster_0_t (new vision::Cluster(t, id));
    // move all the features
    int i = 0;
    for (vision::Cluster::iterator it = last_cluster->begin(); it != last_cluster->end(); it++)
    {
      vision::FeaturePtr f = move_by + (*it);
      f->setId(i++);
      cluster_0_t->addFeature(f);
    }
    cluster_0_traj->addCluster(cluster_0_t);
    last_cluster = cluster_0_t;
  }

  return cluster_0_traj;
}

vision::ClusterTrajectoryPtr generateCurvedClusterTrajectory(int id, int num_features,
                                                     int num_frames,
                                                     std::vector<double> angles,
                                                     vision::FeaturePtr translation=vision::FeaturePtr(new vision::Feature(0,0,0)))
{
  // Create cluster at time 0
  vision::ClusterPtr cluster_0 (new vision::Cluster(0, id));

  for (int i = 0; i < num_features; i++)
  {
    // random start
    vision::FeaturePtr f = generateUniformRandomFeature(4.0, 10.0);
//    vision::FeaturePtr f (new vision::Feature(1,0,0)); // FIXME
    f->setId(i);
    cluster_0->addFeature(f);
  }

  vision::ClusterTrajectoryPtr cluster_0_traj(new vision::ClusterTrajectory);
  cluster_0_traj->setClusterId(id);
  cluster_0_traj->addCluster(cluster_0);

  // generate trajectory
  vision::ClusterPtr last_cluster = cluster_0;
  for (int t = 1; t < num_frames; t++) {
    vision::ClusterPtr cluster_0_t (new vision::Cluster(t, id));
    // move all the features
    int i = 0;
    for (vision::Cluster::iterator it = last_cluster->begin(); it != last_cluster->end(); it++)
    {
      vision::FeaturePtr prev = (*it);
      // rotate feature
      vision::FeaturePtr f(new vision::Feature(
          prev->getX() * cos(angles[t-1]) - prev->getY() * sin(angles[t-1]),
          prev->getX() * sin(angles[t-1]) + prev->getY() * cos(angles[t-1]),
          prev->getZ()));
      f->setId(i++);
      cluster_0_t->addFeature(f+translation);
    }
    cluster_0_traj->addCluster(cluster_0_t);
    last_cluster = cluster_0_t;
  }

  return cluster_0_traj;
}

vision::ClusterTrajectoryPtr generateCurvedClusterTrajectory(int id, int num_features,
                                                     int num_frames,
                                                     double angle,
                                                     vision::FeaturePtr translation=vision::FeaturePtr(new vision::Feature(0,0,0)))
{
  std::vector<double> angles;
  for (int i = 0; i < num_frames-1; i++)
    angles.push_back(angle);

  return generateCurvedClusterTrajectory(id,
                                         num_features,
                                         num_frames,
                                         angles,
                                         translation);
}

/*****************************************************************/

vision::ClusterTrajectoryPtr generateRandomClusterTrajectory(int id, int num_features,
                                               int num_frames,
                                               double max_rand = 50.0,
                                               double min_rand = 10.0,
                                               double max_move_rand = 10.0,
                                               double min_move_rand = 5.0)
{

  // Create cluster at time 0
  vision::ClusterPtr cluster_0 (new vision::Cluster(0, id));

  for (int i = 0; i < num_features; i++)
  {
    vision::FeaturePtr f(new vision::Feature(uniform(max_rand, min_rand),
                              uniform(max_rand, min_rand),
                              uniform(max_rand, min_rand)));
    f->setId(i);
    cluster_0->addFeature(f);
  }

  vision::ClusterTrajectoryPtr cluster_0_traj(new vision::ClusterTrajectory);
  cluster_0_traj->setClusterId(id);
  cluster_0_traj->addCluster(cluster_0);

  // generate trajectory
  vision::ClusterPtr last_cluster = cluster_0;
  for (int t = 1; t < num_frames; t++) {
    vision::ClusterPtr cluster_0_t (new vision::Cluster(t, id));
    // move all the features
    vision::FeaturePtr move_by(new vision::Feature(uniform(max_move_rand, min_move_rand),
                                   uniform(max_move_rand, min_move_rand),
                                   uniform(max_move_rand, min_move_rand)));
//    ROS_INFO_STREAM("move by: " << *move_by );
    int i = 0;
    for (vision::Cluster::iterator it = last_cluster->begin(); it != last_cluster->end(); it++)
    {
      vision::FeaturePtr f = move_by + (*it);
      f->setId(i++);
      cluster_0_t->addFeature(f);
    }
    cluster_0_traj->addCluster(cluster_0_t);
    last_cluster = cluster_0_t;
  }

  return cluster_0_traj;
}

/**
 * Merge the cluster trajectories by adding the Feature values
 * (same amount of features in the end!)
 */
vision::ClusterTrajectoryPtr mergeClusterTrajectories(vision::ClusterTrajectoryPtr ctraj1,
                                              vision::ClusterTrajectoryPtr ctraj2,
                                              int id)
{
  vision::ClusterTrajectoryPtr cm_traj(new vision::ClusterTrajectory);
  cm_traj->setClusterId(id);

  EXPECT_TRUE (ctraj1->getTrajectoryLength() == ctraj2->getTrajectoryLength());
  int num_frames = ctraj1->getTrajectoryLength();

  for (int t = 0; t < num_frames; t++) {
    vision::ClusterPtr cm (new vision::Cluster(t, id));
    // move all the features
    vision::ClusterPtr c1 = ctraj1->getCluster(t);
    vision::ClusterPtr c2 = ctraj2->getCluster(t);

    vision::Cluster::iterator it1 = c1->begin(),
        it2 = c2->begin();
    int i = 0;
    for (; it1 != c1->end() && it2 != c2->end(); it1++, it2++)
    {
      vision::FeaturePtr f = (*it1) + (*it2);
      f->setId(i++);
      cm->addFeature(f);
    }
    cm_traj->addCluster(cm);
  }

  return cm_traj;
}

/**
 * Merge the cluster trajectories by creating a trajectory with all the
 * features separately
 * (double amount of features in the end!)
 */
vision::ClusterTrajectoryPtr mergeClusterTrajectoriesSeparately(
    vision::ClusterTrajectoryPtr ctraj1,
    vision::ClusterTrajectoryPtr ctraj2,
    int id)
{

  vision::ClusterTrajectoryPtr cm_traj(new vision::ClusterTrajectory);
  cm_traj->setClusterId(id);

  EXPECT_TRUE (ctraj1->getTrajectoryLength() == ctraj2->getTrajectoryLength());
  int num_frames = ctraj1->getTrajectoryLength();

  for (int t = 0; t < num_frames; t++) {
    vision::ClusterPtr cm (new vision::Cluster(t, id));
    // move all the features
    vision::ClusterPtr c1 = ctraj1->getCluster(t);
    vision::ClusterPtr c2 = ctraj2->getCluster(t);

    int i = 0; // feature id counter

    vision::Cluster::iterator it1 = c1->begin();
    for (; it1 != c1->end(); it1++)
    {
      vision::FeaturePtr f = (*it1)->clone();
      f->setId(i++);
      cm->addFeature(f);
    }
    vision::Cluster::iterator it2 = c2->begin();
    for (; it2 != c2->end(); it2++)
    {
      vision::FeaturePtr f = (*it2)->clone();
      f->setId(i++);
      cm->addFeature(f);
    }
    cm_traj->addCluster(cm);
  }

  return cm_traj;

}
