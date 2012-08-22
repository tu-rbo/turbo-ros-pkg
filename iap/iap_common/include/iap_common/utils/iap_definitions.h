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
 * iap_definitions.h
 *
 *  Created on: Dec 14, 2011
 *      Author: roberto
 */

#ifndef IAP_DEFINITIONS_H_
#define IAP_DEFINITIONS_H_

//Vector of Features
typedef std::vector<Feature> FeatureSet;

//The same Feature_Set in different time steps ->
//-> Feature_Trajectory_Set.at(i).size() == Feature_Trajectory_Set.at(j).size() = Number_features
typedef std::vector<FeatureSet> FeatureTrajectorySet;

//Every Feature_Set is a cluster
// -> Feature_Trajectory_Set.at(i).size() = Cluster_i_Size
// (in general != Feature_Trajectory_Set.at(j).size() = Cluster_j_Size)
typedef std::vector<FeatureSet> ClusterSet;

//The same Cluster_Set in different time steps ->
// -> Cluster_Trajectory_Set.at(i).size() == Cluster_Trajectory_Set.at(j).size() = Number_clusters
typedef std::vector<ClusterSet> ClusterTrajectorySet;


#endif /* IAP_DEFINITIONS_H_ */
