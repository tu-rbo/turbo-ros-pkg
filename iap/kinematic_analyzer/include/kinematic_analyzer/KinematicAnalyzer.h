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
#ifndef KINEMATIC_ANALYZER_H_
#define KINEMATIC_ANALYZER_H_

#include "feature.h"
#include "FeatureSet.h"
#include "FeatureTrajectorySet.h"
#include "ClusterTrajectorySet.h"
#include "JointEstimator.h"

#include "iap_common/KinematicStructureMsg.h"

class KinematicStructure : public std::map<std::pair<int, int>, JointEstimatorResultPtr>
{
public:
  iap_common::KinematicStructureMsg toROSMsg();
};

typedef enum IntervalSelection
{
  ONA = 0, NOA = 1, OA = 2
} IntervalSelection;

class KinematicAnalyzer
{
public:

  /**
   * Constructor
   */
  KinematicAnalyzer();

  /**
   * Destructor
   */
  virtual ~KinematicAnalyzer();

  /**
   * Set the last image received from the sensor where we will print the resulting axes
   * @param last_image - The last received RGB image
   */
  void setLastRGBImage(cv::Mat last_image);

  /**
   * Add a new FeatureSet (from the last frame) to the accumulated FeatureTrajectorySet
   * @param feature_set - Set of features to be add
   */
  virtual void addFeatureSet3D(vision::FeatureSetPtr feature_set);

  /**
   * Set the vector containing the segmentation of the feature sets into clusters. It merges the past and the new
   * segmentation to assure that the "children" clusters are parts of only one "parent" cluster.
   * @param new_segmentation - New segmentation. One integer per feature with the ClusterId for this Feature
   */
  virtual void setSegmentation(std::vector<int> &new_segmentation);

  /**
   * Launch the process to estimate the kinematic structure based on received 3D feature sets and segmentation
   */
  virtual void estimateKinematicStructure();

  /**
   * Draw the estimated axes on the last received RGB frame
   */
  virtual void drawAxes();

  /**
   * Set the joint estimator to be used when computing the joints between rigid bodies
   * @param je - Smart pointer to the joint estimator object to be used
   */
  virtual void setJointEstimator(JointEstimatorPtr je);

  /**
   * Get the last result of the kinematic analysis for the specified pair of clusters
   * @param cluster_id1 - id of the first cluster
   * @param cluster_id2 - id of the second cluster
   * @return - Smart pointer to the result of the analysis
   */
  virtual JointEstimatorResultPtr getJointEstimatorResult(int cluster_id1, int cluster_id2) const;

  /**
   * Get the last result of the kinematic analysis over all pair of clusters
   * @return - Map with the results for every pair of clusters
   */
  virtual KinematicStructure getKinematicStructure() const;

  /**
   * Reset the KinematicAnalyzer object
   */
  virtual void reset();

  /**
   * Set the criterion to select intervals for the kinematic analysis
   * @param interval_selection - type of criterion to be used
   */
  virtual void setIntervalSelectionCriterion(IntervalSelection interval_selection);

  /**
   * If interval_selection is set to one of the overlap settings,
   * you can set the actual overlap here
   * @param interval_selection - type of criterion to be used
   */
  virtual void setIntervalSelectionOverlap(int overlap);

protected:

  JointEstimatorPtr _joint_estimator;

  KinematicStructure _kinematic_structure;

  vision::FeatureTrajectorySetPtr _accumulator_3d;
  std::vector<int> _segmentation;

  IntervalSelection _interval_selection;
  int _interval_selection_overlap;

  cv::Mat _last_image;
  cv::Mat _last_image_cpy;

  int _frame_counter;
};

#endif /* KINEMATIC_ANALYZER_H_ */
