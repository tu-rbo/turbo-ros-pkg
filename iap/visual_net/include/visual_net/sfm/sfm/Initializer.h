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
 * Initalizer.h
 *
 *  Created on: 28.07.2010
 *      Author: dermax
 */

#ifndef INITALIZER_H_
#define INITALIZER_H_
#include "CameraModel.h"

namespace SFM
{
/**
 * Initializer class
 * Provides a first estimation of the 3D coordinates of the 2D tracked points, that will be later refined by the Estimator
 * Without the results of the Initializer, the Estimator won't converge
 */
class Initializer
{
public:

  /**
   * Constructor
   * @param camera - CameraModel with the required parameters of the camera
   * @param min_motion_pix - Minimum number of pixels that the features should move to trigger the initializer
   */
  Initializer(vision::CameraModel &camera, double min_motion_pix);

  /**
   * Destructor
   */
  virtual ~Initializer();

  /**
   * Run the Initializer. Pure virtual, it must be implemented in derived class.
   * @param features - Features Trajectory Set with the coordinates of the tracked points in each considered frame of the interval
   * @return - Feature Set with the 3D coordinates of the tracked features in the first considered frame of the interval
   */
  virtual std::vector<vision::FeaturePtr> run(std::vector<std::vector<vision::FeaturePtr> > &features) = 0;

  /**
   * The Initializer contains a set of settings to test. If the initializer has to be run with different settings
   * we could choose the set that provides best results. Pure virtual, it must be implemented in derived classes
   * @param num - The set of settings to test in the next run
   */
  virtual void setSetting(int num)=0;

  /**
   * Return the number of different setting sets that the Initializer will test.
   * @return - Number of
   */
  int getNumSettings();

  /**
   * Check if the Features Trajectory Set provided is static, that is, if the Features move less than certain threshold
   * @return - True if the mean motion of all features in all frames is smaller than motion_threshold
   */
  bool isStatic(std::vector<std::vector<vision::FeaturePtr> > &features);

  /**
   * Provides a default complete SfM solution when the features don't move enough to run the SfM algorithm. The 3D coordinates are:
   * x = (u_coord - cam_x_resolution/2) * 1/cam_focal_length
   * y = -(v_coord - cam_y_resolution()/2) * 1/cam_focal_length
   * z = default_depth
   * @param features - The 2D Features in all frames where they were tracked (Feature Trajectory Set)
   * @return - The default complete structure
   */
  std::vector<std::vector<vision::FeaturePtr> >
                                                getStaticStructure(
                                                                   std::vector<std::vector<vision::FeaturePtr> > &features);

  /**
   * Provides a default complete structure as the repetition of the previously found initial structure
   * @param features - The 2D Features in all frames where they were tracked (Feature Trajectory Set)
   * @param previous_init_structure - The previously found 3D initial structure to be replicated
   * @return - Static structure replicating the given one
   */
  std::vector<std::vector<vision::FeaturePtr> >
  continueStaticStructure(std::vector<std::vector<vision::FeaturePtr> > &features,
                          std::vector<vision::FeaturePtr> &previous_init_structure);

  /**
   * Set the motion threshold that marks when the Features moved enough to run the SfM algorithm
   * @param threshold - The new value of the motion threshold
   */
  void setMotionThreshold(double threshold);

  /**
   * Set the default value used for the z coordinate when the Features don't move enough to run the SfM algorithm
   * @param depth - The new value of the default depth
   */
  void setDefaultDepth(double depth);

  /**
   * Get the default value used for the z coordinate when the Features don't move enough to run the SfM algorithm
   * @return - The value of the default depth
   */
  double getDefaultDepth();

  vision::CameraModel cam;              // Model of the camera containing all required parameters
protected:
  double motion_threshold;              // Motion threshold that marks when we can run or not the SfM algorithm
  double default_depth;                 // Default depth value to be assigned when the Features don't move enough to run the algorithm
  int num_settings;                     // Number of setting sets to be tested when running the algorithm
};

}

#endif /* INITALIZER_H_ */
