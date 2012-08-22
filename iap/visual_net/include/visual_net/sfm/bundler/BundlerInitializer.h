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
 * BundlerInitalizer.h
 *
 *  Created on: 06.08.2010
 *      Author: dermax
 */
#define SUBSAMPLE_LEVEL 1

#define MIN_MATCHES 10

#define __BUNDLER__

#include "IOUtils.h"
#include "BundlerApp.h"
#include "Estimator.h"
#include "Initializer.h"

namespace SFM
{
/**
 * @brief SFM bundle adjustment with code from "bundler"
 */
class BundlerInitializer : public Initializer, BundlerApp
{
public:
  /**
   * Constructor
   * @param camera - The model of the camera used to record the feature tracks (parameters needed: resolution and focal length)
   */
  BundlerInitializer(vision::CameraModel &camera, double min_motion_pix);

  /**
   * Required for BundlerApp class
   */
  bool OnInit();

  /**
   * Required for BundlerApp class
   */
  void ProcessOptions(int argc, char **argv);

  /**
   * Run the initialization procedure of Bundler
   * @param features - xy in pixel values (2D features). The coordinates must be referred to the center of the image
   * @return - xyz positions (3D features) of the initial structure (features) with some scale factor
   */
  std::vector<vision::FeaturePtr> run(std::vector<std::vector<vision::FeaturePtr> > &features);

  /**
   * Set the frames to be used to run Bundler Adjustment
   * @param frames - A vector of frame index numbers for the frames to use for BA
   */
  void setBundlerFrames(std::vector<int>& frames);

  /**
   * The BundlerInitializer contains a set of settings to test. If the initializer has to be run with different settings
   * we could choose the set that provides best results. Pure virtual, it must be implemented in derived classes
   * @param num - The set of settings to test in the next run
   */
  void setSetting(int num);

private:

  bool use_last_frame; // Use the last feature in addition to the frames selected based on the motion threshold
  std::vector<int> bundler_frame_index; // Variable to store the frames from setBundlerFrames()

  /**
   * Estimate the motion between frames and select the set of frames to be used for BA
   * It return the Features only in the selected frames
   * @param features - All the Feature Trajectories in 2D
   * @return - Only the Feature Trajectories in the frames that will be used for BA
   */
  std::vector<std::vector<vision::FeaturePtr> >
                                                selectBundlerFrames(
                                                                    std::vector<std::vector<vision::FeaturePtr> > features);

  /**
   * Find the median distance between two features as the addition of their distances in every frame, divided by the number of frames
   * @param f1 - Trajectory as vector of FeaturePtr (position in every frame) of the first feature
   * @param f2 - Trajectory as vector of FeaturePtr (position in every frame) of the second feature
   * @return - Double value of the median distance between both feature trajectories
   */
  double absDif(std::vector<vision::FeaturePtr> & f1, std::vector<vision::FeaturePtr> & f2)
  {
    double dif = 0.0;
    assert(f1.size() == f2.size());
    for (unsigned int i = 0; i < f1.size(); i++)
    {
      dif += sqrt(pow(f1[i]->getX() - f2[i]->getX(), 2) + pow(f1[i]->getY() - f2[i]->getY(), 2));
    }
    return dif / (double)f1.size();
  }
};

}
