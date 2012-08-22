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
#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include "trackerListener.h"

#include <vector>

namespace vision{
/**
 *\class    Accumulator
 *\brief    Implements TrackerListener. Accumilates a history of features.
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:44 $
 *$Revision: 1.1 $
 */
 class Accumulator : public TrackerListener{
  public:
    /**
     * Default constructor
     */
    Accumulator();

    /**
     * Default destructor
     */
    ~Accumulator();

    /**
     * Returns accumilated good features
     *
     * @return - vector of vectors of features that have not yet been lost
     */
    std::vector< std::vector< FeaturePtr > > getGoodFeatureHistory();

    /**
     * Returns all accumilated features.
     *
     * @return - vector of all vectors of features since begining of tracking. 
     */
    std::vector< std::vector< FeaturePtr > > getAllFeatureHistory();

    /**
     * clones all current features and adds the vector to the history of
     * features
     *
     * @param img - The image on wich the tracker performed its tracking
     * @param features - The set of features on the new image
     */
    virtual void notifyStep(const Image &img,  const std::vector<FeaturePtr> &features);

    /**
     * Deletes the old history of features and initilizes the new history
     * vector with the new features
     * 
     * @param features - The new set of detected features
     */
    virtual void notifyReset(const std::vector<vision::FeaturePtr> &features);

	/**
     * Returns the tracked features only in the last step
     * 
     * @return - vector of features tracked in the last step 
     */
	std::vector<FeaturePtr> getLastTrackedFeatures();

	/**
     * Returns only the good tracked features in the last step
     * 
     * @return - vector of features tracked in the last step 
     */
	std::vector<FeaturePtr> getLastGoodTrackedFeatures();

  private:
    /**
     * each vector in the history vector is a clone of all features from each
     * thack step from creation or last call to notifyReset
     */
    std::vector< std::vector< vision::FeaturePtr > > featureHistory;
 };
};
#endif
