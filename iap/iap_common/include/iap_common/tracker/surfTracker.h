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
#ifndef SURF_TRACKER_H
#define SURF_TRACKER_H

#include <vector>
#include "surfFeature.h"
#include "image.h"
#include "tracker.h"
#include "surfDetector.h"

namespace vision{

	using namespace std;

/**
 *\class    SURFTracker
 *\brief
 *$Author: roberto $
 *$Date: 2010/09/06 12:36:56 $
 *$Revision: 1.0 $
 */
 class SURFTracker : public Tracker{
  public:
	/**
     * Initializes the data members
     */
    SURFTracker();

    /**
     * releases the alocated memory used for tracking
     */
    ~SURFTracker();

    /**
     * @return number of non-lost features being tracked
     */
    int getCount();

    /**
     * retrieves the current image from the VideoSensor
     * 
     * @param sens - must be of type VideoSensor, the Sensor class that
     * this tracker is subscribed to.
     */
    virtual void notify(const Sensor* sens);

    /**
     * tracks features from the previous image to the current one.
     */
    virtual void step();

    /**
     * detects a new set of features for tracking
     * 
     * @param d - the detector used for detecting new features
     */
    virtual void resetFeatures(FeatureDetector* d);

	/**
     * Track SURF features from current image to the given one.
     * 
	 * @param destImage - the image where to track the features to
	 * @return a pair of vectors with the found pairs
     */
	pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> > findSURFPairs(Image* destImage);

	/**
     * Track SURF features from one image to another
     * 
     * @param sourceImage - the image where to track the features from
	 * @param destImage - the image where to track the features to
	 * @param numberFeatures -  max number of features to track
	 * @return a pair of vectors with the found pairs
     */
	pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> > findSURFPairs(Image* sourceImage, Image* destImage, int numberFeatures);

	/**
     * Track the given SURF features in the given image
     * 
     * @param sourceFeatures - vector of SURF features to track
	 * @param destImage - the image where to track the features to
	 * @return a pair of vectors with the found pairs
     */
	pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> > findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, Image* destImage);

	/**
     * Track the given SURF features in the given image
     * 
     * @param sourceFeatures - vector of SURF features to track
	 * @param destImage - the image where to track the features to
	 * @return a pair of vectors with the found pairs
     */
	pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> > findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, const Image* destImage);

	/**
     * Track SURF features pairs between two sets of SURF features
     * 
     * @param sourceFeatures - vector of SURF features to track
	 * @param destFeatures - vector of possible SURF features pairs
	 * @return a pair of vectors with the found pairs
     */
	pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> > findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, std::vector<SURFFeaturePtr> destFeatures);

	/**
     * Return the vector of features converted to be used by a LK tracker
     * 
     */
	std::vector<FeaturePtr> convertForLK();

  private:
    int _count; /**< number of features */
    CvSize _imgSize; /**< the size of the image (assumed consistent) */
    bool _isFreshImage, _isInit;

	SURFDetector *_surfDetector;
 };
};
#endif
