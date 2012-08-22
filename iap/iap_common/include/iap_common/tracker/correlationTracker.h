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
#ifndef CORRELATION_TRACKER_H
#define CORRELATION_TRACKER_H


#include "cv.h"
#include "tracker.h"

namespace vision{
/**
 *\class    CorrelationTracker
 *\brief
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:46 $
 *$Revision: 1.1 $
 */
 class CorrelationTracker : public Tracker{
  public:
    /**
     * Initializes the data members
     */
    CorrelationTracker(int winSize = 10);

    /**
     * releases the alocated memory used for tracking
     */
    ~CorrelationTracker();

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
     * This mask describes the region in which to select features for tracking
     */
    void setForegroundMask(const Image* mask);

  private:
    Image *_mask;
    int _winSize;
    int _count; /**< number of features */
    bool _isFreshImage;

 };
};
#endif
