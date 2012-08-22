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
#ifndef CLOSE_UP_DISPLAY_H
#define CLOSE_UP_DISPLAY_H

#include "trackerListener.h"

#include <vector>
namespace vision{
/**
 *\class    CloseUpDisplay
 *\brief    Implements TrackerListener. Displays the results of tracking
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:45 $
 *$Revision: 1.1 $
 */
 class CloseUpDisplay : public TrackerListener{

  public:
    /**
     * creates a window for displaying the video and tracked features
     */
    CloseUpDisplay(int window = 2, int groupId = 1);

    /**
     * distroys the window 
     */
    ~CloseUpDisplay();

    /**
     * displays the current frame and features, if the features are lost a
     * message regarding the lost feature is printed to standard out the first
     * time it gets lost
     * 
     * @param img - The image that gets displayed
     * @param features - the set of features that get drwan on the image
     */
    virtual void notifyStep(const Image &img,  const std::vector<FeaturePtr> &features);

    /**
     * resets the memory of lost features
     * 
     * @param features - The new set of detected features
     */
    virtual void notifyReset(const std::vector<FeaturePtr> &features);

  private:
    Image* _lastImg;
    int _window;
    int _groupId;
 };
};
#endif
