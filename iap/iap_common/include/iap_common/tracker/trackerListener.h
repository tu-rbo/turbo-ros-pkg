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
#ifndef TRACKER_LISTENER_H
#define TRACKER_LISTENER_H

#include <vector>
#include <boost/shared_ptr.hpp>

namespace vision {

class Feature;
typedef boost::shared_ptr<Feature> FeaturePtr;
class Image;
/**
 *\class    TrackerListener
 *\brief    Interface for subscribing to listen to updates from a tracker 
 * object.
 *$Author: dubik $
 *$Date: 2009/03/11 17:53:18 $
 *$Revision: 1.2 $
 */
class TrackerListener {

public:
    TrackerListener() {
    }
    virtual ~TrackerListener() {
    }

    /**
     * Gets called after tracker executes a call to step
     * 
     * @param img - The image on wich the tracker performed its tracking
     * @param features - The set of features on the new image
     */
    virtual void notifyStep(const Image &img,
            const std::vector<FeaturePtr> &features) = 0;

    /**
     * Gets called after tracker executes a call to reset
     * 
     * @param features - The new set of detected features
     */
    virtual void notifyReset(const std::vector<FeaturePtr> &features) = 0;
};
}
;
#endif
