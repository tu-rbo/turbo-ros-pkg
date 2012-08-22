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
/** ***************************************************************************
 * Tracker Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:09 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "tracker.h"
#include "trackerListener.h"

using namespace vision;

Tracker::Tracker()
{
    ready = false;
    curImage = 0;
}
// ============================================================================

Tracker::~Tracker()
{}
// ============================================================================

void Tracker::addListener(TrackerListener* listener)
{
    listeners.push_back(listener);
}
// ============================================================================

void Tracker::notifyStepListeners()
{
    std::vector<TrackerListener*>::iterator listenersIt;

    for(listenersIt = listeners.begin();
        listenersIt != listeners.end();listenersIt++)
    {
        TrackerListener* listener = *(listenersIt);
        listener->notifyStep(*curImage, features);
    }
}
// ============================================================================

void Tracker::notifyResetListeners()
{
    std::vector<TrackerListener*>::iterator listenersIt;

    for(listenersIt = listeners.begin();
        listenersIt != listeners.end(); listenersIt++)
    {
        TrackerListener* listener = *(listenersIt);
        listener->notifyReset(features);
    }
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
