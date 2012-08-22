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
#ifndef DISPLAY_TRACK_H
#define DISPLAY_TRACK_H



#include "trackerListener.h"
#include "highgui.h"
#include <vector>
namespace vision{
/**
 *\class    DisplayTrack
 *\brief    Implements TrackerListener. Displays the results of tracking
 *$Author: dubik $
 *$Date: 2009/03/11 17:53:17 $
 *$Revision: 1.2 $
 */
 class DisplayTrack : public TrackerListener{

  public:
    /**
     * creates a window for displaying the video and tracked features
     */
    DisplayTrack();

	/**
     * creates a named window for displaying the video and tracked features
     */
    DisplayTrack(const char* windowName);

    DisplayTrack(std::string saveToFilename, int height, int width, double fps = 15);

    /**
     * distroys the window 
     */
    ~DisplayTrack();

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

    void setDisplayImageVar(bool value);

    void addCircle(const Feature *f);
	void addCircles(const std::vector<Feature> features, int thickness=2);
    void addEdges(const std::vector<Feature> &features, bool **connectivity);
	void clear();

  private:
    /** keeps track of weather it has already notified of lost feature*/
    std::vector<bool> lost; 
    Image* _lastImg;
    CvVideoWriter* _writer;
    bool _displayImage;
	const char* _windowName;
 };
};
#endif
