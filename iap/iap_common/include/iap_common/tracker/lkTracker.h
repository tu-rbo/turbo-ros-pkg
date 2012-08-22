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
#ifndef LK_TRACKER_H
#define LK_TRACKER_H

#include "cv.h"
#include "tracker.h"
#include "surfFeature.h"

namespace vision{
	/**
	*\class    LKTracker
	*\brief    Concrete Tracker. Lukcas-Kanade point feature tracker.
	* The opencv implimentation of point feature tracking
	*$Author: dubik $
	*$Date: 2009/03/10 15:00:50 $
	*$Revision: 1.1 $
	*/
	class LKTracker : public Tracker{
	public:
		/**
		* Initializes the data members
		*/
		LKTracker(int winSize = 10, int pyrLevels = 3);

		/**
		* releases the alocated memory used for tracking
		*/
		~LKTracker();

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
		* obtains a new set of features for tracking from a vector of features
		* 
		* @param detectedFeatures - the detected features to be tracked
		*/
		void resetFeatures(std::vector<FeaturePtr> detectedFeatures);

		/**
		* obtains a new set of features for tracking from a vector of SURF features
		* 
		* @param detectedFeatures - the detected features to be tracked
		*/
		void resetFeatures(std::vector<SURFFeaturePtr> detectedFeatures);

	private:
		int _count; /**< number of features */
		CvSize _imgSize; /**< the size of the image (assumed consistent) */
		int _winSize; /**< the size of the search window for searching next frame */
		int _pyrLevels;
		CvPoint2D32f *_points[2], *_swapPoints;
		char* _status;
		float* _error;
		int _flags;
		bool _isFreshImage, _isInit;
		Image* _currentImage;	

		IplImage *_image, *_grey, *_prevGrey, *_pyramid,
			*_prevPyramid, *_swapTemp, *_frame, *_greyTmp;

	};
};
#endif
