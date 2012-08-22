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
/**
 * Virtual Class for different types of segmentation.
 *
 * This is the superclass for all types of segmentation.
 *
 *
 *$RCSfile: segmentation.h,v $
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:52 $
 *$Revision: 1.1 $
 */

#ifndef __SEGMENTATION_h__
#define __SEGMENTATION_h__

#include "cv.h"

class Segmentation {
 private:

 protected:
  IplImage* img;

 public:

  /**
   * This constructor takes an image and creates a segmentation object
	 * Setting blur to true will cause the segmentation algorithm to blur the 
	 * image before it performs the segmentation.
   */
  Segmentation(const IplImage* rawImg, bool blur=false);

	/**
	 * Copy constructor
	 */
	Segmentation(const Segmentation &copy);

	/**
	 * Assignment operator
	 */
	// TODO: Do we need an assignment operator here?  I don't think so since Segmentation is a pure virtual class
	//Segmentation operator = (const Segmentation &assignee);

	/**
	 *
	 */
	virtual ~Segmentation(void);
	
  /**
   * Segment the image
   */
  virtual IplImage* segment(void) = 0;
  
};

#endif
