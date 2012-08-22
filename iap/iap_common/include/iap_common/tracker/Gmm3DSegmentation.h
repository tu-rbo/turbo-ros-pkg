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
 * Implements a Gaussian Mixture Model for segmentation.  This implementation
 * performs EM on all three channels, BGR, of the image.  It is a 3D Gaussian Mixture
 * Model.
 *
 * This is the subclass of Segmentation.
 *
 *
 *$RCSfile: Gmm3DSegmentation.h,v $
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:43 $
 *$Revision: 1.1 $
 */

#ifndef __GMM3DSEGMENTATION_h__
#define __GMM3DSEGMENTATION_h__

#include "segmentation.h"
#include "cv.h"
#include "opencv2/legacy/legacy.hpp"

class Gmm3DSegmentation : public Segmentation {
public:
	/**
   * This constructor takes an image and creates a segmentation object
   */
	Gmm3DSegmentation(const IplImage* rawImg, bool blur=false);
	
 	IplImage* segment(void);
private:
	/* data */
};

#endif /* __GMM3DSEGMENTATION_h__ */
