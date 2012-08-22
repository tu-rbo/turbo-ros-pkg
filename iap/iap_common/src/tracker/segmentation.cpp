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
 *$RCSfile: segmentation.cpp,v $
 *$Author: dubik $
 *$Date: 2009/03/10 15:01:06 $
 *$Revision: 1.1 $
 */

#include "segmentation.h"
#include "cv.h"

Segmentation::Segmentation(const IplImage* rawImg, bool blur) {
	img = cvCloneImage(rawImg);
	if (blur) {
		cvSmooth(img, img, CV_BLUR,15,0,0,0);
	}	
}

Segmentation::Segmentation(const Segmentation &copy) {
	img = cvCloneImage(copy.img);
}

// Segmentation Segmentation::operator = (const Segmentation &assignee) {
// 	img = cvCloneImage(assignee.img);
// }

Segmentation::~Segmentation(void) {
	cvReleaseImage(&img);
}
