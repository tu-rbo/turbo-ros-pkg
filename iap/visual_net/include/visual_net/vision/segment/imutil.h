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
/* some image utilities */

#ifndef IMUTIL_H
#define IMUTIL_H

#include "segimage.h"
#include "misc.h"

namespace segmentation{

/* compute minimum and maximum value in an image */
template <class T>
void min_max(segimage<T> *im, T *ret_min, T *ret_max) {
  int width = im->width();
  int height = im->height();
  
  T min = imRef(im, 0, 0);
  T max = imRef(im, 0, 0);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      T val = imRef(im, x, y);
      if (min > val)
	min = val;
      if (max < val)
	max = val;
    }
  }

  *ret_min = min;
  *ret_max = max;
} 

/* threshold image */
template <class T>
segimage<uchar> *threshold(segimage<T> *src, int t) {
  int width = src->width();
  int height = src->height();
  segimage<uchar> *dst = new segimage<uchar>(width, height);
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(dst, x, y) = (imRef(src, x, y) >= t);
    }
  }

  return dst;
}

};
#endif

