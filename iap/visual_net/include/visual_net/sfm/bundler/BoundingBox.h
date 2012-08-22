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
/* 
 *  Copyright (c) 2008-2010  Noah Snavely (snavely (at) cs.cornell.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* BoundingBox.h */
/* Routines for bounding boxes */

#ifndef __bounding_box_h__
#define __bounding_box_h__

#include <vector>
#include "vector.h"

class BoundingBox {
public:
    BoundingBox() { }
    BoundingBox(double xmin, double ymin, double xmax, double ymax) :
	m_xmin(xmin), m_xmax(xmax), m_ymin(ymin), m_ymax(ymax) { }
    BoundingBox(double x, double y) :
	m_xmin(x), m_xmax(x), m_ymin(y), m_ymax(y) { }

    /* Returns the area of the bounding box */
    double Area();

	BoundingBox Intersect(const BoundingBox &bbox) const;

    /* Returns true iff the box contains the given point */
    bool Contains(double x, double y);
    bool Contains(const BoundingBox &bbox);

    void Scale(double scale);
    void Print();

    double Width()  { return m_xmax - m_xmin; }
    double Height() { return m_ymax - m_ymin; }

    double m_xmin, m_xmax;
    double m_ymin, m_ymax;
};

/* Create a bounding box for a set of points */
BoundingBox CreateBoundingBox(const std::vector<v2_t> &points);

/* Return the union of two bounding boxes */
BoundingBox BoundingBoxUnion(const BoundingBox &bb1, const BoundingBox &bb2);

#endif /* __bounding_box_h__ */
