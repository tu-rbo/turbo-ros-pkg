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
#include "cv.h"
#include "highgui.h"
#include "feature.h"
#include <stdio.h>
#include <vector>
#include <iostream>

#ifndef _Triangulate_H_
#define _Triangulate_H_

class Triangulate{
    std::vector<CvPoint2D32f> points;
    bool **triangulation;
    CvSubdiv2D* subdiv;
    CvMemStorage* storage;
    CvRect rect;
  public:
    Triangulate(std::vector<vision::FeaturePtr> features, CvRect _rect=cvRect(0,0,4*800,4*800));
    ~Triangulate();
	bool** getTriangulation();
	bool** getIntersectingEdges(std::vector<vision::FeaturePtr> newPoints);	//indicates which edges should be erased to remove intersections
																			//(greedy, removes edges that create more intersections first)
  private:
    bool isInVector(const CvPoint2D32f &pt);
    CvSubdiv2D* init_delaunay( CvMemStorage* storage);
    void draw_subdiv_edge(CvSubdiv2DEdge edge);
    void draw_subdiv(CvSubdiv2D* subdiv);
    void markMatrix(CvPoint pt1, CvPoint pt2);
	bool isIntersecting(std::pair<double,double> A1, std::pair<double,double> A2, std::pair<double,double> B1, std::pair<double,double> B2);	//checks for the intersection of two
																																				//line segments (A1,A2) and (B1,B2)
	
};

#endif
