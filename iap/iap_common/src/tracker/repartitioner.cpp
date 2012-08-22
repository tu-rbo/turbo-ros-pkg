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
#include "repartitioner.h"
#include "cv.h"
#include "highgui.h"
#include "feature.h"

using namespace vision;

Repartitioner::Repartitioner()
{

}

Repartitioner::~Repartitioner()
{

}

void Repartitioner::notifyStep(const Image &img,  const std::vector<FeaturePtr>
&features)
{

}


void Repartitioner::notifyReset(const std::vector<FeaturePtr> &features)
{
	
}


void Repartitioner::setPartitions(std::vector< std::vector< FeaturePtr > > part)
{
	partitions = part;
	//constructHulls(part);
	displayParts(part);
}

std::vector< std::vector< FeaturePtr > > Repartitioner::getNewPartitions()
{
	std::vector< std::vector< FeaturePtr > > b;

	return b;
}

void Repartitioner::displayParts(const std::vector< std::vector< FeaturePtr > > &part)
{
	int blue = 0;
	int red = 255;
	IplImage* img = cvCreateImage( cvSize( 1000, 1000 ), 8, 3 );
	cvZero( img );
	cvNamedWindow( "hull", 1 );

	std::vector< std::vector< FeaturePtr > >::const_iterator partitionIt = part.begin();
	while(partitionIt != part.end())
	{	
		std::vector< FeaturePtr > set = (std::vector< FeaturePtr >) *(partitionIt++);
		std::vector< FeaturePtr >::const_iterator setIt = set.begin();
		while(setIt != set.end())
		{
			FeaturePtr ft = *(setIt++);
			CvPoint pt = cvPointFrom32f( cvPoint2D32f( ft->getX(), ft->getY() ) );
			cvCircle( img, pt, 2, CV_RGB( red, 0, blue ), CV_FILLED, CV_AA, 0 );
		}
		blue+=100;
		red -= 100;
	}
	constructHulls(part, img);
	cvShowImage( "hull", img );
	cvWaitKey();
	cvDestroyWindow( "hull" );
	cvReleaseImage(&img);
}



//private
void Repartitioner::constructHulls(const std::vector< std::vector< FeaturePtr > > &part, IplImage* img)
{
	int count, hullCount;
	CvPoint2D32f *points, *outTemp;
	int *hull;
	CvPoint2D32f pt0;

	//To prevent memory leakage check if hullVector had been previously initialized, if so
	//we have to release all the CvMats and clear the vector of the cleared data.
	if(!hullVector.empty())
	{
		std::vector< CvMat >::iterator hullIt = hullVector.begin();
		while(hullIt != hullVector.end())
		{
			CvMat pts = *(hullIt++);
			//cvReleaseMat( &pts );
		}
		hullVector.clear();
	}

	
	std::vector< std::vector< FeaturePtr > >::const_iterator partitionIt = part.begin();
	while(partitionIt != part.end())
	{	
		std::vector< FeaturePtr > set = (std::vector< FeaturePtr >) *(partitionIt++);
		std::vector< FeaturePtr >::const_iterator setIt = set.begin();

		count = set.size();
		points = new CvPoint2D32f[count];
		CvMat pointMat = cvMat( 1, count, CV_32FC2, points );
		hull = new int[count];
        	CvMat hullMat = cvMat( 1, count, CV_32SC1, hull );

		int i = 0;
		while(setIt != set.end())
		{
			FeaturePtr ft = *(setIt++);
			pt0.x = ft->getX();
			pt0.y = ft->getY();
			points[i++] = pt0;
		}
		cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
		hullCount = hullMat.cols;
		outTemp = new CvPoint2D32f[hullCount];
	

		for(i = 0; i < hullCount; i++ )
		{
			CvPoint2D32f pt = points[hull[i]];
			outTemp[i] = pt;
		}
		CvMat outMat = cvMat( 1, hullCount, CV_32FC2, outTemp );
		hullVector.push_back(outMat);
/////////////////////////////////////////////////////////////////////////////
////debug display//////////
/////////////////////////

		CvPoint pt0 = cvPointFrom32f( points[hull[hullCount-1]] );
		for( i = 0; i < hullCount; i++ )
		{
			CvPoint pt = cvPointFrom32f ( points[hull[i]] );
			cvLine( img, pt0, pt, CV_RGB( 0, 255, 0 ));
			pt0 = pt;
		}
		


		delete[] points;
		delete[] hull;
	
	}
}
