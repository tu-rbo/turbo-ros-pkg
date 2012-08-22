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
 * ManualDetector Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:05 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "manualDetector.h"
#include "feature.h"
#include "image.h"
#include "cv.h"
#include <vector>
#include <iostream>

using namespace vision;

void on_mouse( int event, int x, int y, int flags, void* param );
std::vector<FeaturePtr> manDecFeatures;
std::vector<FeaturePtr> manDecFeaturesRet;
Image* manDecFrame;  
int manDecGroupId = 1;

void on_mouse( int event, int x, int y, int flags, void* param )
{
    if(manDecFrame)
    {
        if( ! manDecFrame->getIplImage()->origin )
        y =  manDecFrame->getIplImage()->height - y;

        if( event == CV_EVENT_LBUTTONDOWN )
        {
	    CvSize imgSize = cvGetSize(manDecFrame->getIplImage());
            //find closest feature
            std::vector<FeaturePtr>::const_iterator fetsIt = manDecFeatures.begin();
            double x1, y1, x2, y2, dist;
            x1 = (double) x;
            y1 = (double) y;

            double min = 9999;
            FeaturePtr ft, ftMin;
            while(fetsIt != manDecFeatures.end())
            {
                 ft = *(fetsIt);
                 x2 = ft->getX();
                 y2 = ft->getY();
                 dist =
                    sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) );

                if(dist < min)
                {
                    min = dist;
                    ftMin = *(fetsIt);
                }
                fetsIt++;
             }

                ftMin->setGroupId(manDecGroupId);
                cvCircle( manDecFrame->getIplImage(),
                cvPointFrom32f(cvPoint2D32f( ftMin->getX(), manDecFrame->getHeight() - ftMin->getY() )), 3,
                CV_RGB(255,255 * (manDecGroupId-1),0), -1, 8,0);
                manDecFeaturesRet.push_back(ftMin);
                cvShowImage( "select",  manDecFrame->getIplImage() );
        }
    }
}


ManualDetector::ManualDetector(const FeatureDetector* fd)
{
    _fd = fd;
    manDecFrame = 0;
}
// ============================================================================

ManualDetector::~ManualDetector()
{
   if(manDecFrame) delete manDecFrame;
}
// ============================================================================


std::vector<FeaturePtr> ManualDetector::detect(const Image *img,
                                             const Image *mask) const
{
    if(manDecFrame) delete manDecFrame;
    manDecFrame = img->clone();


    std::vector<FeaturePtr> ftList;
    FeaturePtr ft;

    manDecFeatures = _fd->detect(manDecFrame, mask);

    std::vector<FeaturePtr>::const_iterator fetsIt = manDecFeatures.begin();
    double x, y;

    while(fetsIt != manDecFeatures.end())
    {
        ft = *(fetsIt++);
        x = ft->getX();
        y = ft->getY();

        cvCircle( manDecFrame->getIplImage(),
                  cvPointFrom32f(cvPoint2D32f( x, manDecFrame->getHeight() - y )), 3,
                  CV_RGB(0,0,255), -1, 8,0);
    }


    cvNamedWindow( "select", 0 );
    cvShowImage( "select",  manDecFrame->getIplImage() );
    cvSetMouseCallback( "select", on_mouse, 0 );

    while(true)
    {
        if( cvWaitKey(10) != -1 ) break;
    }
    manDecGroupId++;

//     while(true)
//    {
//        if( cvWaitKey(10) != -1 ) break;
//    }

    cvDestroyWindow("select");
    
    if( manDecFeaturesRet.size() >0)
	return manDecFeaturesRet;
    else
    	return manDecFeatures;

}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
