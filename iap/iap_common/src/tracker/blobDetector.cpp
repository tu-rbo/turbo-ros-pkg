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
 * BlobDetector Model (implementation) based off of ManualDetector
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:00:58 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "blobDetector.h"
#include "feature.h"
#include "image.h"
#include "cv.h"
#include <vector>
#include <iostream>

using namespace vision;

void on_mouse_select_blob( int event, int x, int y, int flags, void* param );
std::vector<FeaturePtr> blobFeatures;
std::vector<FeaturePtr> blobFeaturesRet;
Image* blobFrame;
int blobGroupId = 1;
bool blobMouseGood;
void selectBlob(Image* image, int x, int y);

void on_mouse_select_blob(int event, int x, int y, int flags, void* param) {
    if( event == CV_EVENT_LBUTTONDOWN) {
    	Image* testFrame;	
    	blobMouseGood = true;
        testFrame = ((Image*)param)->clone();
        selectBlob(testFrame, x, y);
        cvWaitKey(10);
    }
}

BlobDetector::BlobDetector(const FeatureDetector* fd)
{
    _fd = fd;
    blobFrame = 0;
}
// ============================================================================

BlobDetector::~BlobDetector()
{
   if(blobFrame) delete blobFrame;
}
// ============================================================================


std::vector<FeaturePtr> BlobDetector::detect(const Image *img,
                                             const Image *mask) const
{
    if(blobFrame) delete blobFrame;
    blobFrame = img->clone();


    std::vector<FeaturePtr> ftList;
    FeaturePtr ft;

    blobFeatures = _fd->detect(blobFrame, mask);

    std::vector<FeaturePtr>::const_iterator fetsIt = blobFeatures.begin();
    double x, y;

    while(fetsIt != blobFeatures.end())
    {
        ft = *(fetsIt++);
        x = ft->getX();
        y = ft->getY();

        cvCircle( blobFrame->getIplImage(),
                  cvPointFrom32f(cvPoint2D32f( x, blobFrame->getHeight() - y )), 3,
                  CV_RGB(0,0,255), -1, 8,0);
    }


    cvNamedWindow( "select", 0 );
    cvShowImage( "select",  blobFrame->getIplImage() );
    Image* original = img->clone();
    cvSetMouseCallback( "select", on_mouse_select_blob, original );


    while(true)
    {
        if( cvWaitKey(10) != -1 ) break;
    }
    blobGroupId++;


    //remove all features not in blob
    
    cvDestroyWindow("select");
    
    if( blobFeaturesRet.size() >0)
	return blobFeaturesRet;
    else
    	return blobFeatures;

}


void selectBlob(Image* image, int x, int y ) {
    //Inits
    CvPoint center;
    CvPoint pt = cvPoint((int)x, (int)y);
    CvConnectedComp comp;
    int offset = 11;
    CvScalar lo = CV_RGB(offset, offset, offset);
    CvScalar high = CV_RGB(offset, offset, offset);
    CvScalar region = CV_RGB(255, 255, 255);
    CvScalar red = CV_RGB(255, 0, 0);
    double centerX;
    double centerY;
    CvRect r;
    Image* flooded_image;
           
    //Fill Region around seed pt
    cvFloodFill( image->getIplImage(), pt, region, lo, high, &comp, 4 , NULL );
    
    //Create binary image, white=object & black=background
    image->color2GrayScale();
    image->display("Gray start");
    image->thresholdFrame(254, 255);
    image->display("thresholded start");
    flooded_image = image->clone();
    //Find connected Components in binary image
    IplImage* dst = cvCreateImage( cvGetSize(image->getIplImage()), 8, 3 );
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contour = 0;
    cvFindContours( image->getIplImage(), storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    cvZero( dst );
    
    for( ; contour != 0; contour = contour->h_next ) {
        if(fabs(cvContourArea( contour, CV_WHOLE_SEQ )) > 10) {
            
            CvScalar color = CV_RGB( 0, 0, 255 );
            cvDrawContours( dst, contour, color, color, -1, CV_FILLED, 8 );
            
            //Find bounding rectangle and calculate center
            r = cvBoundingRect( contour, 0 );
            centerX = r.x+(r.width/2);
            centerY = r.y+(r.height/2);
            center = cvPoint((int)centerX, (int)centerY);
            cvCircle( dst, center, 2, red, 5, 8, 0 );
            
        }
        
    }
    
    std::vector<FeaturePtr>::const_iterator fetsIt = blobFeatures.begin();
    int x2, y2;
    Image components(dst);
    cvNamedWindow( "Components", 1 );
    cvShowImage( "Components", dst );
    FeaturePtr ft;
    while(fetsIt != blobFeatures.end())
    {
         ft = *(fetsIt);
         CvPoint ftPoint = cvPointFrom32f(cvPoint2D32f( ft->getX(), blobFrame->getHeight() - ft->getY() ));
         x2 = ftPoint.x;
         y2 = ftPoint.y;
         int range = 5;
         for(int i = -1 * range; i <= range; ++i){
        	 for(int j = -1 * range; j <= range; ++j){
        		 if(y2 + i > 0 && y2 + i < components.getHeight()&& x2 + j > 0 && x2 + j < components.getWidth()){
        			 Pixel p = components.get2D( (int) y2 + i, (int)x2 + j ); 
        			 if( p.R == 0 && p.G == 0 && p.B == 255 ){
			            ft->setGroupId(blobGroupId);
			            cvCircle( 	blobFrame->getIplImage(),
			            			cvPointFrom32f(cvPoint2D32f( ft->getX(), blobFrame->getHeight() - ft->getY() )), 3,
			            			CV_RGB(255,255 * (blobGroupId-1),0), -1, 8,0);
			            blobFeaturesRet.push_back(ft);
			            cvShowImage( "select",  blobFrame->getIplImage() );
			        	i = range + 1;
			        	j = range + 1;
			         }
		         }
        	 }
         }
         fetsIt++;    
    }

    cvWaitKey();
            
    //*************************************************************

}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
