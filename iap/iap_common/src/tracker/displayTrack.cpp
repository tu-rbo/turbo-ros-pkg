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
 * DisplayTrack Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/11 17:53:18 $
 * $Revision: 1.2 $
 *****************************************************************************/
#include "displayTrack.h"
#include "stFeature.h"
#include "image.h"


#include "highgui.h"
#include <iostream>

using namespace vision;

DisplayTrack::DisplayTrack()
{
    _lastImg = 0;
    _writer = 0;
    _displayImage = true;
	_windowName = "video";
    cvNamedWindow( _windowName, 0 );
}

DisplayTrack::DisplayTrack(const char* windowName)
{
    _lastImg = 0;
    _writer = 0;
    _displayImage = true;
	_windowName = windowName;
    cvNamedWindow( _windowName, 0 );
}
// ============================================================================

DisplayTrack::DisplayTrack(std::string saveToFilename, int height, int width, double fps)
{
    _writer = cvCreateAVIWriter( saveToFilename.c_str(), CV_FOURCC('D','I','V','X'),
                                fps, cvSize(width, height), 0);
    _lastImg = 0;
    _displayImage = true;
    cvNamedWindow( _windowName, 0 );
}

DisplayTrack::~DisplayTrack()
{
    cvDestroyWindow(_windowName);
    if(_lastImg) delete _lastImg;
    if(_writer) cvReleaseVideoWriter(&_writer);
}
// ============================================================================

void DisplayTrack::notifyReset(const std::vector<FeaturePtr> &features)
{
    if(!features.empty())
    {
        lost.assign(features.size(), false);
    }
}
// ============================================================================

void DisplayTrack::notifyStep(const Image &img,
                              const std::vector<FeaturePtr> &features)
{
    if(_lastImg) delete _lastImg;
    _lastImg = img.clone();
    IplImage *iimg = _lastImg->getIplImage();


    if(!features.empty())
    {
        if(lost.size() == 0)
        {
            lost.assign(features.size(), false);
        }
        float x, y;
        int i = 0;
        std::vector<FeaturePtr>::const_iterator featuresIt;
        for(featuresIt = features.begin(); featuresIt != features.end();
            featuresIt++)
        {
            FeaturePtr feat = *(featuresIt);

            x = feat->getX();
            y = feat->getY();

            if( !feat->isLost() )
            {
                cvCircle( iimg, cvPointFrom32f(cvPoint2D32f(x, img.getHeight() - y - 1)), 3,
                          CV_RGB(0,255,0), -1, 8,0);
		//std::cout << feat->getX() << " " << feat->getY() << " " << feat->getQuality() << std::endl;

            }else
            {
                if(!lost.at(i))
                {
                    std::cout << "lost feature: " << i << std::endl;
                    lost.at(i) = true;
                }
            }
            i++;
        }
    }
    if(_writer) cvWriteFrame(_writer, iimg);
    if(_displayImage)
       cvShowImage( _windowName, iimg );
    //cvWaitKey(10);
}

void DisplayTrack::setDisplayImageVar(bool value)
{
	_displayImage = value;
}

void DisplayTrack::addCircle(const Feature *f){
  IplImage *iimg = _lastImg->getIplImage();
  cvCircle(iimg, cvPointFrom32f(cvPoint2D32f(f->getX(),_lastImg->getHeight()-f->getY()-1 )), 1, CV_RGB(f->getR(),f->getG(),f->getB()), -1, 8,0);
  if(_writer) cvWriteFrame(_writer, iimg);
  if(_displayImage) cvShowImage( _windowName, iimg );
}

void DisplayTrack::addCircles(const std::vector<Feature> features, int thickness){
	IplImage *iimg = _lastImg->getIplImage();
	for(std::vector<Feature>::const_iterator it=features.begin();it!=features.end();it++){
		cvCircle(iimg, cvPointFrom32f(cvPoint2D32f(it->getX(),_lastImg->getHeight()-it->getY()-1 )), 1, CV_RGB(255,0,0), thickness, 8,0);
	}
	if(_writer) cvWriteFrame(_writer, iimg);
	if(_displayImage) cvShowImage( _windowName, iimg );
}

void DisplayTrack::addEdges(const std::vector<Feature> &features, bool **connectivity){
  IplImage *iimg = _lastImg->getIplImage();

  for(unsigned int i=0;i<features.size();i++){
    for(unsigned int j=i+1; j<features.size(); j++){
      if(connectivity[i][j]){
	cvLine( iimg, cvPointFrom32f(cvPoint2D32f(features[i].getX(),_lastImg->getHeight()-features[i].getY()-1 )),
                      cvPointFrom32f(cvPoint2D32f(features[j].getX(),_lastImg->getHeight()-features[j].getY()-1 )), CV_RGB( 0, 255, 0 ));
      }
    }
  }

  if(_writer) cvWriteFrame(_writer, iimg);
  if(_displayImage) cvShowImage( _windowName, iimg );
}

void DisplayTrack::clear(){
	IplImage *iimg = _lastImg->getIplImage();
	Image img(iimg);
	img.zero();
}
			
			
	
/*****************************************************************************
                                End of File
 ******************************************************************************/
