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
 * Image Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/05/29 19:56:38 $
 * $Revision: 1.2 $
 *****************************************************************************/

#include "image.h"
#include <iostream>
#include <cv.h>
#include <highgui.h>
//#include <pnmfile.h>
//#include "segment-image.h"

using namespace std;
using namespace vision;

Image::Image(int width, int height, int channels) {
    if((channels != 3) && (channels != 1)) {
        throw std::string(
                "only 3 channel color and 1 channel greyscale are supported");
    }
    _image =cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, channels);
    _isColor = (channels == 3);
    this->zero();
} //===========================================================================

Image::Image(IplImage* image) {
    if(image->depth != IPL_DEPTH_8U) {
        throw std::string("only 8 bit  images are suported");
    }
    if((image->nChannels != 3) && (image->nChannels != 1)) {
        throw std::string(
                "only 3 channel color and 1 channel greyscale are supported");
    }
    _isColor = (image->nChannels == 3);
    _image=cvCloneImage(image);
} //===========================================================================

Image::Image(string &filename){
	_image=cvLoadImage(filename.c_str());
	 _isColor = (_image->nChannels == 3);	
} //===========================================================================

Image::~Image() {
    cvReleaseImage(&_image);
} //===========================================================================

void Image::saveImage(string &filename) {
    cvSaveImage(filename.c_str(), _image);
}//===========================================================================

int Image::display(char *windowName) {
    //cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(windowName, 0);
    
    cvShowImage(windowName, this->getIplImage());
    
    int keyPressed = (cvWaitKey(10) & 255);
    return keyPressed;
} //===========================================================================
IplImage* Image::getIplImage(){
    return _image;
}
const IplImage* Image::getIplImage() const{
    return _image;
}
// ============================================================================
Image* Image::clone() const {
    if(_image ==0 || _image == NULL) {
        printf("WARNING: ActiveSegmenter::step(): cloning a null image\n");
    }
    Image* img = new Image(_image);
    return img;
} //===========================================================================
Image& Image::operator=(const Image &i) {
    
    this->_image = i._image;
    this->_isColor = i._isColor;
  
    
    return *this;
}//===========================================================================F
int Image::getWidth() const {
    int width = 0;
    if(_image) {
        width = cvGetSize(_image).width;
    }
    return width;
} //===========================================================================

int Image::getHeight() const {
    int height = 0;
    if(_image) {
        height = cvGetSize(_image).height;
    }
    return height;
} //===========================================================================

void Image::color2GrayScale() {
    /*
     * if(!_isColor) {
     * throw std::string(
     * "The image is already greyscale and can not be converted");
     * }
     */
    IplImage *grey = cvCreateImage( cvGetSize(_image), IPL_DEPTH_8U, 1 );
    cvCvtColor( _image, grey, CV_BGR2GRAY );
    cvReleaseImage(&_image);
    _image = 0;
    _image = grey;
    _isColor = false;
} //===========================================================================
void Image::gray2Color() {
 
    IplImage *color = cvCreateImage( cvGetSize(_image), IPL_DEPTH_8U, 3 );
    cvCvtColor( _image, color, CV_GRAY2BGR );
    cvReleaseImage(&_image);
    _image = 0;
    _image = color;
    _isColor = true;
} //===========================================================================

Pixel Image::get2D(int pRowI, int pColJ) const {
    Pixel p;
    CvScalar opencvPixel;
    opencvPixel = cvGet2D(this->getIplImage(), pRowI, pColJ);
    p.GRAY_VALUE = (float)opencvPixel.val[0];
    p.B = (float)(opencvPixel.val[0]);
    p.G = (float)(opencvPixel.val[1]);
    p.R = (float)(opencvPixel.val[2]);
    
    return p;
    /*
     * if((pRowI < 0) || (pRowI >= this->getHeight()) ||
     * (pColJ < 0) || (pColJ >= this->getWidth())){
     * throw std::string("can not get pixel, value out of bounds");
     * }
     *
     * int step       = _image->widthStep/sizeof(uchar);
     * int channels   = _image->nChannels;
     * uchar * data    = (uchar *)_image->imageData;
     *
     * Pixel pix;
     * if(_isColor)
     * {
     * pix.B = data[pRowI*step+pColJ*channels+0];
     * pix.G = data[pRowI*step+pColJ*channels+1];
     * pix.R = data[pRowI*step+pColJ*channels+2];
     * }
     * else
     * {
     * pix.GRAY_VALUE = data[pRowI*step+pColJ*channels+0];
     * }
     * return pix;
     */
} //===========================================================================

void Image::set2D(int pRowI, int pColJ, const Pixel &px) {
    if(_isColor)
        cvSet2D(this->getIplImage(), pRowI, pColJ, CV_RGB(px.R, px.G, px.B));
    else {
        CvScalar gray;
        gray.val[0] = (double)px.GRAY_VALUE;
        cvSet2D(this->getIplImage(), pRowI, pColJ, gray);
    }
    /*
     * if((pRowI < 0) || (pRowI >= this->getHeight()) ||
     * (pColJ < 0) || (pColJ >= this->getWidth())){
     * throw std::string("can not set pixel, value out of bounds");
     * }
     *
     * int step       = _image->widthStep/sizeof(uchar);
     * int channels   = _image->nChannels;
     * uchar * data    = (uchar *)_image->imageData;
     *
     * //NOTE: OpenCV Image is BGR!
     * if(_isColor){
     * data[pRowI*step+pColJ*channels+0] = (uchar) px.B;
     * data[pRowI*step+pColJ*channels+1] = (uchar) px.G;
     * data[pRowI*step+pColJ*channels+2] = (uchar) px.R;
     * }else{
     * data[pRowI*step+pColJ*channels+0] = px.GRAY_VALUE;
     * }
     */
} //===========================================================================
void Image::zero() {
    cvSetZero( _image );
} //===========================================================================
void Image::drawCircle(int x, int y, uchar R, uchar G, uchar B, int radius){
	CvPoint p;
	p.x=x;
	p.y=(_image->height)-y-1 ;
	cvCircle(_image, p, radius, CV_RGB(R,G,B), -1, 8,0);
}//===========================================================================
void Image::drawLine(int x1, int y1, int x2, int y2, uchar R, uchar G, uchar B){
	CvPoint pt1,pt2;
	pt1.x=x1;
	pt1.y=y1;
	pt2.x=x2;
	pt2.y=y2;

	int thickness=1;
	cvLine(_image,pt1,pt2,CV_RGB(R,G,B),thickness);
}//===========================================================================
void Image::drawCircles(vector<Feature> features, map<int,int> clusters, vector<int> featuresID){
	vector<vector<int> > colors;
	for(map<int,int>::iterator it=clusters.begin();it!=clusters.end();it++){
		vector<int> featColor;
		featColor.push_back(rand()%255);
		featColor.push_back(rand()%255);
		featColor.push_back(rand()%255);
		colors.push_back(featColor);
	}

	vector<vector<int> >::iterator itColors=colors.begin();
	for(map<int,int>::iterator it=clusters.begin();it!=clusters.end();it++){
		for(size_t i=0;i<features.size();i++){
			if(featuresID[i]==it->first){
				if(it->second>8){
					drawCircle((int)features[i].getX(),/*_image->height-*/(int)features[i].getY(),(*itColors)[0],(*itColors)[1],(*itColors)[2],3+2);
				}
			}
		}
		itColors++;
	}
}//===========================================================================
void Image::drawLines(vector<Feature> features, map<int,int> clusters, vector<int> featuresID, vector<vector<bool> > connectivityMatrix){
	vector<vector<int> > colors;
	for(map<int,int>::iterator it=clusters.begin();it!=clusters.end();it++){
		vector<int> featColor;
		featColor.push_back(rand()%255);
		featColor.push_back(rand()%255);
		featColor.push_back(rand()%255);
		colors.push_back(featColor);
	}

	int numFeatures=features.size();
	
	for(int i=0;i<numFeatures;i++){
		for(int j=0;j<numFeatures;j++){
			if(connectivityMatrix[i][j] && featuresID[i]==featuresID[j]){
				//the two features are connected & in the same cluster
				//note: two features may be connected but also clustered in different groups
				//      in this case we dont want a line between them
				Feature f1=features[i];
				Feature f2=features[j];

				if(clusters[featuresID[i]]>8){		//look for the size of the cluster in which feature i is
					//cluster size is larger than 8
					drawLine((int)f1.getX(),_image->height-(int)f1.getY(),(int)f2.getX(),_image->height-(int)f2.getY(),255,255,255);
				}
			}
		}
	}
}//===========================================================================
void Image::drawRectangle(int x1, int y1, int x2, int y2,
        uchar R, uchar G, uchar B,
        int fillType, int width) {
    CvPoint p1;
    p1.x = x1;
    p1.y = y1;
    
    CvPoint p2;
    p2.x = x2;
    p2.y = y2;
    
    cvRectangle(_image, p1, p2, CV_RGB((int)R, (int)G, (int)B),
            width, fillType, 0);
} //===========================================================================
void Image::drawRectangle(int x1, int y1, int x2, int y2,
        const Pixel &p, int fillType, int width) {
    CvPoint p1;
    p1.x = x1;
    p1.y = y1;
    
    CvPoint p2;
    p2.x = x2;
    p2.y = y2;
    
    cvRectangle(_image, p1, p2, CV_RGB((int)p.R, (int)p.G, (int)p.B),
            fillType, width, 0);
} //===========================================================================
void Image::rotateFrame(double theta) {
    
    IplImage* dst = cvCloneImage(_image);
    
    CvMat *N;
    CvPoint2D32f center;
    center.x = getWidth()*.5f;
    center.y = getHeight()*.5f;
    
    N = cvCreateMat(2, 3, CV_32F);
    cv2DRotationMatrix(center, theta, 1.0, N);
    cvWarpAffine(_image, dst, N);
    
    _image = cvCloneImage(dst);
    
    cvReleaseImage(&dst);
} //===========================================================================
void Image::translateFrame(double deltaX, double deltaY) {
   
    IplImage* dst = cvCloneImage(_image);
   
    CvMat *M;
    M = cvCreateMat(2, 3, CV_32F);
    cvmSet(M, 0, 0, 1);
    cvmSet(M, 0, 1, 0);
    cvmSet(M, 0, 2, deltaX);
    cvmSet(M, 1, 0, 0);
    cvmSet(M, 1, 1, 1);
    cvmSet(M, 1, 2, deltaY);
    
    cvWarpAffine(_image, dst, M);
 
    _image = cvCloneImage(dst);
    cvReleaseImage(&dst);
}//============================================================================
void Image::normalizeFrame() {
    CvScalar tmp;
    double maxPixelValue = 0;
    double scale;
	
    for(int i=0; i< _image->height; i++)
        for(int j=0;j< _image->width; j++) {
        tmp.val[0] = cvGet2D(_image, i, j).val[0];
        if(tmp.val[0] > maxPixelValue) {
            maxPixelValue = tmp.val[0];
        }
        }
    scale = 255.0/maxPixelValue;
    
    if(scale != 1.0) {
        for(int i=0; i<_image->height; i++)
            for(int j=0;j<_image->width; j++) {
            tmp.val[0] = cvGet2D(_image, i, j).val[0]*scale;
            cvSet2D(_image, i, j, tmp);
            }
    }
} //===========================================================================
void Image::thresholdFrame(double threshold, int maxValue) {

    
    cvThreshold(_image, _image, threshold, maxValue, CV_THRESH_BINARY);
    
} //===========================================================================
void Image::thresholdFrameInv(double threshold, int maxValue) {


    cvThreshold(_image, _image, threshold, maxValue, CV_THRESH_BINARY_INV);

} //===========================================================================
void Image::adaptThresholdFrame(int maxValue, int blockSize, double param1) {
    
    cvAdaptiveThreshold(_image, _image, maxValue, CV_ADAPTIVE_THRESH_MEAN_C,
            CV_THRESH_BINARY, blockSize, param1);

} //===========================================================================
void Image::erodeFrame(IplConvKernel* strctEle, int iterations)  {
    
    cvErode(_image, _image, strctEle, iterations);
    
} //===========================================================================
void Image::dilateFrame(IplConvKernel* strctEle, int iterations) {
    
    cvDilate(_image, _image, strctEle, iterations);
    
} //===========================================================================
void Image::smoothFrame() {

    cvSmooth(_image, _image, CV_GAUSSIAN, CV_BLUR);
    
} //===========================================================================

void Image::saveAsPPM(string &name){
  ofstream f;
  f.open(name.c_str());

  int IMGW=_image->width;
  int IMGH=_image->height;

  f<<"P6"<<endl;
  f<<IMGW<<" "<<IMGH<<endl;
  CvScalar opencvPixel;

  f<<255<<endl;
  for(int i=0;i<IMGH;i++){
    for(int j=0;j<IMGW;j++){
      opencvPixel = cvGet2D(_image, i, j);	
      char B=(char)opencvPixel.val[0];
      char G=(char)opencvPixel.val[1];
      char R=(char)opencvPixel.val[2];
      f<<R<<G<<B;
    }
  }

  f.close();
} //===========================================================================

//segmentation::segimage<segmentation::rgb>* Image::IplToSegimage() {
//  int width = _image->width;
//  int height = _image->height;
//
//  segmentation::segimage<segmentation::rgb> *im = new segmentation::segimage<segmentation::rgb>(width, height);
//
//  segmentation::rgb *val=new segmentation::rgb[width*height];
//
//  CvScalar opencvPixel;
//  for(int y=0;y<height;y++){
//    for(int x=0;x<width;x++){
//      opencvPixel = cvGet2D(_image, y, x);
//      char B=(char)opencvPixel.val[0];
//      char G=(char)opencvPixel.val[1];
//      char R=(char)opencvPixel.val[2];
//      val[y*width+x].r=R;
//      val[y*width+x].g=G;
//      val[y*width+x].b=B;
//    }
//  }
//
//  memcpy(im->data, val, width * height * sizeof(segmentation::rgb));
//
//  delete(val);
//
//  return im;
//}
//
//Image* Image::SegimageToIpl(segmentation::segimage<segmentation::rgb>* in) {
//  int width = in->width();
//  int height = in->height();
//  int channels = 3;
//
//  Image *out=new Image(width, height, channels);
//
//  for(int y=0;y<height;y++){
//    for(int x=0;x<width;x++){
//      segmentation::rgb val=in->data[y*width+x];
//      Pixel p(val.r, val.g, val.b);
//      out->set2D(y,x,p);
//    }
//  }
//
//  return out;
//}
//
//Image* Image::graphSegment(float sigma, float k, int min_size){
//  segmentation::segimage<segmentation::rgb> *inImg = IplToSegimage();
//  int num_ccs;
//  segmentation::segimage<segmentation::rgb> *seg = segmentation::segment_image(inImg, sigma, k, min_size, &num_ccs);
//  printf("got %d components\n", num_ccs);
//  Image *segmentedImg=SegimageToIpl(seg);
//  return segmentedImg;
//}

/*****************************************************************************
 * End of File
 ******************************************************************************/
