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
#ifndef IMAGE_H
#define IMAGE_H

#include "cv.h"
#include "opencv2/legacy/legacy.hpp"
#include "pixel.h"
#include "misc.h"
//#include "segimage.h"
#include "feature.h"
#include <string>
#include <boost/shared_ptr.hpp>

namespace vision{

//forward declaration
class Feature;
typedef boost::shared_ptr<Feature> FeaturePtr;

/**
 *\class    Image
 *\brief    A wrapper for the opencv image object
 *$Author: dubik $
 *$Date: 2009/05/29 19:56:38 $
 *$Revision: 1.2 $
 *****************************************************************************/

 class Image{
  public:

    Image(int width, int height, int channels=3);

    /**
     * Constructs this class from openCV image.
     * 
     * @param img - openCV image
     */
    Image(IplImage* image);

	Image(std::string &filename);

    /**
     * Releases the openCV image 
     */
    virtual ~Image();

    /**
     * Gets the openCV image
     * @return - openCV image
     */
    IplImage* getIplImage();

    const IplImage* getIplImage() const;

    /**
     * Clones this image and returns its pointer
     * @return - A pointer to the cloned copy of this image
     */
    Image* clone() const;
    void saveImage(std::string &filename);
    int display(char *windowName); //temp solution
    void zero();
    void set2D(int pRowI, int pColJ, const Pixel &px);
    void color2GrayScale();
    void gray2Color();
    int getWidth() const;
    int getHeight() const;
    Pixel get2D(int pRowI, int pColJ) const;
	void drawCircle(int x, int y, uchar R, uchar G, uchar B, int radius = 3);
	void drawLine(int x1, int y1, int x2, int y2, uchar R, uchar G, uchar B);
	void drawCircles(std::vector<Feature> features, std::map<int,int> clusters, std::vector<int> featuresID);
	void drawLines(std::vector<vision::Feature> features, std::map<int,int> clusters, std::vector<int> featuresID, std::vector<std::vector<bool> > connectivityMatrix);

    void drawRectangle(int  x1, int  y1, int  x2, int y2,
		               uchar R, uchar G, uchar B, int fillType = 8, int width = 1);
    void drawRectangle(int x1, int y1, int x2, int y2,
                       const Pixel &p, int fillType = 8, int width = 1);
    void rotateFrame(double theta);
    void translateFrame(double deltaX, double deltaY);
    void normalizeFrame();  
    void thresholdFrame(double threshold, int maxValue);
    void thresholdFrameInv(double threshold, int maxValue);
    void adaptThresholdFrame( int maxValue, int blockSize, double param1);
    void dilateFrame(IplConvKernel* strctEle, int iterations);
    void erodeFrame(IplConvKernel* strctEle,int iterations);
    void smoothFrame();
    Image& operator=(const Image &i);
 
    //image segmentation using graphs
    void saveAsPPM(std::string &name);
//    Image* graphSegment(float sigma, float k, int min_size);
//    segmentation::segimage<segmentation::rgb>* IplToSegimage();
//    Image* SegimageToIpl(segmentation::segimage<segmentation::rgb>* in);
    
private:
     IplImage* _image; /**< The openCV image that is being wrapped */
     bool _isColor; /**< if false, the image is greyscale */
 };
};
#endif

