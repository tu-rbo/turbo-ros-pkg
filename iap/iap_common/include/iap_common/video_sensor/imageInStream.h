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
#ifndef IMAGE_IN_STREAM_H
#define IMAGE_IN_STREAM_H



#include <string>
#include <vector>

#include "highgui.h"
#include "image.h"

namespace vision{
	/**
	*\class    ImageInStream
	*\brief    Visual data input stream object. It supports streaming from a camera
	*          or a video file and acts as a wrapper arount the openCV 'capture'
	*$Author: dubik $
	*$Date: 2009/03/10 15:01:11 $
	*$Revision: 1.1 $
	*****************************************************************************/
	class ImageInStream{
	public:
		/**
		* Sets the camera as the input device.
		*/
		ImageInStream();

		/**
		* Sets a video file as the input source.
		* 
		* @param location - The path to the video device 
		*/ 
		ImageInStream(std::string location);

		ImageInStream(std::vector< std::string > fileList);

		/**
		* Releases the capture source.
		*/ 
		virtual ~ImageInStream();

		/**
		* Allocates, initializes and returns a pointer to the current image 
		* from the stream.
		* 
		* @return - The pointer to an alocated  
		*/
		Image* getNextImage();

		/**
		* returns the frames per second rate of the video source
		*/
		double getFps() const;

		/**
		* Returns the number of frames of the sequence 
		* NOTE: Use only with video files!
		*/
		double getFrameCount() const;

		/**
		* Returns the (Cv)size of the video stream
		*/
		CvSize getSize() const;

		/**
		* Calibrates the camera using the Chessboard approach of OpenCV
		*
		* @param1 columns - Number of columns of the chessboard
		* @param2 rows - Number of rows of the chessboard
		* @return - True if the calibration process was completed succesfully
		*/
		bool calibrateCamera(int columns, int rows);

	private:
		CvCapture* capture; /**< The openCV capture that is being wrapped */
		std::vector< std::string > images;
	};
};
#endif

