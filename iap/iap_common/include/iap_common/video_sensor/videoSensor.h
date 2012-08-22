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
#ifndef VIDEO_SENSOR_H
#define VIDEO_SENSOR_H

#include "sensor.h"
#include "cv.h"


namespace vision{

	class ImageInStream;
	class Image;

	/**
	*\class    VideoSensor
	*\brief    Concrete Sensor.
	*A concrete extention of Sensor. This class optains sensor data from an
	*ImageInStream 
	*$Author: dubik $
	*$Date: 2009/03/10 15:01:13 $
	*$Revision: 1.1 $
	*****************************************************************************/
	class VideoSensor : public Sensor{
	public:
		/**
		* Creates the VideoSensor from the given file name
		* 
		* @param path - Path to location of video file
		*/	
		VideoSensor(std::string path);

		VideoSensor(std::vector< std::string > fileList);

		/**
		* Creates the VideoSensor from the camera input
		*/
		VideoSensor();


		/**
		* Default destructor
		*/
		virtual ~VideoSensor();

		/**
		* Extracts the next image from the stream and notifies all step Listeners
		* @throws std::string - Throws if there is no more data in the sensor
		*/ 
		virtual void step();

		/**
		* calls step() count times and returns the number of sucessfull calls to
		* step()
		* 
		* @param count - number of times to call step
		* @return - number of sucessfull calls to step
		*/
		virtual int step(int count);

		/**
		* an accesor to the curent image in the video stream for use by
		* SensorListeners
		* 
		* @return - the image optained from the last call to step
		*/
		const Image* getCurrentImage() const;

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
		* sets the intrinsic camera parameters
		* @param fx focal length expressed in pixel-related units
		* @param fy focal length expressed in pixel-related units
		* @param cx the principal point (that is usually at the image center)
		* @param cy the principal point (that is usually at the image center)
		* @param k1 radial distortion coefficient
		* @param k2 radial distortion coefficient
		* @param p1 tangential distortion coefficient
		* @param p2 tangential distortion coefficient
		* @param k3 radial distortion coefficient (normally zero)
		*/
		void setCameraCalibration(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3=0.0);

		/**
		* Calibrates the camera using the Chessboard approach of OpenCV
		*
		* @param1 columns - Number of columns of the chessboard
		* @param2 rows - Number of rows of the chessboard
		* @return - True if the calibration process was completed succesfully
		*/
		bool calibrateCamera(int columns, int rows);

	private:
		/**
		* The notifyStep(..) of every SensorListener is called
		*/
		void notifyAll();

		ImageInStream* _iin; /**< the helper object to query the camera of file */
		Image* _currentImage; /**< the last image optained from calling step() */

		//parameters for fixing lens distortion
		CvMat* cameraMatrix;
		CvMat* distCoeffs;
		bool isCalibrated;

	};
};
#endif
