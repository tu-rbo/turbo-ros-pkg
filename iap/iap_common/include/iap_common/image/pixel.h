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
#ifndef __PIXEL_H__
#define __PIXEL_H__
#include <iostream>
#include <fstream>
#include <cmath>

//TODO: ADD CPP FILE FOR IMPLEMENTED METHODS

namespace vision {
	/** ***************************************************************************
	*\class    Pixel
	*\brief    Container for an image pixel. The pixel can be grayscale and/or RGB
	*          color, later this might be split into different classes but for now
	*          one class will suffice -the programmer must assume what type they
	*          are using, so be smart..
	* $Author: dubik $
	* $Date: 2009/05/29 19:56:38 $
	* $Revision: 1.2 $
	*****************************************************************************/

	class Pixel {
	public:

		Pixel();
		Pixel(int red, int green, int blue, int r, int c, float gray);  
		Pixel(const Pixel &p);
		Pixel(float grayValue) {GRAY_VALUE=grayValue;};
		Pixel(int r, int g, int b) {R=(float)r;G=(float)g;B=(float)b;};    
		Pixel(float r, float g, float b){R=r; G=g; B=b;};
		~Pixel();

		void setRow(int r);
		void setCol(int c);
		void setSelected(bool s);
		void setGrayValue(float g);
		void setRedValue(int r);
		void setGreenValue(int g);
		void setBlueValue(int b);
		int getRow();
		int getCol();
		bool getSelected();
		float getGrayValue();
		float getRedValue() const;
		float getGreenValue() const;
		float getBlueValue() const;
		float R; /**<red value [0-255] */
		float G; /**<green value [0-255] */
		float B; /**<blue value [0-255] */
		float GRAY_VALUE; /**<grayscale value [0-255] */
		int row;
		int col;
		bool selected;

		Pixel sqRoot(){return Pixel((int)sqrt(R), (int)sqrt(G), (int)sqrt(B)); } 

		bool operator==(const Pixel &p) const { return ((p.R == R) && (p.G == G) && (p.B == B));}
		bool operator<(const Pixel &p) const;
		void operator+=(const Pixel &p) { R+=p.R; G+=p.G; B+=p.B; }
		void operator/=(int scale) { R/=scale; G/=scale; B/=scale; }
		Pixel operator-(const Pixel &p1) { return Pixel(R-p1.R,G-p1.G,B-p1.B); }
		Pixel operator*(const Pixel &p1) { return Pixel(R*p1.R,G*p1.G,B*p1.B); }
		Pixel operator/(int scale) { return Pixel(R/scale, G/scale, B/scale); }

		friend std::ostream& operator<<(std::ostream& os, const Pixel& p){
			return os<<"("<<p.R<<","<<p.G<<","<<p.B<<")";
		}

	};
}

#endif

/** ***************************************************************************
* End of File
******************************************************************************/
