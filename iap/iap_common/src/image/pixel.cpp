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
#include "pixel.h"
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace vision;

Pixel::Pixel(){
    R=0;
    G=0;
    B=0;
    GRAY_VALUE=0;
    row = 0;
    col = 0;
    selected = false;
}

Pixel::Pixel(int red, int green, int blue, int r, int c, float gray) {
    R=(float)red;
    G=(float)green;
    B=(float)blue;
    row = r;
    col=c;
    GRAY_VALUE=gray;
    selected = false;
}

Pixel::~Pixel(){


}

Pixel::Pixel(const Pixel &p) {
    R = p.R;
    G = p.G;
    B = p.B;
    GRAY_VALUE = p.GRAY_VALUE;
}


void Pixel::setRow(int r){
    row = r;
}

void Pixel::setCol(int c) {
    col = c;
}

void Pixel::setSelected(bool s) {
    selected = s;
}

void Pixel::setGrayValue(float g) {
    GRAY_VALUE = g;
}

void Pixel::setRedValue(int r) {
    R=(float)r;
}

void Pixel::setGreenValue(int g) {
    G=(float)g;
}

void Pixel::setBlueValue(int b) {
    B=(float)b;
}
int Pixel::getRow(){
    return row;
}

float Pixel::getRedValue() const{
    return R;
}

float Pixel::getGreenValue() const{
    return G;
}

float Pixel::getBlueValue() const{
    return B;
}

int Pixel::getCol() {
    return col;
}
bool Pixel::getSelected(){
    return selected;
}

float Pixel::getGrayValue(){
    return GRAY_VALUE;
}

bool Pixel::operator<(const Pixel &p) const {
	if(R<p.R){
		return true;
	}else if(R==p.R){
		if(G<p.G){
			return true;
		}else if(G==p.G){
			if(B<p.B){
				return true;
			}else{
				return false;
			}
		}else{
			return false;
		}
	}else{
		return false;
	}
}
