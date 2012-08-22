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
#pragma once

#include <string>

////global definitions
///**
// * @brief
// *
// */
//enum JointType{REVOLUTE, PRISMATIC, VIRTUAL_PRISMATIC, DISCONNECTED, UNKNOWN, RIGID};
/**
 * @brief
 *
 */
enum ColorType{RED=0,BLUE,GREEN,YELLOW,ORANGE,PURPLE,BLACK,WHITE,VIOLET,PINK,LIGHTBLUE,COLORFUL};

//always declare global variables as static
__attribute__((unused)) static const char /**< TODO */ *ColorTypeString[]={"RED","BLUE","GREEN","YELLOW","ORANGE","PURPLE","BLACK","WHITE","VIOLET","PINK","LIGHTBLUE","COLORFUL"};
static const int numColorTypes=12; /**< TODO */

static const double globalRedChannel[] = {1,0,0,1,1	  ,0.5,0,1,0.78,0.93,0.12,0}; /**< TODO */
static const double globalGreenChannel[]={0,0,1,1,0.65,0.0,0,1,0.08,0.07,0.56,0}; /**< TODO */
static const double globalBlueChannel[]= {0,1,0,0,0   ,0.5,0,1,0.52,0.54,1.0 ,0}; /**< TODO */
//setColor(dReal r, dReal g, dReal b, dReal alpha);

/**
 * @brief
 *
 */
enum PushStrength{WEAK=10, MEDIUM=50, STRONG=100};


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#define VECTOR_PUSHBACK_ALL(vec, addVec) if ( (addVec).size() > 0 ) \
        (vec).insert((vec).end(), (addVec).begin(), (addVec).end())
