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
 * CubeCornerFeature Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 19:08:37 $
 * $Revision: 1.2 $
 *****************************************************************************/

#include "cubeCornerFeature.h"
#include <string>
#include <iostream>
#include "image.h"
#include "pixel.h"

using namespace vision;

CubeCornerFeature::CubeCornerFeature(float x, float y, bool lost,
                                         float error, float quality)
    : Feature(x,y,lost,error, quality, 0) //last variable groupId=0 by default
{
    _range = 50;
    _colSets = 5;
}
// ============================================================================

CubeCornerFeature::~CubeCornerFeature()
{
}
// ============================================================================

FeaturePtr CubeCornerFeature::clone() const
{
    FeaturePtr copy(new CubeCornerFeature(Feature::_x, Feature::_y, Feature::_lost,
                            Feature::_error));

    return copy;
}
// ============================================================================

float CubeCornerFeature::compareTo(std::pair<float, float> pt,
                 const Image* img, const Image* mask)
{
    int height = img->getHeight();
    int width = img->getWidth();
    int i0 = height - (int)pt.second;
    int j0 = (int)pt.first;
    const int ft_size = 2;

    float sumSqDiff = 0;
    for(int i = -ft_size; i <= ft_size; i++)
    {
        for(int j = -ft_size; j <= ft_size; j++)
        {
            int iInd = i0 + i;
            int jInd = j0 + j;
            if((iInd >= height) || (iInd < 0) || (jInd >= width) || (jInd < 0))
            {

            }else{
                Pixel px = img->get2D(iInd, jInd);
                sumSqDiff += (_descriptor[i + 2][j + 2] - px.GRAY_VALUE) *
                             (_descriptor[i + 2][j + 2] - px.GRAY_VALUE);
            }
        }
    }
    return sqrt(sumSqDiff);
}
// ============================================================================

bool CubeCornerFeature::isFeature(std::pair<float, float> pt,
               const Image* img, const Image* mask)
{
    int _format[1][21][21] ={{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}};

    int _colors[] = {-1, -1, -1, -1, -1};

    int height = img->getHeight();
    int width = img->getWidth();
    int i0 = height - (int)pt.second - 1;
    int j0 = (int)pt.first;
    const int ft_size = 10;
    int formPos = 0;
    //std::cout << i0 << " " << j0 << "\n";
    Pixel pxM;
    if(mask != 0)
        pxM = mask->get2D(i0, j0);

    bool isFt;
    //bool pause = (i0 == 178) && (j0 == 191);
  do{
    isFt = mask == 0 || pxM.GRAY_VALUE > 0;
    for(int i = -ft_size; (i <= ft_size) && isFt; i++)
    {
        //if(pause) std::cout << std::endl;
        for(int j = -ft_size; (j <= ft_size) && isFt; j++)
        {
            int iInd = i0 + i;
            int jInd = j0 + j;
            if((iInd >= height) || (iInd < 0) || (jInd >= width) || (jInd < 0))
            {
                isFt = false;
            }else{
                Pixel px = img->get2D(iInd, jInd);
                int colorIndex = _format[formPos][i + ft_size][j + ft_size];
                int expectedColor = _colors[colorIndex];
                if(expectedColor == -1){
                    _colors[colorIndex] = (int)px.GRAY_VALUE;
                }else{
                    for(int col = 0; col < _colSets; col++)
                    {
                        if(col == colorIndex){

                            isFt = isFt && abs(expectedColor - (int)px.GRAY_VALUE) < 50;
                        }else{

                            isFt = isFt && ((abs(_colors[col] - (int)px.GRAY_VALUE) >= 10) ||
                                            (_colors[col] == -1));
                        }
                    }
                }
            }
        }
    }
    formPos++;
    for(int cc = 0; cc < _colSets; cc++){
        _colors[cc] = -1;
    }

  }while(!isFt && formPos < 1);

  return isFt;
}
// ============================================================================

void CubeCornerFeature::generateDescriptor(std::pair<float, float> pt,
                             const Image *img, const Image *mask)
{
    int height = img->getHeight();
    int width = img->getWidth();
    int i0 = height - (int)pt.second;
    int j0 = (int)pt.first;
    const int ft_size = 2;

    for(int i = -ft_size; i <= ft_size; i++)
    {
        for(int j = -ft_size; j <= ft_size; j++)
        {
            int iInd = i0 + i;
            int jInd = j0 + j;
            if((iInd >= height) || (iInd < 0) || (jInd >= width) || (jInd < 0))
            {
                _descriptor[i + 2][j + 2] = -1;
            }else{
                Pixel px = img->get2D(iInd, jInd);
                _descriptor[i + 2][j + 2] = (int)px.GRAY_VALUE;
            }
        }
    }
}

/** ***************************************************************************
                                End of File
 ******************************************************************************/
