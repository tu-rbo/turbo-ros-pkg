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
 * BottomMaskedFeature Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 19:08:36 $
 * $Revision: 1.2 $
 *****************************************************************************/

#include "bottomMaskedFeature.h"
#include <string>
#include <iostream>
#include "image.h"
#include "pixel.h"

using namespace vision;

BottomMaskedFeature::BottomMaskedFeature(float x, float y, bool lost,
                                         float error, float quality)
    : Feature(x, y, lost, error, quality, 0)  //last variable groupId=0 by default
{
}
// ============================================================================

BottomMaskedFeature::~BottomMaskedFeature()
{
}
// ============================================================================

FeaturePtr BottomMaskedFeature::clone() const
{
    FeaturePtr copy(new BottomMaskedFeature(Feature::_x, Feature::_y, Feature::_lost, Feature::_error));
    return copy;
}
// ============================================================================

float BottomMaskedFeature::compareTo(std::pair<float, float> pt,
                 const Image* img, const Image* mask)
{
    int height = mask->getHeight();
    int width = mask->getWidth();
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

bool BottomMaskedFeature::isFeature(std::pair<float, float> pt,
               const Image* img, const Image* mask)
{
    int height = mask->getHeight();
    int width = mask->getWidth();
    bool isFt = true;
    int i0 = height - (int)pt.second;
    int j0 = (int)pt.first;
    const int ft_size = 2;
    for(int i = -ft_size; (i <= ft_size) && isFt; i++)
    {
        for(int j = -ft_size; (j <= ft_size) && isFt; j++)
        {
            int iInd = i0 + i;
            int jInd = j0 + j;
            if((iInd >= height) || (iInd < 0) || (jInd >= width) || (jInd < 0))
            {
                isFt = false;
            }else{
                Pixel px = mask->get2D(iInd, jInd);
                if(i < 0){
                    isFt = px.GRAY_VALUE !=0;
                }else{
                    isFt = px.GRAY_VALUE ==0;
                }
            }
        }
    }

    return isFt;
}
// ============================================================================

void BottomMaskedFeature::generateDescriptor(std::pair<float, float> pt,
                             const Image *img, const Image *mask)
{
    int height = mask->getHeight();
    int width = mask->getWidth();
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
                _descriptor[i + 2][j + 2] = (float)(px.GRAY_VALUE);
            }
        }
    }
}

/** ***************************************************************************
                                End of File
 ******************************************************************************/
