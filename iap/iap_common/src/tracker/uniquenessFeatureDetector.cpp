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
 * UniquenessFeatureDetector Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:10 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "uniquenessFeatureDetector.h"
#include "stFeature.h"
#include "image.h"

#include "cv.h"
#include <vector>
#include <iostream>
#include <utility>

using namespace vision;

UniquenessFeatureDetector::UniquenessFeatureDetector(int maxFeatures, 
int winSize, const std::vector<FeaturePtr> prototypeFt) : FeatureDetector(maxFeatures)
{
   _winSize = winSize;
   _prototypeFt = prototypeFt;
}
// ============================================================================

UniquenessFeatureDetector::~UniquenessFeatureDetector()
{}
// ============================================================================

std::vector<FeaturePtr> UniquenessFeatureDetector::detect(const Image *img,
                                                        const Image *mask ) const
{
    //TODO: Leak
    Image* grey = img->clone();
    std::vector<FeaturePtr> features;
    std::vector<FeaturePtr> oneTypeFts;
    std::vector<FeaturePtr> retFeatures;

    int width = grey->getWidth();
    int height = grey->getHeight();

    if(FeatureDetector::_maxFeatures > 0)
    {
        features.reserve(FeatureDetector::_maxFeatures);
        std::vector<FeaturePtr>::const_iterator prototypeFtIt = _prototypeFt.begin();
        while( prototypeFtIt != _prototypeFt.end() ){
            //find all features matching the prototype
            FeaturePtr ft = *(prototypeFtIt++);
            for(int x = 0; x < width; x++){
                for(int y = 0; y < height; y++){
                    std::pair<float, float> pt((float)x, (float)y);
                    if(ft->isFeature(pt, grey, mask)){
                        FeaturePtr newFt = ft->clone();
                        newFt->setX((float)x);
                        newFt->setY((float)y);
                        newFt->generateDescriptor(pt, grey, mask);
                        oneTypeFts.push_back(newFt);
                    }
                }
            }

            std::cout << oneTypeFts.size() << std::endl;
            //determine uniqueness of this type of feature
            std::vector<FeaturePtr>::iterator oneTypeFtsIt1 = oneTypeFts.begin();
            std::vector<FeaturePtr>::iterator oneTypeFtsIt2;
            int ftsCount = 0;
            while(oneTypeFtsIt1 != oneTypeFts.end()){
                float accumilatedDiff = 0;
                FeaturePtr ft1 = *(oneTypeFtsIt1++);
                oneTypeFtsIt2 = oneTypeFts.begin();
                while(oneTypeFtsIt2 != oneTypeFts.end()){
                    FeaturePtr ft2 = *(oneTypeFtsIt2++);
                    if((ft1->getX() <= (ft2->getX() + _winSize)) &&
                       (ft1->getX() >= (ft2->getX() - _winSize)) &&
                       (ft1->getY() <= (ft2->getY() + _winSize)) &&
                       (ft1->getY() >= (ft2->getY() - _winSize)) &&
                       (ft1 != ft2)                                 ){
                        std::pair<float, float> pt1(ft2->getX(), ft2->getY());
                        accumilatedDiff += ft1->compareTo(pt1, grey, mask);
                        ftsCount++;
                    }
                }
                ft1->setQuality(accumilatedDiff);
            }

            // now to add this type set of features to the return vector
            features.insert(features.end(), oneTypeFts.begin(), oneTypeFts.end());

            std::vector<FeaturePtr>::iterator i, j;
            for (i = features.begin(); i != features.end(); i++)
                for (j = features.begin(); j < i; j++)
                    if ((*i)->getQuality() < (*j)->getQuality())
                        std::iter_swap(i, j);

        }
    }

   // retFeatures.insert(retFeatures.end(), features.begin(), features.begin() + 10);
    return features;
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
