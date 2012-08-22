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
 * Accumilator Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:00:57 $
 * $Revision: 1.1 $
 *****************************************************************************/
#include "accumulator.h"
#include "feature.h"

#include <iostream>
using namespace vision;


Accumulator::Accumulator()
{
}
// ============================================================================

Accumulator::~Accumulator()
{

}
// ============================================================================

void Accumulator::notifyReset(const std::vector<FeaturePtr> &features)
{
    if(!features.empty()){
        std::vector<FeaturePtr> featuresCopy;

        std::vector<FeaturePtr>::const_iterator featuresIt;
        for(featuresIt = features.begin(); featuresIt != features.end();
            featuresIt++)
        {
            FeaturePtr feat = *(featuresIt);
            featuresCopy.push_back( feat->clone() );
        }
        featureHistory.push_back(featuresCopy);
    }
}
// ============================================================================
void Accumulator::
notifyStep(const Image &img,  const std::vector<FeaturePtr> &features)
{
    if(!features.empty())
    {
        std::vector<FeaturePtr> featuresCopy;

        std::vector<FeaturePtr>::const_iterator featuresIt;
        for(featuresIt = features.begin(); featuresIt != features.end();
        featuresIt++)
        {
            FeaturePtr feat = *(featuresIt);
            featuresCopy.push_back( feat->clone() );
        }
        featureHistory.push_back(featuresCopy);
    }
}
// ============================================================================

std::vector< std::vector< FeaturePtr > > Accumulator::getGoodFeatureHistory()
{
    if(featureHistory.empty()){
        throw std::string(
        "Can not get history: features have not been initialized");
    }

    // the output vector we are building
    std::vector< std::vector< FeaturePtr > > output;

    // holds the tempotary feature list for inspecting
    std::vector< FeaturePtr > temp;
    std::vector<FeaturePtr>::const_iterator tempIt;

    // holds the latest features; here we know wich features to remove that were
    // lost
    std::vector< FeaturePtr > latest = featureHistory.back();
    std::vector<FeaturePtr>::const_iterator latestIt;

    //iterator of history vector
    std::vector< std::vector< FeaturePtr > >::const_iterator histIt;
    histIt = featureHistory.begin();
    while( histIt != featureHistory.end() )
    {
        std::vector< FeaturePtr > outList;

        temp     = (std::vector< FeaturePtr >) *(histIt);
        tempIt   = temp.begin();
        latestIt = latest.begin();

        while(tempIt != temp.end())
        {
            FeaturePtr lostF = *(latestIt);
            FeaturePtr tempF = *(tempIt);

            if(!lostF->isLost())
            {
                outList.push_back(tempF);
            }

            tempIt++;
            latestIt++;
        }
        output.push_back(outList);	
        histIt++;
    }

    return output;
}
// ============================================================================

std::vector< std::vector< FeaturePtr > > Accumulator::getAllFeatureHistory()
{
    if(featureHistory.empty()){
        throw std::string(
            "Can not get history: features have not been initialized");
    }

    return featureHistory;
}

std::vector<FeaturePtr> Accumulator::getLastTrackedFeatures(){
	if(featureHistory.empty()){
        throw std::string(
            "Can not get history: features have not been initialized");
    }

	return featureHistory.back();
}

std::vector<FeaturePtr> Accumulator::getLastGoodTrackedFeatures(){
	if(featureHistory.empty()){
        throw std::string(
            "Can not get history: features have not been initialized");
    }

	std::vector<FeaturePtr> retVal;
	std::vector<FeaturePtr>::iterator featHistIt;

	for(featHistIt = featureHistory.back().begin(); featHistIt < featureHistory.back().end(); featHistIt++){
		if(!(*featHistIt)->isLost()){
			retVal.push_back((*featHistIt));
		}
	}

	return retVal;
}
/** ***************************************************************************
                                End of File
 ******************************************************************************/



