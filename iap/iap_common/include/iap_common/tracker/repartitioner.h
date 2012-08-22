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
#ifndef REPARTITIONER_H
#define REPARTITIONER_H

#include "trackerListener.h"
#include "cv.h"

#include <vector>

namespace vision{
/**
 *\class    Repartitioner
 *\brief    Implements TrackerListener. Given a set of partitioned features 
 * returns a new set of features in the partitions.
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:52 $
 *$Revision: 1.1 $
 */
 class Repartitioner : public TrackerListener{	
  public:
    Repartitioner();
    ~Repartitioner();

    std::vector< std::vector< FeaturePtr > > getNewPartitions();
    void setPartitions(std::vector< std::vector< FeaturePtr > > parts);

    virtual void notifyStep(const Image &img,  const std::vector<FeaturePtr> &features);
    virtual void notifyReset(const std::vector<FeaturePtr> &features);

  private:
    IplImage* lastImg;

    std::vector< CvMat > hullVector;
    std::vector< std::vector< FeaturePtr > > partitions;

    void constructHulls(const std::vector< std::vector< FeaturePtr > >&part,
                        IplImage* img);

    void displayParts(const std::vector< std::vector< FeaturePtr > > &part);
 };
};
#endif
