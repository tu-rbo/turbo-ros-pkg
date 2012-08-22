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
#ifndef MANUAL_DETECTOR_H
#define MANUAL_DETECTOR_H

#include "featureDetector.h"
#include "highgui.h"


namespace vision{
/**
 *\class    ManualDetector
 *\brief    
 * 
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:51 $
 *$Revision: 1.1 $
 */
 class ManualDetector : public FeatureDetector{
  public:

    /**
     *
     */
    ManualDetector(const FeatureDetector* fd);

    /**
     * default destructor
     */
    virtual ~ManualDetector();

    /**
     *
     */
    virtual std::vector<FeaturePtr> detect(const Image *img,
                                         const Image *mask) const;
   private:
    const FeatureDetector* _fd;
 };
};
#endif
