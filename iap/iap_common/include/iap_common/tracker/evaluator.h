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
#ifndef EVALUATOR_H
#define EVALUATOR_H


#include "trackerListener.h"

#include <vector>
#include <fstream>
#include "cv.h"

namespace vision{
/**
 *\class    Evaluator
 *\brief    Implements TrackerListener. Calculates and displays the quality 
 * of tracking in real time using gnuplot.
 *$Author: dubik $
 *$Date: 2009/03/10 15:00:47 $
 *$Revision: 1.1 $
 */
 class Evaluator : public TrackerListener{
  public:
    /**
     * creates the Evaluator and initializes the streams for generating plot
     * data
     * 
     * @param datasetName - the name of the data
     */
    Evaluator(std::string datasetName);

     /**
      * default destructor
      */
    ~Evaluator();

    /**
     * measures the diference of relative distances from last frame, appends
     * the standard deviation of these differences to a gnuplot datafile,
     * generates a gnuplot, and displays the plot.
     * 
     * @param img - The latest image (not used)
     * @param features - the set of features that get evaluated
     */
   virtual void notifyStep(const Image &img,  const std::vector<FeaturePtr> &features);

   /**
    * resets the plot
    * 
    * @param features - the set of features that get evaluated
    */
   virtual void notifyReset(const std::vector<FeaturePtr> &features);

  private:
    std::ofstream dataOut;
    std::ofstream configOut;
    int frame;
    IplImage *feedbackImage;

    std::vector< std::pair<double, bool> > lastDist;
    std::vector< std::vector< double > > diffHist;
    std::string imageName;
    std::string dataName;
    std::string configName;	
 };
};
#endif
