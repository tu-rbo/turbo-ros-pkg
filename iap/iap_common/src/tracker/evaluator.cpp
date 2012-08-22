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
 * Evaluator Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:01:02 $
 * $Revision: 1.1 $
 *****************************************************************************/

#include "evaluator.h"
#include "highgui.h"
#include "feature.h"

#include <utility>
#include <string>

using namespace vision;

Evaluator::Evaluator(std::string datasetName)
{
    dataName = datasetName + ".dat";
    imageName = datasetName + ".png";
    configName = datasetName + ".gnuplot";

    cvNamedWindow( "Standard Dev", 0 );

    std::string command = "rm "+dataName;
    system(command.c_str());

    dataOut.open(dataName.c_str(), std::fstream::app);
    dataOut << "#Frame Number, StdDevOfDeltas\n";
    dataOut.close();

    command = "rm "+configName;
    system(command.c_str());
    dataOut.open(configName.c_str(), std::fstream::app);
    dataOut << "set terminal png \n"
    << "set output \""<< imageName <<"\" \n"
    << "set title \"Standard Deveation of Relative Distance Delta vs."
    << "Frame Number\" \n"
    << "set xlabel \"Frame Number\" \n"
    << "set ylabel \"Standard Deveation of Relative Distance Delta\" \n"
    << "plot  \""<< dataName <<"\" using 1:2 title \'Standard Deveation of"
    << " Relative Distance Delta vs. Frame Number\' with lines";
    dataOut.close();

    frame = 0;
    feedbackImage = 0;
}
// ============================================================================

Evaluator::~Evaluator()
{
    cvDestroyWindow("Standard Dev");
}
// ============================================================================

void Evaluator::notifyReset(const std::vector<FeaturePtr> &features)
{
    std::string command = "rm "+dataName;
    system(command.c_str());
    dataOut.open(dataName.c_str(), std::fstream::app);
    dataOut << "#Frame Number, StdDevOfDeltas\n";
    dataOut.close();

    lastDist.clear();
    //distHist.clear();

    frame = 0;
}
// ============================================================================

void Evaluator::
notifyStep(const Image &img,  const std::vector<FeaturePtr> &features)
{
    double x1, x2, y1, y2, dist, stdev;

    double sum = 0;
    double count = 0;

    std::vector< std::pair<double, bool> > thisDist;

    std::vector<FeaturePtr>::const_iterator firstI = features.begin();
    std::vector<FeaturePtr>::const_iterator secondI;
    while(firstI != features.end())
    {
        FeaturePtr f1 = *(firstI);
        x1 = f1->getX();
        y1 = f1->getY();
        bool lost = f1->isLost();

        secondI = features.begin();
        while(secondI != features.end())
        {
            FeaturePtr f2 = *(secondI);
            x2 = f2->getX();
            y2 = f2->getY();
            lost = lost || f2->isLost();

            dist = sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) );
            std::pair<double, bool> p(dist, lost);
            thisDist.push_back(p);

            secondI++;
        }
        firstI++;
    }

    if(!lastDist.empty())
    {
        std::vector< std::pair<double, bool> >::const_iterator lastI =
                                                            lastDist.begin();

        std::vector< std::pair<double, bool> >::const_iterator thisI =
                                                            thisDist.begin();
        while(thisI != thisDist.end())
        {
            std::pair<double, bool> d1 = *(lastI);
            std::pair<double, bool> d2 = *(thisI);
            double diff = d2.first - d1.first;

            if(!d1.second && !d2.second)
            {
                sum = sum + diff;
                count++;
            }

            thisI++;
            lastI++;
        }

    //------------------
        stdev = sqrt( (( pow(sum,2.0)) -
                    (( 1.0/count) * (pow(sum,2.0))))/ (count-1.0));

        if(feedbackImage)
        {
            cvReleaseImage(&feedbackImage);
        }

        frame++;

        dataOut.open(dataName.c_str(), std::fstream::app);
        dataOut << frame << "\t" << stdev << "\n";
        dataOut.close();

        std::string comand = "gnuplot "+configName;
        system(comand.c_str());	// call gnuplot to plot feeback so far
        IplImage *feedbackImage = cvLoadImage(imageName.c_str());
    /*
        CvFont font;
        char buf[500]; 
        sprintf(buf, "Profile Index: %f", profileIndex);	
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.4,0.4,0,1);
        cvPutText (returnImage, buf, cvPoint(10,15), &font, cvScalar(255));
    */

        cvShowImage( "Standard Dev", feedbackImage );
        //cvWaitKey(10);
    }
    lastDist = thisDist;

}
/** ***************************************************************************
                                End of File
 ******************************************************************************/
