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

#include <vector>
#include <string>
#include "Vertex.h"
#include "Edge.h"
#include "Predictor.h"
#include "CameraModel.h"
#include "feature.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "ekf.h"

namespace VisualGraph
{

  class StructureFromMotion : public Predictor
  {
    //things to do later
    //have the clustring mark vertices according to the assigned cluster number
    //implement a function in graph that returns vector of clusters (each cluster will be a vector<vector<Feature> > for time frame, and then all features in that frame
    //also add to vertices the estimated 3D structure of each feature (in addition to the current 2D data)

    //but for now, just use this class externally


    //pick a sample of size 8 (try...)
    //compute ukf2runs for the sample
    //	go over all other features and and the ukf class to compute error values for them
    //use the output vector of error to pick the error value (i can do max, maybe also mean is good to cover for one time errors
    //														but probably max is more restrictive which is better...)
    //so at the end we should get the largest consistent cluster...

  private:
    int numHypotheses;
    vision::CameraModel cammodel;

  public:
    StructureFromMotion(int frame1, int frame2, int numHypotheses,
        vision::CameraModel &cammodel);
    virtual
    ~StructureFromMotion(void);

    virtual std::vector<std::vector<vision::FeaturePtr> >
    compute(std::vector<std::vector<vision::FeaturePtr> > cluster);
    virtual void
    compute()
    {
      throw("only here temporary, should replace the function above...");
    }
    ;

  private:
    std::vector<int>
    pickRandomSample(int sampleSize, int low, int high);
    SFM::EKF
    computeModel(std::vector<std::vector<vision::FeaturePtr> > features);
    boost::numeric::ublas::vector<double>
    fitsModel(SFM::EKF &ukf,
        std::vector<std::vector<vision::FeaturePtr> > &testPts);

    boost::numeric::ublas::matrix<double>
    pDistJaccard(boost::numeric::ublas::matrix<int> x); //Calculates pairwise jaccard distance
    //Authors: R.Toldo A.Fusiello, University of Verona
    //("Robust Multiple Structures Estimation with J-linkage")

    double
        jaccard(boost::numeric::ublas::matrix<int> &logi,
            boost::numeric::ublas::vector<int> &obsLogi, int col, int row,
            int nMod);
    boost::numeric::ublas::matrix<double>
    linkageIntersect(boost::numeric::ublas::matrix<double> &Y,
        boost::numeric::ublas::matrix<int> &totdbin);

    boost::numeric::ublas::vector<int>
    cluster(boost::numeric::ublas::matrix<double> Z, double cutoff); //uses distance as the criterion for forming clusters. Each node's height in the tree
    //represents the distance between the two subnodes merged at that node. All leaves below
    //any node whose height is less than C are grouped into a cluster
    //(a singleton if the node itself is a leaf)

    int
    any(boost::numeric::ublas::vector<int> in); //any(A) returns logical 1 (true) if any of the elements of A is a nonzero number
    //or is logical 1 (true), and returns logical 0 (false) if all the elements are zero
    void
    clusterPoints(boost::numeric::ublas::matrix<double> &totd,
        double inliersThreshold, boost::numeric::ublas::matrix<double> &Y,
        boost::numeric::ublas::matrix<double> &Z,
        boost::numeric::ublas::vector<int> &T,
        boost::numeric::ublas::matrix<int> &totdbin);

    void
    scaleInputs(std::vector<std::vector<vision::FeaturePtr> > &input);

  };
}
;
