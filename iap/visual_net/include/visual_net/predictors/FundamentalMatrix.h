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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace VisualGraph
{

/**
 \class    FundamentalMatrix
 \brief    A predictor.
 Computes 3D motion hypotheses and clusters
 features into motion models
 **/

class FundamentalMatrix : public Predictor
{
private:
  int numHypotheses;
  int numTrialsPerHypothesis;

public:
  FundamentalMatrix(int numHypotheses, int numTrialsPerHypothesis);
  virtual
  ~FundamentalMatrix(void);

  virtual void
  compute();

private:
  std::vector<int>
  pickRandomSample(int sampleSize, int low, int high);
  boost::numeric::ublas::matrix<double>
  computeModel(boost::numeric::ublas::matrix<double> &frame1, boost::numeric::ublas::matrix<double> &frame2);
  void
  computeSVD(boost::numeric::ublas::matrix<double> &A, boost::numeric::ublas::matrix<double> &U,
             boost::numeric::ublas::matrix<double> &S, boost::numeric::ublas::matrix<double> &V);
  boost::numeric::ublas::vector<double>
  fitsModel(boost::numeric::ublas::matrix<double> &F, boost::numeric::ublas::matrix<double> &ptsFrame1,
            boost::numeric::ublas::matrix<double> &ptsFrame2);
  bool
  isDegenerateModel(boost::numeric::ublas::matrix<double> &frame1, boost::numeric::ublas::matrix<double> &frame2);

  template<class T>
    void
    printMatrix(boost::numeric::ublas::matrix<T> &M);

  boost::numeric::ublas::matrix<double>
  pDistJaccard(boost::numeric::ublas::matrix<int> x); //Calculates pairwise jaccard distance
  //Authors: R.Toldo A.Fusiello, University of Verona
  //("Robust Multiple Structures Estimation with J-linkage")
  //inline
  double
  jaccard(boost::numeric::ublas::matrix<int> &logi, boost::numeric::ublas::vector<int> &obsLogi, int col, int row,
          int nMod);
  boost::numeric::ublas::matrix<double>
  linkageIntersect(boost::numeric::ublas::matrix<double> &Y, boost::numeric::ublas::matrix<int> &totdbin);

  boost::numeric::ublas::vector<int>
  cluster(boost::numeric::ublas::matrix<double> Z, double cutoff); //uses distance as the criterion for forming clusters. Each node's height in the tree
  //represents the distance between the two subnodes merged at that node. All leaves below
  //any node whose height is less than C are grouped into a cluster
  //(a singleton if the node itself is a leaf)

  int
  any(boost::numeric::ublas::vector<int> in); //any(A) returns logical 1 (true) if any of the elements of A is a nonzero number
  //or is logical 1 (true), and returns logical 0 (false) if all the elements are zero

  void
  clusterPoints(std::vector<boost::numeric::ublas::vector<double> > &hypothesesScore, double inliersThreshold,
                boost::numeric::ublas::matrix<double> &Y, boost::numeric::ublas::matrix<double> &Z,
                boost::numeric::ublas::vector<int> &T, boost::numeric::ublas::matrix<int> &totdbin);

};
}
;
