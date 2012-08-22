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
#include "Graph.h"
#include <iostream>
#include "FundamentalMatrix.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;
using namespace boost::numeric::ublas;

namespace VisualGraph
{

FundamentalMatrix::FundamentalMatrix(int numHypotheses, int numTrialsPerHypothesis) :
  Predictor(string("FundamentalMatrix"))
{
  this->numHypotheses = numHypotheses;
  this->numTrialsPerHypothesis = numTrialsPerHypothesis;
}

FundamentalMatrix::~FundamentalMatrix(void)
{

}

void FundamentalMatrix::compute()
{
  //generate two boost 2D vectors ([x,y] in frame1, [x,y] in frame2)
  matrix<double> ptsFrame1(numVertices, 2), ptsFrame2(numVertices, 2);
  for (int i = 0; i < numVertices; i++)
  {
    ptsFrame1(i, 0) = (vList[i]->getFeatureData(frame1)->getX() - 320) / 320;
    ptsFrame1(i, 1) = (vList[i]->getFeatureData(frame1)->getY() - 240) / 240;
    ptsFrame2(i, 0) = (vList[i]->getFeatureData(frame2)->getX() - 320) / 320;
    ptsFrame2(i, 1) = (vList[i]->getFeatureData(frame2)->getY() - 240) / 240;
  }

  std::vector<matrix<double> > hypothesesModel; //a vector of generated hypotheses (i.e. model/Fundamental matrix)
  std::vector<boost::numeric::ublas::vector<double> > hypothesesScore; //a vector of scores for the generated hypotheses
  matrix<double> samplePts1(8, 2), samplePts2(8, 2);

  for (int hyp = 0; hyp < numHypotheses; hyp++)
  {
    for (int trial = 0; trial < numTrialsPerHypothesis; trial++)
    {
      if (hyp % 1000 == 0)
        cout << (double)hyp / (double)numHypotheses << endl;

      //pickRandomSample(8,1,n) = returns a vector of 8 number in the range of 1...n where n=numPts
      std::vector<int> sample = pickRandomSample(8, 0, numVertices - 1);

      //generate a sample of 8pts
      for (int j = 0; j < 8; j++)
      {
        samplePts1(j, 0) = ptsFrame1(sample[j], 0);
        samplePts1(j, 1) = ptsFrame1(sample[j], 1);
        samplePts2(j, 0) = ptsFrame2(sample[j], 0);
        samplePts2(j, 1) = ptsFrame2(sample[j], 1);
      }

      if (!isDegenerateModel(samplePts1, samplePts2))
      {
        //a reliable model can be computed
        matrix<double> model = computeModel(samplePts1, samplePts2);
        boost::numeric::ublas::vector<double> score = fitsModel(model, ptsFrame1, ptsFrame2);

        //printMatrix(model);
        //printMatrix(score);
        //hypothesesModel.push_back(model);  //no need to store the model for now
        hypothesesScore.push_back(score);
        break;
      }
      else
      {
        if (trial == numTrialsPerHypothesis - 1)
        {
          printMatrix(samplePts1);
          printMatrix(samplePts2);
          isDegenerateModel(samplePts1, samplePts2);
          throw("Unable to select a nondegenerate data set!");
        }
      }
    }
  }

  cout << "done with the loop" << endl;
  //now we have a vector of hypotheses and we need to perform clustering
  //cluster points -- needs the scored hyp and some inlier th parameter to return the clustering (depends on c++/mex files and matlab's cluster function...
  boost::numeric::ublas::matrix<double> Y;
  boost::numeric::ublas::matrix<double> Z;
  boost::numeric::ublas::vector<int> T;
  boost::numeric::ublas::matrix<int> totdbin;
  double inliersThreshold = 0.05;

  clusterPoints(hypothesesScore, inliersThreshold, Y, Z, T, totdbin);

  cout << T << endl;

  //now based on totd we have clusters, and we can set the edges of the graph
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = i + 1; j < numVertices; j++)
    {
      if (T(i) == T(j))
      {
        adjMatrix[i][j]->setCapacity(predictorName, 1.0);
        adjMatrix[j][i]->setCapacity(predictorName, 1.0);
      }
    }
  }
}

template<class T>
  void FundamentalMatrix::printMatrix(boost::numeric::ublas::matrix<T> &M)
  {
    for (unsigned int i = 0; i < M.size1(); i++)
    {
      for (unsigned int j = 0; j < M.size2(); j++)
      {
        cout << M(i, j) << " ";
      }
      cout << endl;
    }
    cout << "====================" << endl;
  }

std::vector<int> FundamentalMatrix::pickRandomSample(int sampleSize, int low, int high)
{
  //srand((unsigned)time(0));
  std::vector<int> sample;
  int range = (high - low) + 1;
  int counter = 0;
  while (counter < sampleSize)
  {
    int val = low + int(range * rand() / (RAND_MAX + 1.0));
    bool found = false;
    for (std::vector<int>::iterator it = sample.begin(); it != sample.end(); it++)
    {
      if (*it == val)
      {
        found = true;
      }
    }
    if (!found)
    {
      sample.push_back(val);
      counter++;
    }
  }
  return sample;
}

matrix<double> FundamentalMatrix::computeModel(matrix<double> &frame1, matrix<double> &frame2)
{
  int numPts = frame1.size1();
  matrix<double> hFrame1(numPts, 3);
  matrix<double> hFrame2(numPts, 3);

  for (int i = 0; i < numPts; ++i)
  {
    hFrame1(i, 0) = frame1(i, 0);
    hFrame1(i, 1) = frame1(i, 1);
    hFrame1(i, 2) = 1;
    hFrame2(i, 0) = frame2(i, 0);
    hFrame2(i, 1) = frame2(i, 1);
    hFrame2(i, 2) = 1;
  }

  //compute the centroid of the points
  matrix<double> centroid1(1, 2);
  matrix<double> centroid2(1, 2);
  centroid1(0, 0) = 0;
  centroid1(0, 1) = 0;
  centroid2(0, 0) = 0;
  centroid2(0, 1) = 0;
  for (int i = 0; i < numPts; ++i)
  {
    centroid1(0, 0) += hFrame1(i, 0) / numPts;
    centroid1(0, 1) += hFrame1(i, 1) / numPts;
    centroid2(0, 0) += hFrame2(i, 0) / numPts;
    centroid2(0, 1) += hFrame2(i, 1) / numPts;
  }

  //shift the origin of the points to the centroid
  for (int i = 0; i < numPts; ++i)
  {
    hFrame1(i, 0) -= centroid1(0, 0);
    hFrame1(i, 1) -= centroid1(0, 1);
    hFrame2(i, 0) -= centroid2(0, 0);
    hFrame2(i, 1) -= centroid2(0, 1);
  }

  //normalize the points so that the average distance from the origin is equal to sqrt(2).
  double averagedist1 = 0;
  double averagedist2 = 0;
  for (int i = 0; i < numPts; ++i)
  {
    averagedist1 += sqrt(hFrame1(i, 0) * hFrame1(i, 0) + hFrame1(i, 1) * hFrame1(i, 1)) / numPts;
    averagedist2 += sqrt(hFrame2(i, 0) * hFrame2(i, 0) + hFrame2(i, 1) * hFrame2(i, 1)) / numPts;
  }

  double scale1 = sqrt(2.0) / averagedist1;
  double scale2 = sqrt(2.0) / averagedist2;

  hFrame1 = scale1 * hFrame1;
  hFrame2 = scale2 * hFrame2;

  //similarity transform 1
  matrix<double> T1(3, 3);
  T1(0, 0) = scale1;
  T1(0, 1) = 0;
  T1(0, 2) = -scale1 * centroid1(0, 0);
  T1(1, 0) = 0;
  T1(1, 1) = scale1;
  T1(1, 2) = -scale1 * centroid1(0, 1);
  T1(2, 0) = 0;
  T1(2, 1) = 0;
  T1(2, 2) = 1;

  //similarity transform 2
  matrix<double> T2(3, 3);
  T2(0, 0) = scale2;
  T2(0, 1) = 0;
  T2(0, 2) = -scale2 * centroid2(0, 0);
  T2(1, 0) = 0;
  T2(1, 1) = scale2;
  T2(1, 2) = -scale2 * centroid2(0, 1);
  T2(2, 0) = 0;
  T2(2, 1) = 0;
  T2(2, 2) = 1;

  //compute the Fundamental Matrix
  matrix<double> A(numPts, 9);

  for (int i = 0; i < numPts; i++)
  {
    A(i, 0) = hFrame2(i, 0) * hFrame1(i, 0);
    A(i, 1) = hFrame2(i, 0) * hFrame1(i, 1);
    A(i, 2) = hFrame2(i, 0);
    A(i, 3) = hFrame2(i, 1) * hFrame1(i, 0);
    A(i, 4) = hFrame2(i, 1) * hFrame1(i, 1);
    A(i, 5) = hFrame2(i, 1);
    A(i, 6) = hFrame1(i, 0);
    A(i, 7) = hFrame1(i, 1);
    A(i, 8) = 1;
  }

  //A is of size numPtsX9
  matrix<double> U(numPts, numPts);
  matrix<double> S(numPts, 9);
  matrix<double> V(9, 9);
  computeSVD(A, U, S, V);

  matrix<double> F(3, 3);
  F(0, 0) = V(0, 8);
  F(0, 1) = V(1, 8);
  F(0, 2) = V(2, 8);
  F(1, 0) = V(3, 8);
  F(1, 1) = V(4, 8);
  F(1, 2) = V(5, 8);
  F(2, 0) = V(6, 8);
  F(2, 1) = V(7, 8);
  F(2, 2) = V(8, 8);

  //Rank 2 constraint enforcement
  matrix<double> u(3, 3);
  matrix<double> s(3, 3);
  matrix<double> v(3, 3);
  computeSVD(F, u, s, v);
  s(2, 2) = 0;

  //F=u*s*vt
  matrix<double> tmp = prod(u, s);
  F = prod(tmp, trans(v));

  //denormalization (F = T2'*F*T1)
  tmp = prod(trans(T2), F);
  F = prod(tmp, T1);

  return F;
}

void FundamentalMatrix::computeSVD(matrix<double> &A, matrix<double> &U, matrix<double> &S, matrix<double> &V)
{
  //decompose a matrix A into its Singular Value Decomposition such that A= U*S*V'
  int n = A.size1();
  int m = A.size2();

  CvMat* a = cvCreateMat(n, m, CV_32FC1);
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      cvmSet(a, i, j, A(i, j));
    }
  }

  CvMat* u = cvCreateMat(n, n, CV_32FC1);
  CvMat* ut = cvCreateMat(n, n, CV_32FC1);
  CvMat* d = cvCreateMat(n, m, CV_32FC1);
  CvMat* v = cvCreateMat(m, m, CV_32FC1);
  CvMat* vt = cvCreateMat(m, m, CV_32FC1);
  cvSVD(a, d, u, v, CV_SVD_U_T | CV_SVD_V_T); // A = U D V^T

  cvTranspose(u, ut);
  cvTranspose(v, vt);

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      U(i, j) = cvmGet(ut, i, j);
    }
  }
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      S(i, j) = cvmGet(d, i, j);
    }
  }

  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < m; j++)
    {
      V(i, j) = cvmGet(vt, i, j);
    }
  }

  cvReleaseMat(&a);
  cvReleaseMat(&u);
  cvReleaseMat(&ut);
  cvReleaseMat(&d);
  cvReleaseMat(&v);
  cvReleaseMat(&vt);
}

boost::numeric::ublas::vector<double> FundamentalMatrix::fitsModel(matrix<double> &F, matrix<double> &ptsFrame1,
                                                                   matrix<double> &ptsFrame2)
{
  //computes the distance of all points from the model
  boost::numeric::ublas::vector<double> dist(numVertices, 1);
  for (int i = 0; i < numVertices; i++)
  {
    double x1 = ptsFrame1(i, 0);
    double y1 = ptsFrame1(i, 1);
    double x2 = ptsFrame2(i, 0);
    double y2 = ptsFrame2(i, 1);

    double Ft1 = F(0, 0) * x2 + F(1, 0) * y2 + F(2, 0);
    double Ft2 = F(1, 0) * x2 + F(1, 1) * y2 + F(1, 2);
    double F1 = F(0, 0) * x1 + F(0, 1) * y1 + F(0, 2);
    double F2 = F(1, 0) * x1 + F(1, 1) * y1 + F(1, 2);

    matrix<double> a(4, 1);
    a(0, 0) = x1;
    a(1, 0) = y1;
    a(2, 0) = x2;
    a(3, 0) = y2;

    matrix<double> b(1, 3);
    b(0, 0) = x2;
    b(0, 1) = y2;
    b(0, 2) = 1;

    matrix<double> c(3, 1);
    c(0, 0) = x1;
    c(1, 0) = y1;
    c(2, 0) = 1;

    matrix<double> d(4, 1);
    d(0, 0) = Ft1;
    d(1, 0) = Ft2;
    d(2, 0) = F1;
    d(3, 0) = F2;

    //double Est=[x1 y1 x2 y2]' - (([x2 y2 1]*F*[x1 y1 1]') / ( F1^2 + F2^2 + Ft1^2 + Ft2^2)) * [Ft1 Ft2 F1 F2]';
    //multiple view geometry book
    matrix<double> tmp = prod(b, F);
    matrix<double> e = prod(tmp, c); //now e is actually just a number (1X1 matrix)
    double e1 = e(0, 0);
    matrix<double> Est = a - e1 / (F1 * F1 + F2 * F2 + Ft1 * Ft1 + Ft2 * Ft2) * d;

    double dx1 = fabs(Est(0, 0) - x1) * 320;
    double dy1 = fabs(Est(1, 0) - y1) * 240;
    double dx2 = fabs(Est(2, 0) - x2) * 320;
    double dy2 = fabs(Est(3, 0) - y2) * 240;
    double err1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double err2 = sqrt(dx2 * dx2 + dy2 * dy2);
    double err;
    if (err1 > err2)
    {
      err = err1;
    }
    else
    {
      err = err2;
    }
    if (err > 50)
    {
      dist(i) = 1.0;
    }
    else
    {
      dist(i) = err / 50;
    }
  }
  return dist;
}

bool FundamentalMatrix::isDegenerateModel(matrix<double> &frame1, matrix<double> &frame2)
{
  //returns true if a reliable model cannot be generated from the given 8 pts
  double eps = pow(2.0, -52);

  for (int i = 0; i < 8; i++)
  {
    for (int j = i + 1; j < 8; j++)
    {
      double dist11 = sqrt(
                           (frame1(i, 0) - frame1(j, 0)) * (frame1(i, 0) - frame1(j, 0))
                               + (frame1(i, 1) - frame1(j, 1)) * (frame1(i, 1) - frame1(j, 1)));
      double dist22 = sqrt(
                           (frame2(i, 0) - frame2(j, 0)) * (frame2(i, 0) - frame2(j, 0))
                               + (frame2(i, 1) - frame2(j, 1)) * (frame2(i, 1) - frame2(j, 1)));
      if (dist11 < eps || dist22 < eps)
        return true;
    }
  }

  for (int i = 0; i < 8; i++)
  {
    double dist12 = sqrt(
                         (frame1(i, 0) - frame2(i, 0)) * (frame1(i, 0) - frame2(i, 0)) + (frame1(i, 1) - frame2(i, 1))
                             * (frame1(i, 1) - frame2(i, 1)));
    if (dist12 < eps)
      return true;
  }

  return false;
}

void FundamentalMatrix::clusterPoints(std::vector<boost::numeric::ublas::vector<double> > &hypothesesScore,
                                      double inliersThreshold, boost::numeric::ublas::matrix<double> &Y,
                                      boost::numeric::ublas::matrix<double> &Z, boost::numeric::ublas::vector<int> &T,
                                      boost::numeric::ublas::matrix<int> &totdbin)
{

  //Find every entry below the threshold
  totdbin.resize(numVertices, numHypotheses);

  for (unsigned int i = 0; i < totdbin.size1(); i++)
  {
    for (unsigned int j = 0; j < totdbin.size2(); j++)
    {
      if (hypothesesScore[j](i) < inliersThreshold)
        totdbin(i, j) = 1;
      else
        totdbin(i, j) = 0;
    }
  }
  //std::cout << "inliersThreshold = " << inliersThreshold << endl;
  //std::cout << "totdbin = " << totd << endl;

  //Calculate the jaccard distance
  Y = pDistJaccard(trans(totdbin));

  // Create hierarchical cluster tree Z using the J-Linkage algorith,
  //	Z = linkageIntersect(Y, totdbin)
  Z = linkageIntersect(Y, totdbin);

  double eps = pow(2.0, -52);
  double cutoff = 1.0 - (1.0 / (double)totdbin.size1()) + eps;

  T.resize(Z.size1() + 1);
  T = cluster(Z, cutoff);
}

int FundamentalMatrix::any(boost::numeric::ublas::vector<int> in)
{
  for (unsigned int i = 0; i < in.size(); i++)
  {
    if (in(i) > 0)
      return 1; //true
  }
  return 0; //false
}

boost::numeric::ublas::vector<int> FundamentalMatrix::cluster(boost::numeric::ublas::matrix<double> Z, double cutoff)
{
  unsigned int n = Z.size1() + 1;

  boost::numeric::ublas::vector<int> T(n);
  for (unsigned int i = 0; i < T.size(); i++)
  {
    T(i) = 0;
  }

  boost::numeric::ublas::vector<double> crit(Z.size1());
  for (unsigned int i = 0; i < Z.size1(); i++)
  {
    crit(i) = Z(i, 2);
  }//distance criterion

  /*********************************************/
  //CHECKCUT ALGORITHM STARTS HERE
  //conn = checkcut(Z, cutoff(j), crit); //cut the tree at a specified point
  /*********************************************/
  std::cout << "checkcut algorithm... ";
  //See which nodes are below the cutoff, disconnect those that aren't
  /************/
  boost::numeric::ublas::vector<int> conn(crit.size());
  //conn = (crit <= cutoff);  % these are still connected
  for (unsigned int i = 0; i < crit.size(); i++)
  {
    if (crit(i) < cutoff)
      conn(i) = 1;
    else
      conn(i) = 0;
  }
  /************/
  //We may still disconnect a node unless all non-leaf children are
  //below the cutoff, and grand-children, and so on
  boost::numeric::ublas::vector<int> todo(Z.size1());
  for (unsigned int i = 0; i < Z.size1(); todo(i) = conn(i) && (Z(i, 1) > n || Z(i, 2) > n), i++)
    ;

  while (any(todo))
  {
    /************/
    //rows = find(todo);
    int numberOfNonZeroElements = 0;
    for (unsigned int i = 0; i < todo.size(); i++)
    {
      if (todo(i) > 0)
        numberOfNonZeroElements++;
    }
    boost::numeric::ublas::vector<int> rows(numberOfNonZeroElements);
    int index = 0;
    for (unsigned int i = 0; i < todo.size(); i++)
    {
      if (todo(i) > 0)
        rows(index++) = i;
    }
    /************/

    //See if each child is done, or if it requires disconnecting its parent
    //cdone = true(length(rows),2);
    matrix<int> cdone(rows.size(), 2);
    for (unsigned int i = 0; i < cdone.size1(); i++)
    {
      for (unsigned int j = 0; j < cdone.size2(); j++)
      {
        cdone(i, j) = 1;
      }
    }
    /************/
    //for j=1:2     //1=left child, 2=right child
    for (unsigned int j = 0; j < 2; j++)
    {

      /************/
      //crows = X(rows,j);
      boost::numeric::ublas::vector<double> crows(rows.size());
      for (unsigned int i = 0; i < crows.size(); i++)
      {
        crows(i) = Z(rows(i), j);
      }
      /************/

      //t = (crows>n);
      boost::numeric::ublas::vector<int> t(crows.size());
      for (unsigned int i = 0; i < crows.size(); i++)
      {
        if (crows(i) > n)
          t(i) = 1;
        else
          t(i) = 0;
      }
      /************/
      //if any(t)
      //child = crows(t)-n;
      //cdone(t,j) = ~todo(child);
      //conn(rows(t)) = conn(rows(t)) & conn(child);
      //end
      if (any(t))
      {
        //child size is equal to number of ones in t
        boost::numeric::ublas::vector<int> child((int)count(t.begin(), t.end(), 1));
        int indexC = 0;
        for (unsigned int i = 0; i < t.size(); i++)
        {
          if (t(i) > 0)
            child(indexC++) = crows(i) - n;
        }
        indexC = 0;

        for (unsigned int tc = 0; tc < t.size(); tc++)
        {
          if (t(tc) > 0)
            cdone(tc, j) = !(todo(child(indexC++)));
        }
        indexC = 0;
        for (unsigned int tc = 0; tc < t.size(); tc++)
        {
          if (t(tc) > 0)
            conn(rows(tc)) = conn(rows(tc)) && conn(child(indexC++));
        }
      }//if any(t)
      /************/
    }// for j= 1 2

    // todo(rows(cdone(:,1) & cdone(:,2))) = 0;

    for (unsigned int cc = 0; cc < cdone.size1(); cc++)
    {
      //printf("cc=%d c0=%d c1=%d r(c0&c1)=%d\r\n",cc,cdone(cc,0),cdone(cc,1),rows(cdone(cc,0) and cdone(cc,1)) );
      if ((cdone(cc, 0) && cdone(cc, 1)) > (rows.size() - 1))
        todo(rows(rows.size() - 1)) = 0;
      else
        todo(rows(cdone(cc, 0) && cdone(cc, 1))) = 0;
    }
  }//end while any(todo)
  std::cout << "Done" << std::endl;
  /*********************************************/
  //LABELTREE ALGORITHM STARTS HERE
  //T(:) = labeltree(Z, conn);
  /*********************************************/
  std::cout << "labeltree algorithm... ";
  //n = size(X,1);
  //nleaves = n+1;
  //T = ones(n+1,1);
  n = Z.size1();
  int nleaves = n + 1;

  for (unsigned int tc = 0; tc < T.size(); tc++)
  {
    T(tc) = 1;
  }
  /************/
  //todo = true(n,1);
  for (unsigned int tc = 0; tc < n; tc++)
  {
    todo(tc) = 1;
  }
  /************/
  // Define cluster numbers for each side of each non-leaf node
  // clustlist = reshape(1:2*n,n,2);
  matrix<int> clustlist(n, 2);
  int h;
  h = 1;

  for (unsigned int i = 0; i < 2; i++)
  {
    for (unsigned int j = 0; j < n; j++)
    {
      clustlist(j, i) = h++;
    }
  }

  /************/

  while (any(todo))
  {

    //Work on rows that are now split but not yet processed
    //rows = find(todo & ~conn);
    int numberOfNotConnElements = 0;
    for (unsigned int i = 0; i < todo.size(); i++)
    {
      if (todo(i) && !(conn(i)))
        numberOfNotConnElements++;
    }
    int index = 0;
    boost::numeric::ublas::vector<int> rows(numberOfNotConnElements);
    for (unsigned int i = 0; i < todo.size(); i++)
    {
      if (todo(i) && !(conn(i)))
        rows(index++) = i + 1;
    }

    //if isempty(rows), break; end
    if (numberOfNotConnElements == 0)
      break;

    //for j=1:2    % 1=left, 2=right
    for (unsigned int j = 0; j < 2; j++)
    {

      //children = X(rows,j);
      boost::numeric::ublas::vector<int> children(rows.size());
      for (unsigned int i = 0; i < rows.size(); i++)
        children(i) = Z(rows(i) - 1, j);

      //Assign cluster number to child leaf node
      //leaf = (children <= nleaves);
      //if any(leaf)
      //T(children(leaf)) = clustlist(rows(leaf),j);
      //end
      boost::numeric::ublas::vector<int> leaf(children.size());
      int index = 0;
      for (unsigned int i = 0; i < children.size(); i++)
      {
        if (nleaves >= children(i))
          leaf(i) = 1;
        else
          leaf(i) = 0;
        index++;
      }

      //if any(leaf)
      //T(children(leaf)) = clustlist(rows(leaf),j);
      //end

      if (any(leaf))
      {
        for (unsigned int li = 0; li < leaf.size(); li++)
        {
          if (leaf(li) > 0)
            T(children(li) - 1) = clustlist(rows(li) - 1, j);
        }

      }

      //joint = ~leaf;
      boost::numeric::ublas::vector<int> joint(leaf.size());

      for (unsigned int i = 0; i < leaf.size(); i++)
      {
        joint(i) = !(leaf(i));
      }

      //joint(joint) = conn(children(joint)-nleaves);
      for (unsigned int ji = 0; ji < joint.size(); ji++)
      {
        if (joint(ji) > 0)
        {
          joint(ji) = conn(children(ji) - nleaves - 1);
        }
        else
          joint(ji) = 0;
      }

      if (any(joint))
      {
        /*
         clustnum = clustlist(rows(joint),j);
         childnum = children(joint) - nleaves;
         clustlist(childnum,1) = clustnum;
         clustlist(childnum,2) = clustnum;
         conn(childnum) = 0; */

        //joint.size = rows.size
        int numberOfElems = 0;
        for (unsigned int ji = 0; ji < joint.size(); ji++)
        {
          if (joint(ji) > 0)
            numberOfElems++;
        }

        boost::numeric::ublas::vector<int> clustnum(numberOfElems);
        int clustIndex = 0;
        for (unsigned int ci = 0; ci < joint.size(); ci++)
        {
          if (joint(ci) > 0)
            clustnum(clustIndex++) = clustlist(rows(ci) - 1, j);
        }

        boost::numeric::ublas::vector<int> childnum(numberOfElems);
        int childIndex = 0;
        for (unsigned int ci = 0; ci < joint.size(); ci++)
        {

          if (joint(ci) > 0)
            childnum(childIndex++) = children(ci) - nleaves;
        }

        for (unsigned int cli = 0; cli < childnum.size(); cli++)
        {
          clustlist(childnum(cli) - 1, 0) = clustnum(cli);
          clustlist(childnum(cli) - 1, 1) = clustnum(cli);
        }

        for (unsigned int ci = 0; ci < childnum.size(); ci++)
        {
          conn(childnum(ci) - 1) = 0;
        }

      }//if any joint
    }//for j 1 2

    //Mark these rows as done
    //todo(rows) = 0;
    for (unsigned int cc = 0; cc < rows.size(); cc++)
      todo(rows(cc) - 1) = 0;

  }//while any todo
  //Renumber starting from 1
  //[xx1,xx2,T] = unique(T);
  //get T = b(n)


  boost::numeric::ublas::vector<int> b(T.size());
  boost::numeric::ublas::vector<int> bn(T.size());
  boost::numeric::ublas::vector<int>::iterator it;

  it = unique_copy(T.begin(), T.end(), b.begin());
  sort(b.begin(), it);

  it = unique_copy(b.begin(), it, b.begin());

  b.resize(it - b.begin());

  for (unsigned int tc = 0; tc < T.size(); tc++)
  {
    it = find(b.begin(), b.end(), T(tc));
    bn(tc) = int(it - b.begin()) + 1;
  }
  std::cout << "Done" << std::endl;

  return bn;
}//end cluster

boost::numeric::ublas::matrix<double> FundamentalMatrix::pDistJaccard(boost::numeric::ublas::matrix<int> x)
{
  //size_type size1 () const 	Returns the number of rows.
  //size_type size2 () const 	Returns the number of columns.
  int rows = x.size1();
  int columns = x.size2();

  int *xtemp = new int[rows * columns];
  for (int ci = 0; ci < columns; ci++)
  {
    for (int ri = 0; ri < rows; ri++)
    {
      xtemp[ri + ci * rows] = (int)x(ri, ci);
    }
  }

  //The dimension of the output matrix can be calculated with the little gauss d=(n+n^2)/2 with n=columns-1
  int columnSize = (columns - 1 + pow(static_cast<double> (columns - 1), 2)) / 2;
  double *dtemp = new double[columnSize];
  double *dtempStart = dtemp;

  boost::numeric::ublas::matrix<double> d(1, columnSize);
  int theSum, nz;
  int dp = 0;

  int *XI = xtemp;
  for (unsigned int i = 0; i < x.size2(); i++)
  {
    int *XI0 = XI;
    int *XJ = XI + rows;
    for (unsigned int j = i + 1; j < x.size2(); j++)
    {
      XI = XI0;
      theSum = 0;
      nz = 0;
      for (unsigned int k = 0; k < x.size1(); k++, XI++, XJ++)
      {
        //if (xtemp[k+i*rows] || xtemp[k+j*rows]) {
        if ((*XI) || (*XJ))
        {

          nz++;
          if ((*XI) != (*XJ))
          {
            theSum++;
          }
        }
      }
      if (nz > 0)
      {
        *(dtemp++) = (double)theSum / (double)nz;
      }
      else
      {
        *(dtemp++) = 1.0;
      }
    }
  }
  delete (xtemp);
  for (int di = 0; di < columnSize; di++)
  {
    d(0, di) = dtempStart[di];
  }
  delete (dtempStart);
  return d;

}

double FundamentalMatrix::jaccard(boost::numeric::ublas::matrix<int> &logi,
                                  boost::numeric::ublas::vector<int> &obsLogi, int col, int row, int nMod)
{
  static int countIn = 0;
  static int countUn = 0;
  static int i = 0;
  for (countUn = 0, countIn = 0, i = 0; i < nMod; i++)
  {
    if (logi(i, obsLogi(col)) & logi(i, obsLogi(row)))
      countIn++;
    if (logi(i, obsLogi(col)) | logi(i, obsLogi(row)))
      countUn++;
  }

  if (countUn > 0)
  {
    double cc = (double)countIn / (double)countUn;
    double oneD = 1.0;
    double t2d = oneD - cc;
    return t2d;
  }
  else
    return 1.0;
}

boost::numeric::ublas::matrix<double> FundamentalMatrix::linkageIntersect(boost::numeric::ublas::matrix<double> &Y,
                                                                          boost::numeric::ublas::matrix<int> &totdbin)
{
  /*
   cout<<"printint matrix Y"<<endl<<endl;
   cout << Y << endl;
   cout<<endl<<"========================"<<endl;
   cout<<"printint matrix totdbin"<<endl<<endl;
   printMatrix(totdbin);
   cout<<endl<<"========================"<<endl;
   */

  int q = 0;

  //START C++ CONVERTING
  /* number of pairwise distances --> n */
  const int n = Y.size2(); //columns of Y
  const int m = (int)ceil(sqrt(2 * (double)n)); /* size of distance matrix --> m = (1 + sqrt(1+8*n))/2 */
  /* calculate some other constants */
  const int bn = m - 1; /* number of branches     --> bn */
  const int m2 = m * 2; /* 2*m */
  const int m2m3 = m2 - 3; /* 2*m - 3 */
  const int m2m1 = m2 - 1; /* 2*m - 1 */
  int N = 0;

  const int nMod = totdbin.size2();
  const int nPts = totdbin.size1();
  /* find the best value for N (size of the temporal vector of  */
  /* minimums) depending on the problem size */
  if (m > 1023)
    N = 512;
  else if (m > 511)
    N = 256;
  else if (m > 255)
    N = 128;
  else if (m > 127)
    N = 64;
  else if (m > 63)
    N = 32;
  else
    N = 16;

  const double inf = std::numeric_limits<double>::infinity();
  //boost::numeric::ublas::vector<double> T(N);
  //boost::numeric::ublas::vector<int> K(N);
  //boost::numeric::ublas::vector<int> L(N);
  double *Ta = new double[N];
  int *Ka = new int[N];
  int *La = new int[N];

  int *obpa = new int[m];
  int *obsLogia = new int[m];
  int *scla = new int[m];

  int *logia = new int[nMod * nPts];

  for (int i = 0; i < nMod; i++)
    for (int j = 0; j < nPts; j++)
      logia[j * nMod + i] = totdbin(j, i);

  boost::numeric::ublas::matrix<double> outputM(bn, 3);
  for (int r = 0; r < bn; r++)
  {
    for (int s = 0; s < 3; s++)
    {
      outputM(r, s) = 0.0;
    }
  }

  int sT = 0;
  int outPointer = 0;
  int p1 = 0;
  int p2 = 0;
  int q1 = 0;
  int q2 = 0;
  int k = 0;
  int l = 0;
  double t1 = 0.0;
  double t2 = 0.0;
  double t3 = inf;
  int h = 0;
  int i = 0;
  int j = 0;
  int nk = 0;
  int nl = 0;
  int nkpnl = 0;
  int bc = 0;
  int bp = 0;
  int col = 0;
  int row = 0;

  for (i = 0; i < m; obsLogia[i] = i, obpa[i] = i, scla[i++] = 1)
    ;

  for (bc = 0, bp = m; bc < bn; bc++, bp++)
  {

    for (h = 0; ((Ta[h] < t3) && (h < sT)); h++)
      ;
    sT = h;
    t3 = inf;

    /* ONLY when "T" is empty it searches again "y" for the N minimum
     distances  */

    if (sT == 0)
    {
      for (h = 0; h < N; Ta[h++] = inf)
        ;
      p1 = ((m2m1 - bc) * bc) >> 1;
      for (j = bc; j < m; j++)
      {
        for (i = j + 1; i < m; i++)
        {
          t2 = Y(0, p1++);
          if (t2 <= Ta[N - 1])
          {
            for (h = N - 1; ((t2 <= Ta[h - 1]) && (h > 0)); h--)
            {
              Ta[h] = Ta[h - 1];
              Ka[h] = Ka[h - 1];
              La[h] = La[h - 1];
            }
            Ta[h] = t2;
            Ka[h] = j;
            La[h] = i;
            sT++;
          }
        }
      }
      if (sT > N)
        sT = N;
    }
    /* if sT==0 but bc<bn then the remaining distances in "T" must be
     NaN's ! we break the loop, but still need to fill the remaining
     output rows with linkage info and NaN distances
     */

    if (sT == 0)
      break;

    /* the first entry in the ordered vector of distances "T" is the one
     that will be used for this branch, "k" and "l" are its indexes */
    k = Ka[0];
    l = La[0];
    t1 = Ta[0];

    /* some housekeeping over "T" to inactivate all the other minimum
     distances which also have a "k" or "l" index, and then also take
     care of those indexes of the distances which are in the leftmost
     column */
    for (h = 0, i = 1; i < sT; i++)
    {
      /* test if the other entries of "T" belong to the branch "k" or "l"
       if it is true, do not move them in to the updated "T" because
       these distances will be recomputed after merging the clusters */
      if ((k != Ka[i]) && (l != La[i]) && (l != Ka[i]) && (k != La[i]))
      {
        Ta[h] = Ta[i];
        Ka[h] = Ka[i];
        La[h] = La[i];
        /* test if the preserved distances in "T" belong to the
         leftmost column (to be permutated), if it is true find out
         the value of the new indices for such entry */
        if (bc == Ka[h])
        {
          if (k > La[h])
          {
            Ka[h] = La[h];
            La[h] = k;
          } /* if (k> ...*/
          else
            Ka[h] = k;
        } /* if (bc== ... */
        h++;
      } /* if k!= ... */
    } /* for (h=0 ... */
    sT = h; /* the new size of "T" after the shifting */
    /* Update output for this branch, puts smaller pointers always in the
     leftmost column  */

    if (obpa[k] < obpa[l])
    {
      outputM(outPointer, 0) = (double)(obpa[k] + 1); /* +1 since Matlab ptrs start at 1 */
      outputM(outPointer, 1) = (double)(obpa[l] + 1);
    }
    else
    {
      outputM(outPointer, 0) = (double)(obpa[l] + 1);
      outputM(outPointer, 1) = (double)(obpa[k] + 1);
    }
    outputM(outPointer++, 2) = (double)t1;

    /* Updates obs-branch pointers "obp" */
    obpa[k] = obpa[bc]; /* new cluster branch ptr */
    obpa[l] = bp; /* leftmost column cluster branch ptr */

    /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /* /*
     Merges two observations/clusters ("k" and "l") by re-calculating new
     distances for every remaining observation/cluster and place the
     information in the row/col "l" */

    /*

     example:  bc=2  k=5  l=8   bn=11   m=12

     0
     1    N                             Pairwise
     2    N   N                         Distance
     3    N   N   Y                     Half Matrix
     4    N   N   Y   Y
     5    N   N  p1*  *   *
     6    N   N   Y   Y   Y   +
     7    N   N   Y   Y   Y   +   Y
     8    N   N  p2*  *   *   []  +   +
     9    N   N   Y   Y   Y   o   Y   Y   o
     10   N   N   Y   Y   Y   o   Y   Y   o   Y
     11   N   N   Y   Y   Y   o   Y   Y   o   Y   Y

     0   1   2   3   4   5   6   7   8   9   10   11


     p1 is the initial pointer for the kth row-col
     p2 is the initial pointer for the lth row-col
     *  are the samples touched in the first loop
     +  are the samples touched in the second loop
     o  are the samples touched in the third loop
     N  is the part of the whole half matrix which is no longer used
     Y  are all the other samples (not touched)

     */

    /* computing some limit constants to set up the 3-loops to
     transverse Y */
    q1 = bn - k - 1;
    q2 = bn - l - 1;

    /* initial pointers to the "k" and  "l" entries in the remaining half
     matrix */
    p1 = (((m2m1 - bc) * bc) >> 1) + k - bc - 1;
    p2 = p1 - k + l;

    /* Get the cluster cardinalities  */
    nk = scla[k];
    nl = scla[l];
    nkpnl = nk + nl;

    /* Updates cluster cardinality "scl" */
    scla[k] = scla[bc]; /* leftmost column cluster cardinality */
    scla[l] = nkpnl; /* new cluster cardinality */

    //nMod columns
    //INTERSECT, fomer: 	 intersect(&logi(obsLogi(k),nMod), &logi((obsLogi(l))), nMod);

    for (i = 0; i < nMod; i++)
    {
      //logi(i,obsLogia[k]) = logi(i,obsLogia[l]) = (logi(i,obsLogia[k]) & logi(i,obsLogia[l]));
      logia[nMod * obsLogia[k] + i] = logia[nMod * obsLogia[l] + i] = (logia[nMod * obsLogia[k] + i] & logia[nMod
          * obsLogia[l] + i]);
    }
    col = bc;
    row = l;
    i = 0;
    int countUn;
    int countIn;
    for (q = bn - bc - 1; q > q1; q--)
    {
      /**********JACCARD***********************/

      for (countUn = 0, countIn = 0, i = 0; i < nMod; i++)
      {
        if (logia[nMod * obsLogia[col] + i] & logia[nMod * obsLogia[row] + i])
          countIn++;
        if (logia[nMod * obsLogia[col] + i] | logia[nMod * obsLogia[row] + i])
          countUn++;
      }

      if (countUn > 0)
      {
        double cc = (double)countIn / (double)countUn;
        double oneD = 1.0;
        t2 = oneD - cc;
      }
      else
        t2 = 1.0;

      //t2 = jaccard(logi,obsLogi, col, row, nMod);
      /****************************************/
      if (t2 < t3)
        t3 = t2;
      Y(0, p2) = t2;
      p1 = p1 + q;
      p2 = p2 + q;
      col++;
    }
    col++;
    p1++;
    p2 = p2 + q;
    for (q = q1 - 1; q > q2; q--)
    {
      /**********JACCARD***********************/
      for (countUn = 0, countIn = 0, i = 0; i < nMod; i++)
      {
        if (logia[nMod * obsLogia[col] + i] & logia[nMod * obsLogia[row] + i])
          countIn++;
        if (logia[nMod * obsLogia[col] + i] | logia[nMod * obsLogia[row] + i])
          countUn++;
      }

      if (countUn > 0)
      {
        double cc = (double)countIn / (double)countUn;
        double oneD = 1.0;
        t2 = oneD - cc;
      }
      else
        t2 = 1.0;

      //t2 = jaccard(logi,obsLogi, col, row, nMod);
      /****************************************/
      if (t2 < t3)
        t3 = t2;
      Y(0, p2) = t2;
      p1++;
      p2 = p2 + q;
      col++;
    }
    p1++;
    p2++;
    col = l;
    row++;
    for (q = q2 + 1; q > 0; q--)
    {
      /**********JACCARD***********************/
      for (countUn = 0, countIn = 0, i = 0; i < nMod; i++)
      {
        if (logia[nMod * obsLogia[col] + i] & logia[nMod * obsLogia[row] + i])
          countIn++;
        if (logia[nMod * obsLogia[col] + i] | logia[nMod * obsLogia[row] + i])
          countUn++;
      }
      if (countUn > 0)
      {
        double cc = (double)countIn / (double)countUn;
        double oneD = 1.0;
        t2 = oneD - cc;
      }
      else
        t2 = 1.0;

      //t2 = jaccard(logi,obsLogi, col, row, nMod);
      /****************************************/
      /**********JACCARD***********************/
      /*
       for (countUn = 0, countIn = 0, i = 0; i < nMod; i++)
       {
       if ( logi(i,obsLogia[col]) & logi(i,obsLogia[row]) ) countIn++;
       if ( logi(i,obsLogia[col]) | logi(i,obsLogia[row]) ) countUn++;
       }

       if(countUn > 0){
       double cc = (double)countIn/(double)countUn;
       double oneD = 1.0;
       t2 = oneD - cc;
       }
       else t2=1.0;
       */
      //t2 = jaccard(logi,obsLogi, col, row, nMod);
      /****************************************/
      if (t2 < t3)
        t3 = t2;
      Y(0, p2) = t2;
      p1++;
      p2++;
      row++;
    }

    obsLogia[k] = obsLogia[bc];
    // moves the leftmost column "bc" to row/col "k"
    if (k != bc)
    {
      q1 = bn - k;
      p1 = (((m2m3 - bc) * bc) >> 1) + k - 1;
      p2 = p1 - k + bc + 1;

      for (q = bn - bc - 1; q > q1; q--)
      {
        p1 = p1 + q;
        Y(0, p1) = Y(0, p2++);
      }
      p1 = p1 + q + 1;
      p2++;
      for (; q > 0; q--)
      {
        Y(0, p1++) = Y(0, p2++);
      }
    } /*if (k!=bc) */
  } /*for (bc=0,bp=m;bc<bn;bc++,bp++) */

  /* loop to fill with NaN's in case the main loop ended prematurely */
  for (; bc < bn; bc++, bp++)
  {
    k = bc;
    l = bc + 1;
    if (obpa[k] < obpa[l])
    {
      outputM(outPointer, 0) = (double)(obpa[k] + 1.0);
      outputM(outPointer, 1) = (double)(obpa[l] + 1.0);
    }
    else
    {
      outputM(outPointer, 0) = (double)(obpa[l] + 1.0);
      outputM(outPointer, 1) = (double)(obpa[k] + 1.0);
    }
    obpa[l] = bp;
    outputM(outPointer++, 2) = std::numeric_limits<double>::quiet_NaN(); //NAN
  }

  delete (scla);
  delete (obpa);
  delete (obsLogia);
  delete (Ka);
  delete (Ta);
  delete (La);
  delete (logia);
  return outputM;

}

}
;
