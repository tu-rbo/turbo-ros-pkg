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
#include "StructureFromMotion.h"

#include "Graph.h"
#include <iostream>
#include "FundamentalMatrix.h"
#include <time.h>
#include <cmath>

using namespace std;
using namespace vision;
using namespace boost::numeric::ublas;
using namespace SFM;

namespace VisualGraph
{

  StructureFromMotion::StructureFromMotion(int frame1, int frame2,
      int numHypotheses, CameraModel &cammodel) :
    Predictor(string("StructureFromMotion"))
  {
    this->numHypotheses = numHypotheses;
    this->cammodel = cammodel;
  }

  StructureFromMotion::~StructureFromMotion(void)
  {

  }

  std::vector<std::vector<FeaturePtr> >
  StructureFromMotion::compute(std::vector<std::vector<FeaturePtr> > cluster)
  {
    //for now the input to this function will be a twoD cluster [frame][feature]
    //in the future we should generate the clusters from the graph based on the clusterID of the vertices
    //and also remember to scale to [-1,1] (look in FundamentalMatrix code!
    cout
        << "for now, hardcoding number of vertices, but later should get by adding predictor to graph"
        << endl;
    numVertices = cluster.size();

    scaleInputs(cluster);

    std::vector < SFM::EKF > hypothesesModel; //a vector of generated hypotheses (i.e. model/Fundamental matrix)
    std::vector<boost::numeric::ublas::vector<double> > hypothesesScore; //a vector of scores for the generated hypotheses

    for (int hyp = 0; hyp < numHypotheses; hyp++)
      {
        int sampleSize = 8;
        //pickRandomSample(8,1,n) = returns a vector of 8 number in the range of 1...n where n=numPts
        std::vector<int> sample = pickRandomSample(sampleSize, 0,
            cluster.size() - 1);

        //generate a sample of 8pts
        std::vector < std::vector < FeaturePtr > > sampleFeatures;
        for (unsigned int frame = 0; frame < cluster[0].size(); frame++)
          {
            std::vector < FeaturePtr > currFrame;
            for (int j = 0; j < sampleSize; j++)
              {
                currFrame.push_back(cluster[sample[j]][frame]);
              }
            sampleFeatures.push_back(currFrame);
          }

        //a model can always be computed (no degenrate case)
        SFM::EKF  model = computeModel(sampleFeatures);
        boost::numeric::ublas::vector<double> score = fitsModel(model, cluster);

        hypothesesScore.push_back(score);
      }

    //now we have a vector of hypotheses and we need to perform clustering
    //cluster points -- needs the scored hyp and some inlier th parameter to return the clustering (depends on c++/mex files and matlab's cluster function...
    boost::numeric::ublas::matrix<double> Y;
    boost::numeric::ublas::matrix<double> Z;
    boost::numeric::ublas::vector<int> T;
    boost::numeric::ublas::matrix<int> totdbin;
    double inliersThreshold = 0.05;

    boost::numeric::ublas::matrix<double> totd(numVertices, numHypotheses);
    for (int i = 0; i < numHypotheses; i++)
      {
        for (int j = 0; j < numVertices; j++)
          {
            totd(j, i) = hypothesesScore[i](j);
          }
      }

    clusterPoints(totd, inliersThreshold, Y, Z, T, totdbin);

    cout << T << endl;

    map<int, int> clusterIDs;
    for (int i = 0; i < numVertices; i++)
      {
        if (clusterIDs.find(T(i)) == clusterIDs.end())
          {
            clusterIDs[T(i)] = 1;
          }
        else
          {
            clusterIDs[T(i)] = clusterIDs.find(T(i))->second + 1;
          }
      }

    int largestClusterID = clusterIDs.begin()->first;
    int largestClusterSize = clusterIDs.begin()->second;

    for (map<int, int>::iterator it = clusterIDs.begin(); it
        != clusterIDs.end(); it++)
      {
        if (it->second > largestClusterSize)
          {
            largestClusterID = it->first;
            largestClusterSize = it->second;
          }
      }

    std::vector < std::vector < FeaturePtr > > consistentFeatures;
    for (unsigned int frame = 0; frame < cluster[0].size(); frame++)
      {
        std::vector < FeaturePtr > currFrame;
        for (int j = 0; j < numVertices; j++)
          {
            if (T(j) == largestClusterID)
              {
                currFrame.push_back(cluster[j][frame]);
              }
          }
        consistentFeatures.push_back(currFrame);
      }

    //a model can always be computed (no degenrate case)
    SFM::EKF  ukf(cammodel);
    //return ukf.runTwice(consistentFeatures);

    //now based on totd we have clusters, and we can set the edges of the graph
    for (int i = 0; i < numVertices; i++)
      {
        for (int j = i + 1; j < numVertices; j++)
          {
            if (T(i) == T(j))
              {
                //adjMatrix[i][j]->setCapacity(predictorName, 1.0);
                //adjMatrix[j][i]->setCapacity(predictorName, 1.0);
              }
          }
      }
  }

  std::vector<int>
  StructureFromMotion::pickRandomSample(int sampleSize, int low, int high)
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

  SFM::EKF
  StructureFromMotion::computeModel(std::vector<std::vector<FeaturePtr> > features)
  {
    //this function should get N pts over all frames between frame1-frame2
    //compute UKF 2 runs on the features
    //return a UKF object that represents our model
    SFM::EKF  ukf(cammodel);
    //ukf.runTwice(features);
    return ukf;
  }

  boost::numeric::ublas::vector<double>
  StructureFromMotion::fitsModel(SFM::EKF  &ukf,
      std::vector<std::vector<FeaturePtr> > &testPts)
  {
    //computes the distance of all points from the model
    //this functions get as input a model (UKF object) and a vector<vector<FeaturePtr>> which for every feature has a vector of its observations over time
    //it gets back from the UKF a vector<double> representing the error in pixels in every frame
    //we need to analyze this error, and compute one number representing the error
    //and store it in the dist vector
    boost::numeric::ublas::vector<double> dist(numVertices, 1);

    for (unsigned int featureNum = 0; featureNum < testPts.size(); featureNum++)
      {
       // double err = ukf.testFeature(testPts[featureNum]);
       // dist(featureNum) = err;
      }

    return dist;
  }

  void
  StructureFromMotion::clusterPoints(
      boost::numeric::ublas::matrix<double> &totd, double inliersThreshold,
      boost::numeric::ublas::matrix<double> &Y,
      boost::numeric::ublas::matrix<double> &Z,
      boost::numeric::ublas::vector<int> &T,
      boost::numeric::ublas::matrix<int> &totdbin)
  {
    //Find every entry below the threshold
    totdbin.resize(totd.size1(), totd.size2());

    for (unsigned int i = 0; i < totdbin.size1(); i++)
      {
        for (unsigned int j = 0; j < totdbin.size2(); j++)
          {
            if (totd(i, j) < inliersThreshold)
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
    double cutoff = 1.0 - (1.0 / (double) totdbin.size1()) + eps;

    T.resize(Z.size1() + 1);
    T = cluster(Z, cutoff);
  }

  int
  StructureFromMotion::any(boost::numeric::ublas::vector<int> in)
  {
    for (unsigned int i = 0; i < in.size(); i++)
      {
        if (in(i) > 0)
          return 1; //true
      }
    return 0; //false
  }

  boost::numeric::ublas::vector<int>
  StructureFromMotion::cluster(boost::numeric::ublas::matrix<double> Z,
      double cutoff)
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
    for (unsigned int i = 0; i < Z.size1(); todo(i) = conn(i) && (Z(i, 1) > n
        || Z(i, 2) > n), i++)
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
                boost::numeric::ublas::vector<int> child(
                    (int) count(t.begin(), t.end(), 1));
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

  boost::numeric::ublas::matrix<double>
  StructureFromMotion::pDistJaccard(boost::numeric::ublas::matrix<int> x)
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
            xtemp[ri + ci * rows] = (int) x(ri, ci);
          }
      }

    //The dimension of the output matrix can be calculated with the little gauss d=(n+n^2)/2 with n=columns-1
    int columnSize = (columns - 1 + pow(static_cast<double> (columns - 1), 2))
        / 2;
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
                *(dtemp++) = (double) theSum / (double) nz;
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

  double
  StructureFromMotion::jaccard(boost::numeric::ublas::matrix<int> &logi,
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
        double cc = (double) countIn / (double) countUn;
        double oneD = 1.0;
        double t2d = oneD - cc;
        return t2d;
      }
    else
      return 1.0;
  }

  boost::numeric::ublas::matrix<double>
  StructureFromMotion::linkageIntersect(
      boost::numeric::ublas::matrix<double> &Y,
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
    const int m = (int) ceil(sqrt(2 * (double) n)); /* size of distance matrix --> m = (1 + sqrt(1+8*n))/2 */
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
            outputM(outPointer, 0) = (double) (obpa[k] + 1); /* +1 since Matlab ptrs start at 1 */
            outputM(outPointer, 1) = (double) (obpa[l] + 1);
          }
        else
          {
            outputM(outPointer, 0) = (double) (obpa[l] + 1);
            outputM(outPointer, 1) = (double) (obpa[k] + 1);
          }
        outputM(outPointer++, 2) = (double) t1;

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
            logia[nMod * obsLogia[k] + i] = logia[nMod * obsLogia[l] + i]
                = (logia[nMod * obsLogia[k] + i]
                    & logia[nMod * obsLogia[l] + i]);
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
                if (logia[nMod * obsLogia[col] + i] & logia[nMod
                    * obsLogia[row] + i])
                  countIn++;
                if (logia[nMod * obsLogia[col] + i] | logia[nMod
                    * obsLogia[row] + i])
                  countUn++;
              }

            if (countUn > 0)
              {
                double cc = (double) countIn / (double) countUn;
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
                if (logia[nMod * obsLogia[col] + i] & logia[nMod
                    * obsLogia[row] + i])
                  countIn++;
                if (logia[nMod * obsLogia[col] + i] | logia[nMod
                    * obsLogia[row] + i])
                  countUn++;
              }

            if (countUn > 0)
              {
                double cc = (double) countIn / (double) countUn;
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
                if (logia[nMod * obsLogia[col] + i] & logia[nMod
                    * obsLogia[row] + i])
                  countIn++;
                if (logia[nMod * obsLogia[col] + i] | logia[nMod
                    * obsLogia[row] + i])
                  countUn++;
              }
            if (countUn > 0)
              {
                double cc = (double) countIn / (double) countUn;
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
            outputM(outPointer, 0) = (double) (obpa[k] + 1.0);
            outputM(outPointer, 1) = (double) (obpa[l] + 1.0);
          }
        else
          {
            outputM(outPointer, 0) = (double) (obpa[l] + 1.0);
            outputM(outPointer, 1) = (double) (obpa[k] + 1.0);
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

  void
  StructureFromMotion::scaleInputs(std::vector<std::vector<FeaturePtr> > &input)
  {
    for (std::vector<std::vector<FeaturePtr> >::iterator it1 = input.begin(); it1
        != input.end(); it1++)
      {
        for (std::vector<FeaturePtr>::iterator it2 = it1->begin(); it2
            != it1->end(); it2++)
          {
            (*it2)->setX(
                (float) (((*it2)->getX() - cammodel.getXresolution() / 2.0)
                    / cammodel.getXresolution()) * 2);
            (*it2)->setY(
                (float) (((*it2)->getY() - cammodel.getYresolution() / 2.0)
                    / cammodel.getXresolution()) * 2);
          }
      }
  }

}
;
