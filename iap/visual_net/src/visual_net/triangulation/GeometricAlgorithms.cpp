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
#include "GeometricAlgorithms.h"

using namespace std;
using namespace vision;

void GeometricAlgorithms::removeIntersections(vector<FeaturePtr> features, bool **connectivityMatrix){
  vector<FeaturePtr> neighbors;
  for(unsigned int i=0;i<features.size();i++){
    neighbors.clear();
    for(unsigned int j=0;j<features.size();j++){
      if(connectivityMatrix[i][j]){
        neighbors.push_back(features[j]);
      }
    }
    //now neighbors contains all the neighbors of vertex i
    if(checkIntersection(features[i],neighbors,features,connectivityMatrix)){
      //we want to remove all edges to feature i
      for(unsigned int j=0;j<features.size();j++){
        connectivityMatrix[i][j]=false;
        connectivityMatrix[j][i]=false;
      }
    }
  }
}

bool GeometricAlgorithms::checkIntersection(FeaturePtr f, vector<FeaturePtr> neighbors, vector<FeaturePtr> features, bool **connectivityMatrix){
  //create internal connectivity matrix
  bool **M=new bool*[neighbors.size()];
  for(unsigned int i=0;i<neighbors.size();i++) M[i]=new bool[neighbors.size()];
  for(unsigned int i=0;i<neighbors.size();i++) for(unsigned int j=0;j<neighbors.size();j++) M[i][j]=0;

  //fill the matrix
  for(int i1=0;i1<(int)neighbors.size();i1++){
    for(int i2=i1+1;i2<(int)neighbors.size();i2++){
      int j1,j2;
      bool found1, found2;
      for(j1=0;j1<(int)features.size();j1++){
        if(neighbors[i1]==features[j1]){
          found1=true;
          break;
        }else{
          found1=false;
        }
      }
      for(j2=0;j2<(int)features.size();j2++){
        if(neighbors[i2]==features[j2]){
          found2=true;
          break;
        }else{
          found2=false;
        }
      }
     
      if(found1 && found2){ 
        if(connectivityMatrix[j1][j2]){
          M[i1][i2]=true;
          M[i2][i1]=true;
        }else{
          M[i1][i2]=false;
          M[i2][i1]=false; 
        }
      }
    }
  }

  for(unsigned int i1=0;i1<neighbors.size();i1++){
    for(unsigned int i2=i1+1;i2<neighbors.size();i2++){
      for(unsigned int i3=0;i3<neighbors.size();i3++){
        if(i3!=i1 && i3!=i2 && M[i2][i3]){
          //f is connected to all neighbors; i2 and i3 are connected as well
          if(checkIntersection(f,neighbors[i1],neighbors[i2],neighbors[i3])) return true;
        }
      }
    }
  }
  
  return false;
}


bool GeometricAlgorithms::checkIntersection(FeaturePtr f1a, FeaturePtr f1b, FeaturePtr f2a, FeaturePtr f2b){
  double x1,x2,x3,x4,y1,y2,y3,y4;
  x1=f1a->getX(); y1=f1a->getY();
  x2=f1b->getX(); y2=f1b->getY();
  x3=f2a->getX(); y3=f2a->getY();
  x4=f2b->getX(); y4=f2b->getY();

  double a1, a2, b1, b2, c1, c2, r1, r2, r3, r4, denom, offset, num;

  // Compute a1, b1, c1, where line joining points 1 and 2
  // is "a1 x + b1 y + c1 = 0".
  a1 = y2 - y1;
  b1 = x1 - x2;
  c1 = (x2 * y1) - (x1 * y2);

  // Compute r3 and r4.
  r3 = ((a1 * x3) + (b1 * y3) + c1);
  r4 = ((a1 * x4) + (b1 * y4) + c1);

  // Check signs of r3 and r4. If both point 3 and point 4 lie on
  // same side of line 1, the line segments do not intersect.
  if ((r3 != 0) && (r4 != 0) && same_sign(r3, r4)) return false; //DONT_INTERSECT

  // Compute a2, b2, c2
  a2 = y4 - y3;
  b2 = x3 - x4;
  c2 = (x4 * y3) - (x3 * y4);

  // Compute r1 and r2
  r1 = (a2 * x1) + (b2 * y1) + c2;
  r2 = (a2 * x2) + (b2 * y2) + c2;

  // Check signs of r1 and r2. If both point 1 and point 2 lie
  // on same side of second line segment, the line segments do
  // not intersect.
  if ((r1 != 0) && (r2 != 0) && (same_sign(r1, r2))) return false; //DONT_INTERSECT

  //Line segments intersect: compute intersection point.
  denom = (a1 * b2) - (a2 * b1);

  if (denom == 0) return false; //COLLINEAR
  
  if (denom < 0)  offset = -denom / 2; 
  else            offset =  denom / 2;
  

  // The denom/2 is to get rounding instead of truncating. It
  // is added or subtracted to the numerator, depending upon the
  // sign of the numerator.
  double x,y;
  num = (b1 * c2) - (b2 * c1);
  if (num < 0)  x = (num - offset) / denom;
  else          x = (num + offset) / denom;
  
  num = (a2 * c1) - (a1 * c2);
  if (num < 0)  y = ( num - offset) / denom;
  else          y = ( num + offset) / denom;
  
  // lines_intersect
  return true; //DO_INTERSECT
}

bool GeometricAlgorithms::same_sign(double a, double b){
  return (( a * b) >= 0);
}


void GeometricAlgorithms::removeTrianglesArea(vector<FeaturePtr> prevFeatures, vector<FeaturePtr> currFeatures, bool **connectivityMatrix){
  vector<FeaturePtr> prevNeighbors, currNeighbors;
  for(unsigned int i=0;i<prevFeatures.size();i++){
    prevNeighbors.clear(); 
    currNeighbors.clear();
    for(unsigned int j=0;j<prevFeatures.size();j++){
      if(connectivityMatrix[i][j]){
        prevNeighbors.push_back(prevFeatures[j]);
        currNeighbors.push_back(currFeatures[j]);
      }
    }
    //now neighbors contains all the neighbors of vertex i (prev and curr)
    if(areaChanged(prevFeatures[i],currFeatures[i],prevNeighbors,currNeighbors,prevFeatures,currFeatures,connectivityMatrix)){
      //we want to remove all edges to feature i
      for(unsigned int j=0;j<currFeatures.size();j++){
        connectivityMatrix[i][j]=false;
        connectivityMatrix[j][i]=false;
      }
    }
  }
}

bool GeometricAlgorithms::areaChanged(FeaturePtr prevF, FeaturePtr currF, vector<FeaturePtr> prevNeighbors, vector<FeaturePtr> currNeighbors,
                                             vector<FeaturePtr> prevFeatures,  vector<FeaturePtr> currFeatures, bool **connectivityMatrix){
  double prevArea=0;
  double currArea=0;

  vector<double> prevAreas=computeTrianglesArea(prevF,prevNeighbors,prevFeatures,connectivityMatrix);
  vector<double> currAreas=computeTrianglesArea(currF,currNeighbors,currFeatures,connectivityMatrix);

  for(vector<double>::iterator it=prevAreas.begin();it!=prevAreas.end();it++) prevArea+=*it;
  for(vector<double>::iterator it=currAreas.begin();it!=currAreas.end();it++) currArea+=*it;

  if((fabs(prevArea-currArea)/currArea)>0.3) return true;
  return false;
}

vector<double> GeometricAlgorithms::computeTrianglesArea(FeaturePtr f, vector<FeaturePtr> neighbors, vector<FeaturePtr> features, bool **connectivityMatrix){
  //create internal connectivity matrix
  bool **M=new bool*[neighbors.size()];
  for(unsigned int i=0;i<neighbors.size();i++) M[i]=new bool[neighbors.size()];
  for(unsigned int i=0;i<neighbors.size();i++) for(unsigned int j=0;j<neighbors.size();j++) M[i][j]=0;

  //fill the matrix
  for(int i1=0;i1<(int)neighbors.size();i1++){
    for(int i2=i1+1;i2<(int)neighbors.size();i2++){
      int j1,j2;
      bool found1, found2;
      for(j1=0;j1<(int)features.size();j1++){
        if(neighbors[i1]==features[j1]){
          found1=true;
          break;
        }else{
          found1=false;
        }
      }
      for(j2=0;j2<(int)features.size();j2++){
        if(neighbors[i2]==features[j2]){
          found2=true;
          break;
        }else{
          found2=false;
        }
      }

      if(found1 && found2){
        if(connectivityMatrix[j1][j2]){
          M[i1][i2]=true;
          M[i2][i1]=true;
        }else{
          M[i1][i2]=false;
          M[i2][i1]=false;
        }
      }
    }
  }

  vector<double> trianglesArea;
  for(unsigned int i1=0;i1<neighbors.size();i1++){
    for(unsigned int i2=i1+1;i2<neighbors.size();i2++){
      if(M[i1][i2]){
        //f is connected to i1,i2 by definition (neighbors of f)
        //a triangle is formed if i1,i2 are connected
        //compute the area
        double a=sqrt((neighbors[i1]->getX()-neighbors[i2]->getX())*(neighbors[i1]->getX()-neighbors[i2]->getX())
                     +(neighbors[i1]->getY()-neighbors[i2]->getY())*(neighbors[i1]->getY()-neighbors[i2]->getY()));
        double b=sqrt((neighbors[i1]->getX()-f->getX())*(neighbors[i1]->getX()-f->getX())
                     +(neighbors[i1]->getY()-f->getY())*(neighbors[i1]->getY()-f->getY()));
        double c=sqrt((neighbors[i2]->getX()-f->getX())*(neighbors[i2]->getX()-f->getX())
                     +(neighbors[i2]->getY()-f->getY())*(neighbors[i2]->getY()-f->getY()));
        double s=(a+b+c)/2.0;
        double area=sqrt(s*(s-a)*(s-b)*(s-c));
        trianglesArea.push_back(area);
      }
    }
  }

  return trianglesArea;
}

