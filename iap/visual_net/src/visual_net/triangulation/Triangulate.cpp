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
#include "Triangulate.h"

using namespace std;
using namespace vision;

Triangulate::Triangulate(vector<FeaturePtr> features, CvRect _rect){
  for(vector<FeaturePtr>::iterator it=features.begin();it!=features.end();it++){
    CvPoint2D32f point=cvPoint2D32f((int)(*it)->getX(),(int)(*it)->getY());
    points.push_back(point);
  }

  rect=_rect;

  triangulation=new bool*[points.size()];
  for(unsigned int i=0;i<points.size();i++) triangulation[i]=new bool[points.size()];
  for(unsigned int i=0;i<points.size();i++){
    for(unsigned int j=0;j<points.size();j++){
      triangulation[i][j]=false;
    }
  }

  storage = cvCreateMemStorage(0);
  subdiv = init_delaunay(storage);
  for(vector<CvPoint2D32f>::iterator it=points.begin();it!=points.end();it++){
    cvSubdivDelaunay2DInsert( subdiv, *it);
  }

  draw_subdiv(subdiv);
}

Triangulate::~Triangulate(){
  cvReleaseMemStorage( &storage );
  for(unsigned int i=0;i<points.size();i++){
	  delete(triangulation[i]);
  }
  delete(triangulation);
}

bool Triangulate::isInVector(const CvPoint2D32f &pt){
  for(vector<CvPoint2D32f>::const_iterator it=points.begin();it!=points.end();it++){
    if(it->x==pt.x && it->y==pt.y) return true;
  }
  return false;
}

bool** Triangulate::getTriangulation(){
	bool** TRI=new bool*[points.size()];
	for(unsigned int i=0;i<points.size();i++)
		TRI[i]=new bool[points.size()];
	for(unsigned int i=0;i<points.size();i++){
		for(unsigned int j=0;j<points.size();j++){
			TRI[i][j]=triangulation[i][j];
		}
	}
	return TRI;
}

CvSubdiv2D* Triangulate::init_delaunay( CvMemStorage* storage){
  CvSubdiv2D* subdiv;
  subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv), sizeof(CvSubdiv2DPoint), sizeof(CvQuadEdge2D), storage );
  cvInitSubdivDelaunay2D( subdiv, rect );
  return subdiv;
}

void Triangulate::draw_subdiv_edge(CvSubdiv2DEdge edge){
  CvSubdiv2DPoint* org_pt;
  CvSubdiv2DPoint* dst_pt;
  CvPoint2D32f org;
  CvPoint2D32f dst;
  CvPoint iorg, idst;

  org_pt = cvSubdiv2DEdgeOrg(edge);
  dst_pt = cvSubdiv2DEdgeDst(edge);

  if(org_pt && dst_pt){
    org = org_pt->pt;
    dst = dst_pt->pt;

    if(!isInVector(org) || !isInVector(dst)) return;

    iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
    idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));

    markMatrix(iorg, idst);
  }
}

void Triangulate::draw_subdiv(CvSubdiv2D* subdiv){
  CvSeqReader  reader;
  int i, total = subdiv->edges->total;
  int elem_size = subdiv->edges->elem_size;

  cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

  for( i = 0; i < total; i++ ){
    CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

    if( CV_IS_SET_ELEM( edge )){
      draw_subdiv_edge((CvSubdiv2DEdge)edge);
    }
    CV_NEXT_SEQ_ELEM( elem_size, reader );
  }
}

void Triangulate::markMatrix(CvPoint pt1, CvPoint pt2){
  int i=0;
  int j=0;
  for(vector<CvPoint2D32f>::const_iterator it=points.begin();it!=points.end();it++){
    if(pt1.x==it->x && pt1.y==it->y) break;
    i++;
  }

  for(vector<CvPoint2D32f>::const_iterator it=points.begin();it!=points.end();it++){
    if(pt2.x==it->x && pt2.y==it->y) break;
    j++;
  }
    
  triangulation[i][j]=true;
  triangulation[j][i]=true;
}

bool Triangulate::isIntersecting(pair<double,double> A1, pair<double,double> A2, pair<double,double> B1, pair<double,double> B2){
	double Ax=A1.first;
	double Ay=A1.second;
	double Bx=A2.first;
	double By=A2.second;
	double Cx=B1.first;
	double Cy=B1.second;
	double Dx=B2.first;
	double Dy=B2.second;
	
	pair<double,double> DP=pair<double,double>((Cx-Ax)+(Dx-Bx),(Cy-Ay)+(Dy-By));
	pair<double,double> QA=pair<double,double>(Bx-Ax,By-Ay);
	pair<double,double> QB=pair<double,double>(Dx-Cx,Dy-Cy);

	double d  = QA.second * QB.first - QB.second * QA.first;
	double la = QB.first * DP.second - QB.second * DP.first;
	double lb = QA.first * DP.second - QA.second * DP.first;
	
	if(abs(la)<=abs(d) && abs(lb)<=abs(d))
		return true;
	else
		return false;
	
}



bool** Triangulate::getIntersectingEdges(vector<FeaturePtr> newPoints){
	//newPoints is the new position of the points that were triangulated by this class(points)
	//we try to check intersection between edges created when applying the original triangulation
	//to the new position of the points (the triangles' vertices)

	//create a vector of [i,j,0] for pairs of points connected by an edge
	vector< vector<int> > edges;
	vector<int> edgeInfo;
	int numPts=points.size();

	for(int i=0;i<numPts;i++){
		for(int j=i+1;j<numPts;j++){
			if(triangulation[i][j]){
				edgeInfo.clear();
				edgeInfo.push_back(i);		//vertex index for existing edge in triangulation
				edgeInfo.push_back(j);		//vertex index for existing edge in triangulation
				edgeInfo.push_back(0);		//0=counter for number of intersections
				edgeInfo.push_back(1);		//1=consider this edge for computing intersections
				edges.push_back(edgeInfo);
			}
		}
	}

	vector< vector<int> >::iterator it1, it2;
	for(it1=edges.begin();it1!=edges.end();it1++){
		for(it2=it1+1;it2!=edges.end();it2++){
			//verify that edges have no vertex in common
			if( (*it1)[0]!=(*it2)[0] && (*it1)[0]!=(*it2)[1] && (*it1)[1]!=(*it2)[0] && (*it1)[1]!=(*it2)[1] && (*it1)[3]==1 && (*it2)[3]==1){
				pair<double,double> A1=pair<double,double>(newPoints[(*it1)[0]]->getX(), newPoints[(*it1)[0]]->getY());
				pair<double,double> A2=pair<double,double>(newPoints[(*it1)[1]]->getX(), newPoints[(*it1)[1]]->getY());
				pair<double,double> B1=pair<double,double>(newPoints[(*it2)[0]]->getX(), newPoints[(*it2)[0]]->getY());
				pair<double,double> B2=pair<double,double>(newPoints[(*it2)[1]]->getX(), newPoints[(*it2)[1]]->getY());
				if(isIntersecting(A1,A2,B1,B2)){
					//increase the intersections counter
					(*it1)[2]=(*it1)[2]+1;
					(*it2)[2]=(*it2)[2]+1;
				}
			}
		}
	}


	for(it1=edges.begin();it1!=edges.end();it1++){
		if((*it1)[2]==0){
			//no intersections, therefore we can remove it completely!
			edges.erase(it1);
			it1=edges.begin();
		}
	}

	while(true){
		//find the edge with max number of intersections
		//and make maxIt be an iterator for that edge entry
		int max=0;
		vector< vector<int> >::iterator maxIt;
		for(maxIt=edges.begin();maxIt!=edges.end();maxIt++)
			if(max<(*maxIt)[2] && (*maxIt)[3]==1)
				max=(*maxIt)[2];
		for(maxIt=edges.begin();maxIt!=edges.end();maxIt++){
			if(max==(*maxIt)[2] && (*maxIt)[3]==1){
				break;
			}
		}

		if(max==0){
			//no more intersections, so we want to go over the edges
			//and return a matrix of edges to be removed
			bool ** edgesToRemove=new bool*[numPts];
			for(int i=0;i<numPts;i++)
				edgesToRemove[i]=new bool[numPts];
			for(int i=0;i<numPts;i++)
				for(int j=0;j<numPts;j++)
					edgesToRemove[i][j]=false;
			for(it1=edges.begin();it1!=edges.end();it1++){
				if((*it1)[3]==0){
					//we've marked that edge for removal
					edgesToRemove[(*it1)[0]][(*it1)[1]]=true;
					edgesToRemove[(*it1)[1]][(*it1)[0]]=true;
				}
			}
			return edgesToRemove;
		}else{
			(*maxIt)[3]=0;	//mark edge with max intersections for removal
			for(it1=edges.begin();it1!=edges.end();it1++)
				//clean the intersection counters
				(*it1)[2]=0;

			for(it1=edges.begin();it1!=edges.end();it1++){	
				for(it2=it1+1;it2!=edges.end();it2++){
					//verify that edges have no vertex in common
					if( (*it1)[0]!=(*it2)[0] && (*it1)[0]!=(*it2)[1] && (*it1)[1]!=(*it2)[0] && (*it1)[1]!=(*it2)[1] && (*it1)[3]==1 && (*it2)[3]==1){
						pair<double,double> A1=pair<double,double>(newPoints[(*it1)[0]]->getX(), newPoints[(*it1)[0]]->getY());
						pair<double,double> A2=pair<double,double>(newPoints[(*it1)[1]]->getX(), newPoints[(*it1)[1]]->getY());
						pair<double,double> B1=pair<double,double>(newPoints[(*it2)[0]]->getX(), newPoints[(*it2)[0]]->getY());
						pair<double,double> B2=pair<double,double>(newPoints[(*it2)[1]]->getX(), newPoints[(*it2)[1]]->getY());
						if(isIntersecting(A1,A2,B1,B2)){
							//increase the intersections counter
							(*it1)[2]=(*it1)[2]+1;
							(*it2)[2]=(*it2)[2]+1;
						}
					}
				}
			}
		}
	}
}
