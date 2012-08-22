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
//
//#include "image.h"
//#include "videoSensor.h"
//
//#include "ColorSegmentation.h"
////
//////#include "imageVN.h"
////
////#include "pixel.h"
////#include "feature.h"
////
////#include <vector>
////#include <iostream>
////#include <time.h>
////#include <cmath>
////
////#include <boost/numeric/ublas/matrix.hpp>
////#include <boost/numeric/ublas/io.hpp>
//
//
//using namespace std;
//
//using namespace vision;
//
//
//
//namespace VisualGraph{
//
//ColorSegmentation::ColorSegmentation(int frame, string &videoName, int imgWidth, int imgHeight) : Predictor(frame, -1, string("ColorSegmentation")){
//	this->imgWidth=imgWidth;
//	this->imgHeight=imgHeight;
//	this->videoName=videoName;
//}
//
//ColorSegmentation::~ColorSegmentation(void){
//}
//
//
//void ColorSegmentation::compute(){
//	Sensor* sens = new VideoSensor(videoName.c_str());
//	(dynamic_cast<VideoSensor*>(sens))->step(frame1);
//	Image* lastFrame = ((dynamic_cast<VideoSensor*>(sens))->getCurrentImage())->clone();
//	lastFrame->saveImage(string("toSegment.jpg"));
//
//	//0.5 500 500
//	float sigma=(float)0.5;
//	float k=100; //300
//	int min_size=400; //200
//
//	Image *seg=lastFrame->graphSegment(sigma,k,min_size);
//	seg->saveImage(string("segmented.jpg"));
//
//	cvSaveImage("segmented.ppm", seg->getIplImage());
//	//seg->saveAsPPM(string("segmented.ppm"));
//
//	//a map of colors
//	map<Pixel,int> colors;
//	for(int i=0;i<seg->getHeight();i++){
//		for(int j=0;j<seg->getWidth();j++){
//			Pixel p=seg->get2D(i,j);
//			if(colors.find(p)==colors.end()){
//				colors[p]=1;
//			}else{
//				colors[p]=colors.find(p)->second + 1;
//			}
//		}
//	}
//
//	//make a matrix of colored regions and their neighbors
//	int size=colors.size();
//	int **colorNeighbors=new int*[size];
//	for(int i=0;i<size;i++) colorNeighbors[i]=new int[size];
//	for(int i=0;i<size;i++){
//		for(int j=0;j<size;j++){
//			colorNeighbors[i][j]=0;
//		}
//	}
//
//	for(int i=1;i<seg->getHeight();i++){
//		for(int j=1;j<seg->getWidth();j++){
//			Pixel p=seg->get2D(i,j);
//			int pindex=0;
//			for(map<Pixel,int>::iterator it=colors.begin();it!=colors.end();it++){
//				if(it->first==p)
//					break;
//				else
//					pindex++;
//			}
//
//			for(int a=-1;a<2;a++){
//				for(int b=-1;b<2;b++){
//					if(!(i+a<1 || i+a>=seg->getHeight() || j+b<1 || j+b>=seg->getWidth())){
//						Pixel n=seg->get2D(i+a,j+b);
//						if(!(p==n)){
//							int nindex=0;
//							for(map<Pixel,int>::iterator it=colors.begin();it!=colors.end();it++){
//							if(it->first==n)
//								break;
//							else
//								nindex++;
//							}
//							colorNeighbors[pindex][nindex]=colorNeighbors[pindex][nindex]+1;
//							colorNeighbors[nindex][pindex]=colorNeighbors[nindex][pindex]+1;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	//go over pairs of features, if in the same region, reinforce a lot or add edge
//	//if in neighboring regions reinforce some or add edge
//	//if not neighbors, decrease edge strength if exists
//	for(int i=0;i<numVertices;i++){
//		for(int j=i+1;j<numVertices;j++){
//
//			int indexI=0;
//			Pixel p=seg->get2D(imgHeight-(int)vList[i]->getFeatureData(frame1).getY()-1,(int)vList[i]->getFeatureData(frame1).getX());
//			for(map<Pixel,int>::iterator it=colors.begin();it!=colors.end();it++){
//				if(it->first==p){
//					   break;
//				}else{
//					indexI++;
//				}
//			}
//
//			int indexJ=0;
//			Pixel q=seg->get2D(imgHeight-(int)vList[j]->getFeatureData(frame1).getY()-1,(int)vList[j]->getFeatureData(frame1).getX());
//			for(map<Pixel,int>::iterator it=colors.begin();it!=colors.end();it++){
//				if(it->first==q)
//				   break;
//				else
//					indexJ++;
//			}
//
//
//			if(colorNeighbors[indexI][indexJ]>3){
//				//neighbor colors, so reinforce edge or add it
//				//adjMatrix[i][j]->setCapacity(predictorName,0.01);
//				//adjMatrix[j][i]->setCapacity(predictorName,0.01);
//			}else if(indexI==indexJ){
//				//same color
//				adjMatrix[i][j]->setCapacity(predictorName,1.0);
//				adjMatrix[j][i]->setCapacity(predictorName,1.0);
//			}else{
//				//not color-neighbors
//				//adjMatrix[i][j]->setCapacity(predictorName,0.0);
//				//adjMatrix[j][i]->setCapacity(predictorName,0.0);
//			}
//		}
//	}
//}
//
//};
