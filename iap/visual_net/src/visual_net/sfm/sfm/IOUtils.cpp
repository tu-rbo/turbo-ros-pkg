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
/*
 * IOUtils.cpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jkclevel
 */

#include "IOUtils.h"
using namespace boost::numeric::ublas;
namespace ublas = boost::numeric::ublas;
using namespace vision;

ublas::vector<double>** readControls(int numFrames, std::ifstream &input){
	 ublas::vector<double> **controls=0;

	  ublas::vector<double> currControl(6);
	  if( input.is_open() ){
		  int currFrame = 0;
		  controls = new ublas::vector<double>*[numFrames];

		  while( !input.eof() && currFrame < numFrames){
			  input>>currControl(0);
			  input>>currControl(1);
			  input>>currControl(2);
			  input>>currControl(3);
			  input>>currControl(4);
			  input>>currControl(5);

			 controls[currFrame] = new ublas::vector<double>(currControl);
		  }
		  input.close();
	  }  else{
		    std::cout << "Unable to open control file";
	  }

	  return controls;
}

std::vector<vision::FeaturePtr> *readObservations(int &numFeatures, int &numFrames, std::ifstream &input, int img_w, int img_h){
    int initFrame;

	return readObservations(numFeatures, numFrames, initFrame, input,img_w,img_h);
}

std::vector<vision::FeaturePtr> *readObservations(int &numFeatures, int &numFrames, int &initFrame, std::ifstream &input, int img_w , int img_h){
  //double f = 1/tan(21*M_PI/180);		  //for rescaling pixels to [-1:+1]
  //double px_size = (f*(5.6*0.001))/4.30;  //for rescaling pixels to [-1:+1]

  std::vector<FeaturePtr> *features=0;
  float x,y;
  if(input.is_open()){
    int counter=0;
    int index=0;
    input>>numFrames;
    input>>numFeatures;
    input>>initFrame;
    features=new std::vector<FeaturePtr>[numFrames];
    std::cout<<"Reading: "<<numFrames<<" frames, and "<<numFeatures<<" features init frame:"<<initFrame<<" img_w:"<<img_w<<" img_h:"<<img_h<<std::endl;

    while(!input.eof()){
      if(counter==numFeatures){
        counter=0;
	index++;
//		std::cout<<x<<" "<<y<<std::endl;
      }
      if(index==numFrames) break;
//swapped x and y to reflect possible error
	  input>>y;
     input>>x;

      x=(float)((x-img_w/2.0)/img_w)*2.0f;	//for rescaling pixels to [-1:+1]
      y=(float)((y-img_h/2.0)/img_w)*2.0f;	//for rescaling pixels to [-1:+1]
      if(fabs(x)>0.999 || fabs(y)>0.999) std::cout<<"f["<<counter<<"]=("<<x<<","<<y<<")"<<std::endl;

      FeaturePtr f(new Feature(x,y));

      f->setPosUncertainty(0.01f);
      features[index].push_back(f);
      counter++;
    }
    input.close();
  }
  else{
    std::cout << "Unable to open observation file";
  }

  return features;
}

std::vector<vision::FeaturePtr> *readCluster(int &numFeatures, int &numFrames, int &initFrame, std::ifstream &input,  int img_w , int img_h){
//  double f = 1/tan(21*M_PI/180);		  //for rescaling pixels to [-1:+1]
//  double px_size = (f*(5.6*0.001))/4.30;  //for rescaling pixels to [-1:+1]

  std::vector<FeaturePtr> *features=0;
  float x,y;
  if(input.is_open()){
    int counter=0;
    int index=0;
    input>>numFeatures;
    input>>numFrames;
    initFrame = 0;
    features=new std::vector<FeaturePtr>[numFrames];
    std::cout<<"Reading: "<<numFrames<<" frames, and "<<numFeatures<<" features"<<std::endl;

    while(!input.eof()){
      if(index==numFrames){
        counter++;
        index=0;
      }
      if(counter==numFeatures) break;
      input>>x;
      input>>y;

      x=(float)((x-img_w/2.0)/img_w)*2.0f;	//for rescaling pixels to [-1:+1]
      y=(float)((y-img_h/2.0)/img_w)*2.0f;	//for rescaling pixels to [-1:+1]

      FeaturePtr f(new Feature(x,y));

      f->setPosUncertainty(0.01f);
      features[index].push_back(f);
      index++;
    }
    input.close();
  }
  else{
    std::cout << "Unable to open observation file";
  }

  return features;
}


std::vector< std::vector<vision::FeaturePtr> > readCluster(int &numFeatures, int &numFrames, int &initFrame, std::string filename){

	std::ifstream input(filename.c_str());
  float x,y;
  if(input.is_open()){
    int feature_counter=0;
    int frame_index=0;
    input>>numFeatures;
    input>>numFrames;
    initFrame = 0;
    std::cout<<"Reading: "<<numFrames<<" frames, and "<<numFeatures<<" features"<<std::endl;
	std::vector<std::vector<FeaturePtr> > features(numFrames);

    while(!input.eof()){
      if(frame_index==numFrames){
    	feature_counter++;
        frame_index=0;
      }
      if(feature_counter==numFeatures) break;
      input>>x;
      input>>y;

      FeaturePtr f(new Feature(x,y));

      f->setPosUncertainty(0.01f);
      features[frame_index].push_back(f);
      frame_index++;
    }
    input.close();
    return features;
  }
  else{
   throw std::string("Unable to open cluster file");
  }

}

std::vector<FeaturePtr> *readPositions(int numFeatures, int numFrames, std::ifstream &input){
  std::vector<FeaturePtr> *features=0;
  float x,y,z;
  if(input.is_open()){
    int counter=0;
    int index=0;

    features=new std::vector<FeaturePtr>[numFrames];
    std::cout<<"Reading: "<<numFrames<<" frames, and "<<numFeatures<<" features"<<std::endl;

    while(!input.eof()){
      if(counter==numFeatures){
        counter=0;
        index++;
      }
      if(index==numFrames) break;

     input>>x;
     input>>y;
     input>>z;
     FeaturePtr f(new Feature(x,y,z));

     f->setPosUncertainty(0.01f);
     features[index].push_back(f);

      counter++;
    }
  }
  else{
    std::cout << "Unable to open file";
  }

  return features;
}
std::vector< std::vector<FeaturePtr> > read3dData(int numFeatures, int numFrames, std::ifstream &input){
	std::vector< std::vector<FeaturePtr> > features;
  float x,y,z;
  if(input.is_open()){
    int counter=0;
    int index=0;
    std::vector<FeaturePtr> frame_features;
    std::cout<<"Reading: "<<numFrames<<" frames, and "<<numFeatures<<" features"<<std::endl;
    while(!input.eof()){
      if(counter==numFeatures){
   	    features.push_back(frame_features);
   	    frame_features.clear();
        counter=0;
        index++;
      }
      if(index==numFrames) break;

     input>>x;
     input>>y;
     input>>z;
     FeaturePtr f(new Feature(x,y,z));

     f->setPosUncertainty(0.01f);
     frame_features.push_back(f);

      counter++;
    }
  }
  else{
    std::cout << "Unable to open file";
  }

  return features;
}


void writePositions(std::vector< std::vector<vision::FeaturePtr> > features, std::string filename)
{
	std::ofstream output(filename.c_str());
	if(output.is_open()){
		for(std::vector< std::vector<vision::FeaturePtr> >::iterator frame = features.begin(); frame!= features.end(); frame++)
		{
			for(std::vector<vision::FeaturePtr> ::iterator feature = frame->begin(); feature!= frame->end(); feature++)
			{
				output<<(*feature)->getX()<<" "<<(*feature)->getY()<<" "<<(*feature)->getZ()<<" ";
			}
			output<<std::endl;

		}
	}

}

void plotMatrix(boost::numeric::ublas::matrix<double> &m){
		for ( size_t i=0; i < m.size1(); ++i )
		{
			for(size_t j=0; j < m.size2(); ++j){
				std::cout<<m( i,j ) <<"\t";
			}
			std::cout<<std::endl;
		}
		std::cout<<std::endl;
}
