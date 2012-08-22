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
* SURFTracker Model (implementation)
******************************************************************************
* $Author: roberto $
* $Date: 2010/09/06 12:51:04 $
* $Revision: 1.0 $
*****************************************************************************/

#include "surfTracker.h"
#include "feature.h"
#include "videoSensor.h"
#include "featureDetector.h"
#include "image.h"

#include "highgui.h"
#include "cv.h"
#include <iostream>
#include <string>

using namespace vision;
using std::vector;

SURFTracker::SURFTracker() : Tracker()
{
	_isInit=false;
	_isFreshImage = false;
	_surfDetector = NULL;
}
// ============================================================================

SURFTracker::~SURFTracker()
{

	std::vector<FeaturePtr>::const_iterator featuresIt =
		Tracker::features.begin();

	features.clear();

	if(Tracker::curImage){
		delete Tracker::curImage;
		Tracker::curImage = 0;
	}
	Tracker::notifyResetListeners();
}
// ============================================================================

int SURFTracker::getCount()
{
	return _count;
}
// ============================================================================

void SURFTracker::notify(const Sensor* sens){

	const VideoSensor* vSens = (dynamic_cast<const VideoSensor*>(sens));
	Image* newImage = vSens->getCurrentImage()->clone();

	if(!_isInit){
		IplImage* newIImage = newImage->getIplImage();
		_imgSize =       cvGetSize(newIImage);
		_isInit = true;
	}

	if(Tracker::curImage){
		delete Tracker::curImage;
		Tracker::curImage = 0;
	}
	Tracker::curImage = newImage;
	_isFreshImage = true;
}


// ============================================================================

void SURFTracker::resetFeatures(FeatureDetector* d)
{
	if(Tracker::curImage == 0){
		throw std::string(
			"there is no image on wich to detect features");
	}

	// BEGIN: delete the features that may have been detected preveously
	std::vector<FeaturePtr>::const_iterator featuresItD =
		Tracker::features.begin();
	while(featuresItD != Tracker::features.end()){
		SURFFeaturePtr sf = boost::dynamic_pointer_cast<SURFFeature>(*featuresItD);	//Don't know if delete works properly when passed a non-casted pointer
		featuresItD++;
	}
	Tracker::features.clear();
	// END: delete

	// detect features on the last known image
	Tracker::features = d->detect(Tracker::curImage);
	_count = Tracker::features.size();

	_surfDetector = new SURFDetector(*(dynamic_cast<SURFDetector*>(d)));

	Tracker::ready = true;
	Tracker::notifyResetListeners();
}
// ============================================================================

void flannFindPairs2(vector<SURFFeaturePtr> &prevFeatures, vector<SURFFeaturePtr> &currFeatures, vector<int> &ptpairs){
	//object=prev, image=curr
	unsigned int length=(*prevFeatures.begin())->descriptor.size();	//we assume all descriptors have the same length

	cv::Mat m_object(prevFeatures.size(), length, CV_32F);
	cv::Mat m_image(currFeatures.size(), length, CV_32F);

	// copy descriptors
	float* ptr = m_object.ptr<float>(0);
	for(unsigned int i=0;i<prevFeatures.size();i++){
		float *desc=new float[length];
		for(unsigned int j=0;j<length;j++){
			desc[j]=prevFeatures[i]->descriptor[j];
		}
		memcpy(ptr, desc, length*sizeof(float));
		delete(desc);
		ptr += length;
	}

	ptr = m_image.ptr<float>(0);
	for(unsigned int i=0;i<currFeatures.size();i++){
		float *desc=new float[length];
		for(unsigned int j=0;j<length;j++){
			desc[j]=currFeatures[i]->descriptor[j];
		}
		memcpy(ptr, desc, length*sizeof(float));
		delete(desc);
		ptr += length;
	}

	// find nearest neighbors using FLANN
	cv::Mat m_indices(prevFeatures.size(), 2, CV_32S);
	cv::Mat m_dists(prevFeatures.size(), 2, CV_32F);
	cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
	flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked

	int* indices_ptr = m_indices.ptr<int>(0);
	float* dists_ptr = m_dists.ptr<float>(0);
	for (int i=0;i<m_indices.rows;++i) {
		if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
			ptpairs.push_back(i);
			ptpairs.push_back(indices_ptr[2*i]);
		}
	}
}

void SURFTracker::step()
{
	if(!_isFreshImage){
		throw std::string(
			"You need to call step() on the sensor first");
	}

	if(Tracker::ready && _count > 0){
		if(!_surfDetector){
			//We detect more features in the destImage, to have higher probability of tracking success
			_surfDetector = new SURFDetector(_count);
		}
		vector<FeaturePtr> featuresDetected = _surfDetector->detect(Tracker::curImage, 0);
		std::cout << featuresDetected.size() <<std::endl;
		//find the pairs of features (feature # in the object, feature # in the scene)
		vector<int> ptpairs;

		//flannFindPairs2 accepts only vector<SURFFeatures*>!!
		vector<SURFFeaturePtr> surfFeaturesCurr;
		vector<FeaturePtr>::iterator fi = featuresDetected.begin();
		for(;fi<featuresDetected.end();fi++){
			SURFFeaturePtr sfactual = boost::dynamic_pointer_cast<SURFFeature>(*fi);
			surfFeaturesCurr.push_back(sfactual);
		}

		//flannFindPairs2 accepts only vector<SURFFeatures*>!!
		vector<SURFFeaturePtr> surfFeaturesPrev;
		vector<FeaturePtr>::iterator fpi = this->features.begin();
		for(;fpi<this->features.end();fpi++){
			if(!((*fpi)->isLost())){
				SURFFeaturePtr sfprevious = boost::dynamic_pointer_cast<SURFFeature>(*fpi);
				surfFeaturesPrev.push_back(sfprevious);
			}
		}


		flannFindPairs2(surfFeaturesPrev, surfFeaturesCurr, ptpairs);

		//create a vector of mappings (cannot exceed the number of features in the previous frame)
		int *detected = new int[surfFeaturesPrev.size()];
		for(unsigned int i=0;i<surfFeaturesPrev.size();i++){
			detected[i]=-1;
		}

		for(unsigned int i=0;i<ptpairs.size();i+=2){
			detected[ptpairs[i]]=ptpairs[i+1];
		}

		int found = 0;
		int lost = 0;
		unsigned int j=0;
		for(;j<surfFeaturesPrev.size();j++){
			if(detected[j]!=-1 && !surfFeaturesPrev[j]->isLost()){ //Feature tracked
				this->features[j]->setLost(false);
				this->features[j]->setPos(surfFeaturesCurr[detected[j]]->getX(), _imgSize.height - surfFeaturesCurr[detected[j]]->getY());
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->laplacian = surfFeaturesCurr[detected[j]]->laplacian;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->size= surfFeaturesCurr[detected[j]]->size;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->dir= surfFeaturesCurr[detected[j]]->dir;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->hessian= surfFeaturesCurr[detected[j]]->hessian;	
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->descriptor= surfFeaturesCurr[detected[j]]->descriptor;	
				found+=1;
			}else{	//Feature not tracked
				this->features[j]->setLost(true);
				this->features[j]->setPos(0,0);
				this->features[j]->setError(-1);
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->laplacian=0;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->size=0;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->dir=0;
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->hessian=0;	
				boost::dynamic_pointer_cast<SURFFeature>(this->features[j])->descriptor.clear();
				lost+=1;
			}
		}
		std::cout << "Pair features found = " << found << std::endl;
		std::cout << "Pair features lost = " << lost << std::endl;	
		_count = found;
	}

	Tracker::notifyStepListeners();

	_isFreshImage = false;
}

pair<vector<SURFFeaturePtr>,vector<SURFFeaturePtr> > SURFTracker::findSURFPairs(Image* destImage){
	if(!_isFreshImage){
		throw std::string(
			"You need to call step() on the sensor first");
	}
	if(Tracker::ready && _count > 0){
		//We detect more features in the destImage, to have higher probability of tracking success
		SURFDetector* surfDetectorTemp = new SURFDetector(2*_count);
		vector<FeaturePtr> featuresDetected = surfDetectorTemp->detect(destImage, 0);

		//find the pairs of features (feature # in the object, feature # in the scene)
		vector<int> ptpairs;

		//flannFindPairs2 accepts only vector<SURFFeatures*>!!
		vector<SURFFeaturePtr> surfFeaturesCurr;
		vector<FeaturePtr>::iterator fi = featuresDetected.begin();
		for(;fi<featuresDetected.end();fi++){
			SURFFeaturePtr sfactual = boost::dynamic_pointer_cast<SURFFeature>(*fi);
			surfFeaturesCurr.push_back(sfactual);
		}

		//flannFindPairs2 accepts only vector<SURFFeatures*>!!
		//Try to find pairs only from the non-lost previous features
		vector<SURFFeaturePtr> surfFeaturesPrev;
		vector<FeaturePtr>::iterator fpi = this->features.begin();
		for(;fpi<this->features.end();fpi++){
			if(!((*fpi)->isLost())){
				SURFFeaturePtr sfprevious = boost::dynamic_pointer_cast<SURFFeature>(*fpi);
				surfFeaturesPrev.push_back(sfprevious);
			}
		}


		flannFindPairs2(surfFeaturesPrev, surfFeaturesCurr, ptpairs);

		//create a vector of mappings (cannot exceed the number of features in the previous frame)
		int *detected = new int[surfFeaturesPrev.size()];
		for(unsigned int i=0;i<surfFeaturesPrev.size();i++){
			detected[i]=-1;
		}

		for(unsigned int i=0;i<ptpairs.size();i+=2){
			detected[ptpairs[i]]=ptpairs[i+1];
		}

		int found = 0;
		int lost = 0;
		unsigned int j=0;
		std::vector<SURFFeaturePtr> foundPairs;
		for(;j<surfFeaturesPrev.size();j++){
			if(detected[j]!=-1){ //Feature tracked
				foundPairs.push_back(SURFFeaturePtr(new SURFFeature(surfFeaturesCurr[detected[j]]->getX(), _imgSize.height - surfFeaturesCurr[detected[j]]->getY(), surfFeaturesCurr[detected[j]]->laplacian, surfFeaturesCurr[detected[j]]->size, surfFeaturesCurr[detected[j]]->dir, surfFeaturesCurr[detected[j]]->hessian, surfFeaturesCurr[detected[j]]->descriptor, false)));
				found++;
				j++;
			}else{	//Feature not tracked
				std::vector<float> null_descriptor;
				foundPairs.push_back(SURFFeaturePtr(new SURFFeature(0, 0, 0, 0, 0, 0, null_descriptor, true)));
				lost++;
				j++;
			}
		}
		std::cout << "Pair features found = " << found << std::endl;
		std::cout << "Pair features lost = " << lost << std::endl;	
		return std::pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> >(surfFeaturesPrev, foundPairs);
	}
}

pair<vector<SURFFeaturePtr>,vector<SURFFeaturePtr> > SURFTracker::findSURFPairs(Image* sourceImage, Image* destImage, int numberFeatures){
	SURFDetector* surfDetectorTemp = new SURFDetector(numberFeatures);
	vector<FeaturePtr> featuresDetectedSource = surfDetectorTemp->detect(sourceImage, 0);
	//We detect more features in the destImage, to have higher probability of tracking success
	surfDetectorTemp = new SURFDetector(2*numberFeatures);	
	vector<FeaturePtr> featuresDetectedDest = surfDetectorTemp->detect(destImage, 0);
	std::cout << featuresDetectedDest.size() <<std::endl;
	//find the pairs of features (feature # in the object, feature # in the scene)
	vector<int> ptpairs;

	//flannFindPairs2 accepts only vector<SURFFeatures*>!!
	vector<SURFFeaturePtr> surfFeaturesCurr;
	vector<FeaturePtr>::iterator fi = featuresDetectedDest.begin();
	for(;fi<featuresDetectedDest.end();fi++){
		SURFFeaturePtr sfactual = boost::dynamic_pointer_cast<SURFFeature>(*fi);
		surfFeaturesCurr.push_back(sfactual);
	}

	//flannFindPairs2 accepts only vector<SURFFeatures*>!!
	vector<SURFFeaturePtr> surfFeaturesPrev;
	vector<FeaturePtr>::iterator fpi = featuresDetectedSource.begin();
	for(;fpi<featuresDetectedSource.end();fpi++){
		SURFFeaturePtr sfprevious = boost::dynamic_pointer_cast<SURFFeature>(*fpi);
		surfFeaturesPrev.push_back(sfprevious);
	}


	flannFindPairs2(surfFeaturesPrev, surfFeaturesCurr, ptpairs);

	//create a vector of mappings (cannot exceed the number of features in the previous frame)
	int *detected = new int[surfFeaturesPrev.size()];
	for(unsigned int i=0;i<surfFeaturesPrev.size();i++){
		detected[i]=-1;
	}

	for(unsigned int i=0;i<ptpairs.size();i+=2){
		detected[ptpairs[i]]=ptpairs[i+1];
	}

	int found = 0;
	int lost = 0;
	unsigned int j=0;
	std::vector<SURFFeaturePtr> foundPairs;
	for(;j<surfFeaturesPrev.size();j++){
		if(detected[j]!=-1){ //Feature tracked
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(surfFeaturesCurr[detected[j]]->getX(), _imgSize.height - surfFeaturesCurr[detected[j]]->getY(), surfFeaturesCurr[detected[j]]->laplacian, surfFeaturesCurr[detected[j]]->size, surfFeaturesCurr[detected[j]]->dir, surfFeaturesCurr[detected[j]]->hessian, surfFeaturesCurr[detected[j]]->descriptor, false)));
			found++;
		}else{	//Feature not tracked
			std::vector<float> null_descriptor;
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(0, 0, 0, 0, 0, 0, null_descriptor, true)));
			lost++;
		}
	}
	std::cout << "Pair features found = " << found << std::endl;
	std::cout << "Pair features lost = " << lost << std::endl;	
	return std::pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> >(surfFeaturesPrev, foundPairs);
}

pair<vector<SURFFeaturePtr>,vector<SURFFeaturePtr> > SURFTracker::findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, Image* destImage){
	//We detect more features in the destImage, to have higher probability of tracking success
	SURFDetector* surfDetectorTemp = new SURFDetector(2*sourceFeatures.size());

	vector<FeaturePtr> featuresDetectedDest = surfDetectorTemp->detect(destImage, 0);
	std::cout << featuresDetectedDest.size() <<std::endl;
	//find the pairs of features (feature # in the object, feature # in the scene)
	vector<int> ptpairs;

	//flannFindPairs2 accepts only vector<SURFFeatures*>!!
	vector<SURFFeaturePtr> surfFeaturesCurr;
	vector<FeaturePtr>::iterator fi = featuresDetectedDest.begin();
	for(;fi<featuresDetectedDest.end();fi++){
		SURFFeaturePtr sfactual = boost::dynamic_pointer_cast<SURFFeature>(*fi);
		surfFeaturesCurr.push_back(sfactual);
	}

	//Suppossing that the given sourceFeatures vector does not contain lost features
	flannFindPairs2(sourceFeatures, surfFeaturesCurr, ptpairs);

	//create a vector of mappings (cannot exceed the number of features in the previous frame)
	int *detected = new int[sourceFeatures.size()];
	for(unsigned int i=0;i<sourceFeatures.size();i++){
		detected[i]=-1;
	}

	for(unsigned int i=0;i<ptpairs.size();i+=2){
		detected[ptpairs[i]]=ptpairs[i+1];
	}

	int found = 0;
	int lost = 0;
	unsigned int j=0;
	std::vector<SURFFeaturePtr> foundPairs;
	for(;j<sourceFeatures.size();j++){
		if(detected[j]!=-1){ //Feature tracked
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(surfFeaturesCurr[detected[j]]->getX(), _imgSize.height - surfFeaturesCurr[detected[j]]->getY(), surfFeaturesCurr[detected[j]]->laplacian, surfFeaturesCurr[detected[j]]->size, surfFeaturesCurr[detected[j]]->dir, surfFeaturesCurr[detected[j]]->hessian, surfFeaturesCurr[detected[j]]->descriptor, false)));
			found++;
		}else{	//Feature not tracked
			std::vector<float> null_descriptor;
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(0, 0, 0, 0, 0, 0, null_descriptor, true)));
			lost++;
		}
	}
	std::cout << "Pair features found = " << found << std::endl;
	std::cout << "Pair features lost = " << lost << std::endl;
	return std::pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> >(sourceFeatures, foundPairs);
}


//CHANGE!!!!!!!! The first vector of the pair is no longer the sourceFeatures vector, but the sourceFeatures vector without the lost features
//CHANGE2!!!!!!! In a first version, the first vector of the returned pair was sourceFeatures with 0 feature where a pair was not found
//				In the second version, the returned pair is formed by 2 vector containing ONLY the found pairs
pair<vector<SURFFeaturePtr>,vector<SURFFeaturePtr> > SURFTracker::findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, const Image* destImage){
	//We detect more features in the destImage, to have higher probability of tracking success
	SURFDetector* surfDetectorTemp = new SURFDetector(2*sourceFeatures.size());

	vector<FeaturePtr> featuresDetectedDest = surfDetectorTemp->detect(destImage, 0);
	std::cout << "Number of candidates to be pairs "<< featuresDetectedDest.size() <<std::endl;
	//find the pairs of features (feature # in the object, feature # in the scene)
	vector<int> ptpairs;

	//flannFindPairs2 accepts only vector<SURFFeatures*>!!
	vector<SURFFeaturePtr> surfFeaturesCurr;
	vector<FeaturePtr>::iterator fi = featuresDetectedDest.begin();
	for(;fi<featuresDetectedDest.end();fi++){
		SURFFeaturePtr sfactual = boost::dynamic_pointer_cast<SURFFeature>(*fi);
		surfFeaturesCurr.push_back(sfactual);
	}

	//Suppossing that the given sourceFeatures vector does not contain lost features
	flannFindPairs2(sourceFeatures, surfFeaturesCurr, ptpairs);

	//create a vector of mappings (cannot exceed the number of features in the previous frame)
	int *detected = new int[sourceFeatures.size()];
	for(unsigned int i=0;i<sourceFeatures.size();i++){
		detected[i]=-1;
	}

	for(unsigned int i=0;i<ptpairs.size();i+=2){
		detected[ptpairs[i]]=ptpairs[i+1];
	}

	int found = 0;
	int lost = 0;
	unsigned int j=0;
	std::vector<SURFFeaturePtr> foundPairsFirst;
	std::vector<SURFFeaturePtr> foundPairsSecond;
	for(;j<sourceFeatures.size();j++){
		if(detected[j]!=-1){ //Pair found
			foundPairsFirst.push_back(sourceFeatures[j]);	
			foundPairsSecond.push_back(SURFFeaturePtr(new SURFFeature(surfFeaturesCurr[detected[j]]->getX(), surfFeaturesCurr[detected[j]]->getY(), surfFeaturesCurr[detected[j]]->laplacian, surfFeaturesCurr[detected[j]]->size, surfFeaturesCurr[detected[j]]->dir, surfFeaturesCurr[detected[j]]->hessian, surfFeaturesCurr[detected[j]]->descriptor, false)));
			found++;
		}else{	//Pair not found
			lost++;
		}
	}
	std::cout << "Pair features found = " << found << std::endl;
	std::cout << "Pair features lost = " << lost << std::endl;
	return std::pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> >(foundPairsFirst, foundPairsSecond);
}

pair<vector<SURFFeaturePtr>,vector<SURFFeaturePtr> > SURFTracker::findSURFPairs(std::vector<SURFFeaturePtr> sourceFeatures, std::vector<SURFFeaturePtr> destFeatures){

	//find the pairs of features (feature # in the object, feature # in the scene)
	vector<int> ptpairs;

	flannFindPairs2(sourceFeatures, destFeatures, ptpairs);

	//create a vector of mappings (cannot exceed the number of features in the previous frame)
	int *detected = new int[sourceFeatures.size()];
	for(unsigned int i=0;i<sourceFeatures.size();i++){
		detected[i]=-1;
	}

	for(unsigned int i=0;i<ptpairs.size();i+=2){
		detected[ptpairs[i]]=ptpairs[i+1];
	}

	int found = 0;
	int lost = 0;
	unsigned int j=0;
	std::vector<SURFFeaturePtr> foundPairs;
	for(;j<sourceFeatures.size();j++){
		if(detected[j]!=-1){ //Feature tracked
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(destFeatures[detected[j]]->getX(), _imgSize.height - destFeatures[detected[j]]->getY(), destFeatures[detected[j]]->laplacian, destFeatures[detected[j]]->size, destFeatures[detected[j]]->dir, destFeatures[detected[j]]->hessian, destFeatures[detected[j]]->descriptor, false)));
			found++;
		}else{	//Feature not tracked
			std::vector<float> null_descriptor;
			foundPairs.push_back(SURFFeaturePtr(new SURFFeature(0, 0, 0, 0, 0, 0, null_descriptor, true)));
			lost++;
		}
	}
	std::cout << "Pair features found = " << found << std::endl;
	std::cout << "Pair features lost = " << lost << std::endl;
	return std::pair<std::vector<SURFFeaturePtr>,std::vector<SURFFeaturePtr> >(sourceFeatures, foundPairs);
}

std::vector<FeaturePtr> SURFTracker::convertForLK(){
	if(!(this->features.size()>0)){
		throw std::string(
			"There are no features to pass");
	}
	std::vector<FeaturePtr> retVal;
	vector<FeaturePtr>::iterator fi = this->features.begin();
	for(;fi<this->features.end();fi++){
		if(!(*fi)->isLost()){
			retVal.push_back((*fi));
		}
	}
	return retVal;
}
/** ***************************************************************************
End of File
******************************************************************************/

