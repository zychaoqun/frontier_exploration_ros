#include "frontier_detector.h"


FrontierDetector::FrontierDetector (cv::Mat image, int threshold){

	_mapImage = image;
	_sizeThreshold = threshold;


}



void FrontierDetector::computeFrontiers(){


	//compute frontier cells
	//Group frontier cells into contiguous regions
	//Crop regions which are too small
	//Compute centroid of each region


}





void FrontierDetector::rankFrontiers(){}