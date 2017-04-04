#include "frontier_detector.h"


FrontierDetector::FrontierDetector (cv::Mat image, int threshold){

	_mapImage = image;
	_sizeThreshold = threshold;


}



void FrontierDetector::computeFrontiers(){


	//1 compute frontier cells
	//2 Group frontier cells into contiguous regions
	//2 Crop regions which are too small
	//3 Compute centroid of each region




	for(int c = 0; c < _mapImage.cols; c++) {
    	for(int r = 0; r < _mapImage.rows; r++) {

    		if (_mapImage.at<unsigned char>(r, c) == _freeColor ){ 

    			std::array<int,2> coord = {r,c};

    			if (_mapImage.at<unsigned char>(r + 1, c) == _unknownColor ){ 
    				_frontiers.push_back(coord);	}
    			else if (_mapImage.at<unsigned char>(r - 1, c) == _unknownColor ) {
    				_frontiers.push_back(coord);		}
    			else if (_mapImage.at<unsigned char>(r, c + 1) == _unknownColor ) {
    				_frontiers.push_back(coord);		}    			
    			else if (_mapImage.at<unsigned char>(r, c - 1) == _unknownColor ) {
    				_frontiers.push_back(coord);		}
    		
    						}

    				}
    			}


    //_frontiers.resize(50);

    for (int i = 0; i < _frontiers.size(); i++){

    	coordVector region = {};

    	if (included(_frontiers[i], _regions) == false){ //I proceed only if the current coord has not been already considered
    													//doesn't consider the failed regions..... (not needed iterations)
    		
	    	region.push_back(_frontiers[i]);
			
			for (int j = i; j < _frontiers.size(); j++){

				/*coordVector::size_type size = region.size();
				for (coordVector::size_type k = 0; k < size; k++){
		    		if (isNeighbor(region[k],_frontiers[j])){ //If the point is the same return false
		    			region.push_back(_frontiers[j]);
		    			k = k + 1;

		    			}

	    		}*/

	    		for (int k = 0; k < region.size(); k++){
	    			if (hasNeighbor(region[k], _frontiers[j])){
	    				region.push_back(_frontiers[j]);
	    				//std::cout<<i<< " "<<j<< " ... ";
	    				break;
	    			}
	    		}

    					}


    	if (region.size() >= _sizeThreshold){
	    	//std::cout<<std::endl;	
	    	//std::cout<<i <<" YES with "<< region.size()<<std::endl;

	    	_regions.push_back(region);
	    								}

    			
    				}
    			}



}





void FrontierDetector::rankFrontiers(){}



coordVector FrontierDetector::getFrontierPoints(){
	return _frontiers;
}

regionVector FrontierDetector::getFrontierRegions(){
	return _regions;
}


bool FrontierDetector::hasNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ){
	if (abs(abs(coordI[0] - coordJ[0]) + abs(coordI[1] - coordJ[1])) == 1 )
		return true;
	else 
		return false;
}

bool FrontierDetector::included(std::array<int,2> coord , regionVector regions){
	for (int i = 0; i < _regions.size(); i++){
		for (int j = 0; j < _regions[i].size(); j++){
			if (_regions[i][j] == coord){
				return true;
			}
		}
	}
	return false;

}