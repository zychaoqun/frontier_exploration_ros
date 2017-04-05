#include "frontier_detector.h"


FrontierDetector::FrontierDetector (cv::Mat image, std::string name, int threshold){

	_mapImage = image;
	_sizeThreshold = threshold;
	_topicPointsName = name;

	_pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud>(_topicPointsName,1);


}



void FrontierDetector::computeFrontiers(){


	//1 compute frontier cells
	//2 Group frontier cells into contiguous regions
	//3 Crop regions which are too small
	//4 Compute centroid of each region




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

    for (int i = 0; i < _frontiers.size(); i++){

    	coordVector region = {};
    	if (included(_frontiers[i], _regions) == false){ //I proceed only if the current coord has not been already considered
    													//doesn't consider the failed regions..... (not needed iterations)
    		
	    	region.push_back(_frontiers[i]);
			
			for (int j = i + 1; j < _frontiers.size(); j++){

	    		for (int k = 0; k < region.size(); k++){
	    			if (hasNeighbor(region[k], _frontiers[j])){
	    				region.push_back(_frontiers[j]);
	    				break;								}
	    											}

    									}

	    	if (region.size() >= _sizeThreshold)
		    	_regions.push_back(region);
		    								
    							}
    				
    					}



}





void FrontierDetector::rankRegions(){
	//Reorder the _regions vector
}


void FrontierDetector::publishFrontierPoints(){

	sensor_msgs::PointCloud pointsMsg;

	//header (uint32 seq, time stamp, string frame_id)

	//points[] (float32 x, y, z)
	//channels[] (string name, float32[] values)
	for (int i = 0; i < _frontiers.size(); i++){
		geometry_msgs::Point32 point;
		point.x = _frontiers[i][0];
		point.y = _frontiers[i][1];
		point.z = 0;

		pointsMsg.points.push_back(point);

		
		sensor_msgs::ChannelFloat32 channel;
		channel.name = "rgb";
		channel.values.push_back(0);
		channel.values.push_back(0);
		channel.values.push_back(255);

		pointsMsg.channels.push_back(channel);
		}

	
_pubFrontierPoints.publish(pointsMsg);


}


floatCoordVector FrontierDetector::computeCentroids(){

	floatCoordVector centroids;

	for (int i = 0; i < _regions.size(); i++){

		float accX = 0;
		float accY = 0;

		for (int j = 0; j <_regions[i].size(); j++){
			accX+=_regions[i][j][0];
			accY+=_regions[i][j][1];
										}


		float meanX = accX/_regions[i].size();
		float meanY = accY/_regions[i].size();

		std::array<float,2> centroid = {meanX, meanY};

		centroids.push_back(centroid);

						}

return centroids;

}


coordVector FrontierDetector::getFrontierPoints(){
	return _frontiers;	}

regionVector FrontierDetector::getFrontierRegions(){
	return _regions;	}


bool FrontierDetector::hasNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ){

	if ((coordI[0] != coordJ[0]) || (coordI[1] != coordJ[1])){
		if ((abs(coordI[0] - coordJ[0]) <= 1)&&(abs(coordI[1] - coordJ[1]) <= 1)){
			return true; 								
							}
					}	
		

	return false;

	/*if (abs(abs(coordI[0] - coordJ[0]) + abs(coordI[1] - coordJ[1])) == 1 ){
		return true;
	}
	else 
		return false;*/
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