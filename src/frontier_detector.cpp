#include "frontier_detector.h"

using namespace sensor_msgs;

FrontierDetector::FrontierDetector (cv::Mat *image, float resolution, std::string namePoints, std::string nameMarkers, int thresholdSize, int thresholdNeighbors){

	_mapImage = image;
	_mapResolution = resolution;
	_sizeThreshold = thresholdSize;
	_neighborsThreshold = thresholdNeighbors;
	_topicPointsName = namePoints;
	_topicMarkersName = nameMarkers;

	_pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud2>(_topicPointsName,1);
	_pubCentroidMarkers = _nh.advertise<visualization_msgs::MarkerArray>( _topicMarkersName,1);

}



void FrontierDetector::computeFrontiers(){


	//1 compute frontier cells
	//2 Group frontier cells into contiguous regions
	//3 Crop regions which are too small
	//4 Compute centroid of each region


	_frontiers.clear();
	_regions.clear();


	for(int c = 0; c < _mapImage->cols; c++) {
    	for(int r = 0; r < _mapImage->rows; r++) {

    		if (_mapImage->at<unsigned char>(r, c) == _freeColor ){ 

    			std::array<float,2> coord = {r,c};

    			if (_mapImage->at<unsigned char>(r + 1, c) == _unknownColor ){ 
    				_frontiers.push_back(coord);	}
    			else if (_mapImage->at<unsigned char>(r - 1, c) == _unknownColor ) {
    				_frontiers.push_back(coord);		}
    			else if (_mapImage->at<unsigned char>(r, c + 1) == _unknownColor ) {
    				_frontiers.push_back(coord);		}    			
    			else if (_mapImage->at<unsigned char>(r, c - 1) == _unknownColor ) {
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
	//Assign a score to each region, according to its dimension and the number of close regions...
	//Weight the score according to the distance 
	//Move toward best centroid.
}


void FrontierDetector::publishFrontierPoints(){

	sensor_msgs::PointCloud2Ptr pointsMsg = boost::make_shared<sensor_msgs::PointCloud2>();
	
	pointsMsg->header.frame_id = "map";
	pointsMsg->is_bigendian = false;
	pointsMsg->is_dense = false;


	pointsMsg->width = _frontiers.size();
	pointsMsg->height = 1;


  	sensor_msgs::PointCloud2Modifier pcd_modifier(*pointsMsg);
  	pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*pointsMsg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*pointsMsg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*pointsMsg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pointsMsg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pointsMsg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pointsMsg, "b");

	for (int i = 0; i < _frontiers.size(); i++, ++iter_x, ++iter_y, ++iter_z,  ++iter_r, ++iter_g, ++iter_b){
		*iter_x = _frontiers[i][0]*_mapResolution;
		*iter_y = _frontiers[i][1]*_mapResolution;
		*iter_z = 0;

		*iter_r = 1;
        *iter_g = 0;
        *iter_b = 0;

		} 

	
	_pubFrontierPoints.publish(pointsMsg);


}


void FrontierDetector::publishCentroidMarkers(){
	visualization_msgs::MarkerArray markersMsg;

	coordVector centroids = computeCentroids();


	for (int i = 0; i < centroids.size(); i++){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		//marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = centroids[i][0] * _mapResolution;
		marker.pose.position.y = centroids[i][1] * _mapResolution;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.4;
		marker.scale.y = 0.4;
		marker.scale.z = 0.4;
		marker.color.a = 1.0; 
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		markersMsg.markers.push_back(marker);
	}

	_pubCentroidMarkers.publish(markersMsg);

}


coordVector FrontierDetector::computeCentroids(){

	coordVector centroids;

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


bool FrontierDetector::hasNeighbor(std::array<float,2> coordI, std::array<float,2> coordJ){

	if ((coordI[0] != coordJ[0]) || (coordI[1] != coordJ[1])){
		if ((abs(coordI[0] - coordJ[0]) <= _neighborsThreshold)&&(abs(coordI[1] - coordJ[1]) <= _neighborsThreshold)){
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

bool FrontierDetector::included(std::array<float,2> coord , regionVector regions){
	for (int i = 0; i < _regions.size(); i++){
		for (int j = 0; j < _regions[i].size(); j++){
			if (_regions[i][j] == coord){
				return true;
			}
		}
	}
	return false;

}