#include "frontier_detector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv){


std::string mapPath;

if (argc == 1){
	mapPath = "occupancygrid.png";	}
else if (argc == 2){
		mapPath = argv[1];	}
else 	{
		std::cout<<"Invalid number of arguments"<<std::endl;
		exit(0);	}



Mat map = imread(mapPath, CV_LOAD_IMAGE_UNCHANGED);

    if(! map.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;}


namedWindow( "original window", WINDOW_AUTOSIZE );// Create a window for display.
imshow( "original window", map );      




//Initialize ROS
ros::init(argc, argv, "frontier_detector");


string topic = "points";

FrontierDetector frontiers(map, topic , 10);

frontiers.computeFrontiers();


coordVector points = frontiers.getFrontierPoints();
regionVector regions = frontiers.getFrontierRegions();
floatCoordVector centroids = frontiers.computeCentroids();

std::cout<< "Frontier Points: " << points.size() <<std::endl;
std::cout<< "Frontier Regions: " << regions.size() <<std::endl; 


Mat frontierMap = cv::Mat(map.rows, map.cols, CV_8UC1);
frontierMap.setTo(cv::Scalar(0));

for (int i = 0; i < points.size(); i++){

int x = points[i][0];
int y = points[i][1];

frontierMap.at<unsigned char>(x, y) = 255;

}

Mat regionsMap = cv::Mat(map.rows, map.cols, CV_8UC1);
regionsMap.setTo(cv::Scalar(0));

for (int i = 0; i < regions.size(); i++){
	for (int j = 0; j<regions[i].size(); j++){

int x = regions[i][j][0];
int y = regions[i][j][1];

regionsMap.at<unsigned char>(x, y) = 255;

		}

	
	int x = round(centroids[i][0]);
	int y = round(centroids[i][1]);
	circle(regionsMap, Point(y, x), 5, 200);
	std::cout<<x<< "  "<<y<<std::endl;
}



namedWindow( "Regions window", WINDOW_AUTOSIZE );// Create a window for display.
imshow( "Regions window", regionsMap ); 

imwrite("image.jpg", regionsMap);

waitKey(0);  


/*
ros::Rate loop_rate(10);
while (ros::ok()){

 	frontiers.publishFrontierPoints();

 	loop_rate.sleep();

 	}*/


}