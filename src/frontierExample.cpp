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



Mat image = imread(mapPath, CV_LOAD_IMAGE_UNCHANGED);

FrontierDetector frontiers(image,3);



}