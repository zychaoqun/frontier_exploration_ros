
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h> 

#include "g2o/core/hyper_graph.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"



typedef std::vector<std::array<int,2>> coordVector;
typedef std::vector<coordVector> regionVector;

class FrontierDetector {

public:
	FrontierDetector (cv::Mat image, int threhsold = 3);

	void computeFrontiers();
	void rankFrontiers();

	coordVector getFrontierPoints();
	regionVector getFrontierRegions();






protected:

	bool hasNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ);
	bool included(std::array<int,2> coord , regionVector regions);



	//Is neighbor inline private method

	cv::Mat _mapImage;
	int _sizeThreshold;

	int _freeColor = 255;
	int _occupiedColor = 0;
	int _unknownColor = 127;

	coordVector _frontiers;
	regionVector _regions;



};