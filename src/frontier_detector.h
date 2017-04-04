
#include <iostream>
#include <opencv2/highgui/highgui.hpp>


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



class FrontierDetector {

public:
	FrontierDetector (cv::Mat image, int threhsold = 3);

	void computeFrontiers();
	void rankFrontiers();






protected:
	cv::Mat _mapImage;
	int _sizeThreshold;




};