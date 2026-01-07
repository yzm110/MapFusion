#include <iostream>
#include <algorithm>
#include <vector>

// #include "topic/topic_trait.h"
#include "common/framework.h"
// #include "codeconfig.h"
// #include "message.h"
#include "compile_config.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
// #include "utils/common.h"
// #include "utils/decision.h"
// #include "utils/trajectory.h"
// #include "utils/standard_scenario_model.h"


//#include "lat_interface.h"

namespace CommuteMode {
namespace MapFusion {



void CrossPlanning(const std::vector<float> &Lane_1_x,const std::vector<float> &Lane_1_y,
                const std::vector<float> &Lane_2_x,const std::vector<float> &Lane_2_y,
                std::vector<float> &Lane_out_x,std::vector<float> &Lane_out_y);

Eigen::Vector2d findIntersection(const Eigen::Vector2d &Lane_1_Start,const Eigen::Vector2d &Lane_1_End,const Eigen::Vector2d &Lane_2_Start,const Eigen::Vector2d &Lane_2_End);

void resampleTrajectory(std::vector<float> &X,std::vector<float> &Y,int numPoints);

void interpoint(std::vector<float> &X,int numPoints);

std::vector<Eigen::Vector2d> cubicBezierDoubleTangent(const Eigen::Vector2d &P0,const Eigen::Vector2d &P3,const Eigen::Vector2d &t1,const Eigen::Vector2d &t2);

float calculateCurvatureThreePoint(const std::vector<float> x,const std::vector<float> y);

std::vector<std::vector<float>> curvatureLimit(const std::vector < std::vector<Eigen::Vector2d> > All_curve,float k_curvatureLimit);

int determinePointRegion(const Eigen::Vector2d intersection, const Eigen::Vector2d line1_vec, const Eigen::Vector2d line2_vec, const Eigen::Vector2d point);

float pointToLineDistance(const Eigen::Vector2d point, const Eigen::Vector2d line_point1, const Eigen::Vector2d line_point2);

float line_looktable(const std::vector<float> &X,const std::vector<float> &Y,float inputX);

std::vector<Eigen::Vector2d> mergeXYTraj(std::vector<Eigen::Vector2d> traj_1,std::vector<Eigen::Vector2d> traj_2,std::vector<Eigen::Vector2d> traj_3);

void TjaLinearInterpolation(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &xSampled, std::vector<float> &ySampled);

float TjaCalculateHeading(const std::vector<float> &x, const std::vector<float> &y,const int &n, int point);

void TanLimit(float &k);

float safeDivide(float nominator,float denominator,float threshold);

}
}
