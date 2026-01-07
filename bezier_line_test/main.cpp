#include "common_point.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

const std::vector<Point2Dd>  enter_center_line_points_wgs84 = { { 120.64432186131657, 31.41842618612635 }, { 120.644338361316571, 31.418449186126349 }, { 120.64435536131657, 31.418472186126351 }, { 120.644371861316571, 31.41849518612635 }, { 120.644388861316571, 31.418518186126349 }, { 120.644405361316572, 31.418541186126351 }, { 120.644421861316573, 31.41856418612635 }, { 120.644438361316574, 31.418587186126349 }, { 120.644454861316575, 31.418610186126351 }, { 120.644471861316575, 31.41863318612635 }, { 120.644488361316576, 31.418656186126348 }, { 120.644504861316577, 31.418679186126351 }, { 120.644521861316576, 31.41870218612635 }, { 120.644538361316577, 31.418725186126348 }, { 120.644555361316577, 31.418748186126351 } };
const std::vector<Point2Dd> exit_center_line_points_wgs84=
{ { 120.64484989891001, 31.418813487815729 }, { 120.644889955008381, 31.418792889756308 }, { 120.644944423749266, 31.418764546765967 }, { 120.644998989039479, 31.418735818986622 }, { 120.645105387926151, 31.418680336708793 }, { 120.645217892438964, 31.41862128553311 } };

std::vector<Point2Dd> GenerateInterpolatedLine(
    const Point2Dd& end_point_first, const Point2Dd& start_point_second, double interval) {
    std::vector<Point2Dd> interpolated_line{};
    interpolated_line.clear();

    // 计算两点之间的距离
    double dx = start_point_second.x - end_point_first.x;
    double dy = start_point_second.y - end_point_first.y;
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    if (distance > 1000) {
      // std::cout << __FILE__ << __LINE__ << "distance: " << distance << std::endl;
      return interpolated_line;
    }

    // 根据距离和插值间隔计算插值点的数量
    int num_points = static_cast<int>(distance / interval);

    // 生成插值点
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = end_point_first.x * (1 - t) + start_point_second.x * t;
        double y = end_point_first.y * (1 - t) + start_point_second.y * t;
        interpolated_line.push_back(Point2Dd{x, y});
    }

    return interpolated_line;
}

// 计算控制点
Point2Dd CalculateControlPoint(const Point2Dd& A1, const Point2Dd& A3, const Point2Dd& B1, double k) {
    double A1B1Dist = std::sqrt(std::pow(A1.x - B1.x, 2) + std::pow(A1.y - B1.y, 2));
    double A1A3Dist = std::sqrt(std::pow(A1.x - A3.x, 2) + std::pow(A1.y - A3.y, 2));
    Point2Dd P1;
    P1.x = A1.x + k * (A1.x - A3.x) * (A1B1Dist / A1A3Dist);
    P1.y = A1.y + k * (A1.y - A3.y) * (A1B1Dist / A1A3Dist);
    return P1;
}

// 计算贝塞尔曲线的点
std::vector<Point2Dd> GenerateBezierCurve(const Point2Dd& P0, const Point2Dd& P1,
                                                         const Point2Dd& P2, const Point2Dd& P3,
                                                         const double& segment_length) {
    std::vector<Point2Dd> bezierPoints;
    bezierPoints.push_back(P0);

    double t = 0.0;
    Point2Dd prev_point = P0;

    while (t < 1.0) {
        double t1 = 1 - t;
        double t1_squared = t1 * t1;
        double t_squared = t * t;

        Point2Dd bezierPoint;
        bezierPoint.x = P0.x * t1 * t1_squared +
                        3 * P1.x * t1_squared * t +
                        3 * P2.x * t1 * t_squared +
                        P3.x * t_squared * t;
        bezierPoint.y = P0.y * t1 * t1_squared +
                        3 * P1.y * t1_squared * t +
                        3 * P2.y * t1 * t_squared +
                        P3.y * t_squared * t;

        double distance = std::sqrt(std::pow(bezierPoint.x - prev_point.x, 2) +
                          std::pow(bezierPoint.y - prev_point.y, 2));
        if (distance >= segment_length) {
            bezierPoints.push_back(bezierPoint);
            prev_point = bezierPoint;
        }

        t += 0.001;  // 增加t的步长以确保生成足够多的点
    }

    bezierPoints.push_back(P3);  // 确保最后一个点是P3

    return bezierPoints;
}
bool WGS84ToBody(const Point2Dd& point_wgs84, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_body) {
    static double DEG_TO_RAD = M_PI / 180.0;

    // 转换自车的经纬度为弧度
    double ego_lon = ego_pos_wgs84.x * DEG_TO_RAD;
    double ego_lat = ego_pos_wgs84.y * DEG_TO_RAD;

    // 转换目标点的经纬度为弧度
    double point_lon = point_wgs84.x * DEG_TO_RAD;
    double point_lat = point_wgs84.y * DEG_TO_RAD;

    // 计算必要的地球半径
    double tmp, sqrttmp, rm, rn;
    tmp = sin(ego_lat);
    tmp *= tmp;
    tmp = 1 - 0.00669437999013 * tmp;
    sqrttmp = sqrt(tmp);
    rm = 6378137.0 * (1.0 - 0.00669437999013) / (sqrttmp * tmp);
    rn = 6378137.0 / sqrttmp;

    double dn = (point_lat - ego_lat) * rm;
    double de = (point_lon - ego_lon) * rn * cos(ego_lat);

    point_body.x = dn * cos(ego_pos_wgs84.yaw) + de * sin(ego_pos_wgs84.yaw);
    point_body.y = dn * sin(ego_pos_wgs84.yaw) - de * cos(ego_pos_wgs84.yaw);

    return true;
}
bool LineWGS84ToBody(const std::vector<Point2Dd>& line_wgs84, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_body) {
    for (const auto& point : line_wgs84) {
        Point2Dd point_body;
        WGS84ToBody(point, ego_pos_wgs84, point_body);
        line_body.emplace_back(point_body);
    }
    return true;
}
bool BodyToWGS84(const Point2Dd& point_body, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_wgs84) {
    static double DEG_TO_RAD = M_PI / 180.0;
    static double RAD_TO_DEG = 180.0 / M_PI;

    // 转换自车的经纬度为弧度
    double ego_lon = ego_pos_wgs84.x * DEG_TO_RAD;
    double ego_lat = ego_pos_wgs84.y * DEG_TO_RAD;

    // 计算必要的地球半径
    double rm, rn, tmp, sqrttmp;
    tmp = sin(ego_lat);
    tmp *= tmp;
    tmp = 1 - 0.00669437999013 * tmp;
    sqrttmp = sqrt(tmp);
    rm = 6378137.0 * (1.0 - 0.00669437999013) / (sqrttmp * tmp);
    rn = 6378137.0 / sqrttmp;

    // 反向计算 dn 和 de
    double dn = (point_body.x * cos(ego_pos_wgs84.yaw) + point_body.y * sin(ego_pos_wgs84.yaw));
    double de = (point_body.x * sin(ego_pos_wgs84.yaw) - point_body.y * cos(ego_pos_wgs84.yaw));

    // 计算经纬度
    point_wgs84.y = ego_lat + dn / rm;  // 纬度加上偏移
    point_wgs84.x = ego_lon + de / (rn * cos(ego_lat));  // 经度加上偏移

    // 将结果转换为度
    point_wgs84.y *= RAD_TO_DEG;
    point_wgs84.x *= RAD_TO_DEG;

    // 确保经度在[-180, 180]范围内
    if (point_wgs84.x < -180.0) point_wgs84.x += 360.0;
    if (point_wgs84.x > 180.0) point_wgs84.x -= 360.0;


    return true;
}
bool LineBodyToWGS84(const std::vector<Point2Dd>& line_body, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_wgs84) {
    for (const auto& point : line_body) {
        Point2Dd point_wgs84;
        BodyToWGS84(point, ego_pos_wgs84, point_wgs84);
        line_wgs84.emplace_back(point_wgs84);
    }
    return true;
}
int main(){
    DoublePosePoint enter_ego_pose_point{};
      enter_ego_pose_point.x= 120.6445525;
      enter_ego_pose_point.y = 31.4187499;
      enter_ego_pose_point.yaw = 0.602756;
        std::vector<Point2Dd> exit_center_line_points_body{};
        std::vector<Point2Dd>  enter_center_line_points_body{};
        LineWGS84ToBody(enter_center_line_points_wgs84, enter_ego_pose_point, enter_center_line_points_body);
        LineWGS84ToBody(exit_center_line_points_wgs84, enter_ego_pose_point, exit_center_line_points_body);
        {
        std::stringstream ss, ss1;
        ss<<"enter_center_line_x = [";
        ss1<<"enter_center_line_y = [";
        for(int j=0; j<enter_center_line_points_body.size();j++){
            if(j == enter_center_line_points_body.size()-1){
                ss<< enter_center_line_points_body[j].x;
            }else{
                ss<< enter_center_line_points_body[j].x<<" ,";
            }            
        }
        for(int j=0; j<enter_center_line_points_body.size();j++){
            if(j == enter_center_line_points_body.size()-1){
                ss1<< enter_center_line_points_body[j].y;
            }else{
                ss1<< enter_center_line_points_body[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }        
        {
        std::stringstream ss, ss1;
        ss<<"exit_center_line_x = [";
        ss1<<"exit_center_line_y = [";
        for(int j=0; j<exit_center_line_points_body.size();j++){
            if(j == exit_center_line_points_body.size()-1){
                ss<< exit_center_line_points_body[j].x;
            }else{
                ss<< exit_center_line_points_body[j].x<<" ,";
            }            
        }
        for(int j=0; j<exit_center_line_points_body.size();j++){
            if(j == exit_center_line_points_body.size()-1){
                ss1<< exit_center_line_points_body[j].y;
            }else{
                ss1<< exit_center_line_points_body[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }    
    std::vector<Point2Dd> intersection_center_line_body{};
    intersection_center_line_body = GenerateInterpolatedLine(
        enter_center_line_points_body.back(), exit_center_line_points_body.front(), 2.5);
    Point2Dd enter_point_last = enter_center_line_points_body.back();  // P0 A1
    Point2Dd enter_point_second_last =
    enter_center_line_points_body[enter_center_line_points_body.size() - 3];  //  P1 A3
    Point2Dd exit_point_first = exit_center_line_points_body.front();   //   P3 B1
    Point2Dd exit_point_second = exit_center_line_points_body[2];   //    P2 B3
    // 系数和贝塞尔曲线点数
    double k = 0.5;
    // int n = 50;   //  贝塞尔曲线点的数量
    double two_point_distance = 1.25;

    Point2Dd  P1 = CalculateControlPoint(enter_point_last, enter_point_second_last, exit_point_first, k);
    Point2Dd  P2 = CalculateControlPoint(exit_point_first, exit_point_second, enter_point_last, k);

    std::cout<<"P0_X =["<<enter_point_last.x<<"]"<<std::endl;
    std::cout<<"P0_Y =["<<enter_point_last.y<<"]"<<std::endl;
    std::cout<<"P1_X =["<<P1.x<<"]"<<std::endl;
    std::cout<<"P1_Y =["<<P1.y<<"]"<<std::endl;
    std::cout<<"P2_X =["<<P2.x<<"]"<<std::endl;
    std::cout<<"P2_Y =["<<P2.y<<"]"<<std::endl;
    std::cout<<"P3_X =["<<exit_point_first.x<<"]"<<std::endl;
    std::cout<<"P3_Y =["<<exit_point_first.y<<"]"<<std::endl;
    std::cout<<"enter_point_second_last_x =["<<enter_point_second_last.x<<"]"<<std::endl;
    std::cout<<"enter_point_second_last_y =["<<enter_point_second_last.y<<"]"<<std::endl;
    std::cout<<"exit_point_second_x =["<<exit_point_second.x<<"]"<<std::endl;
    std::cout<<"exit_point_second_y =["<<exit_point_second.y<<"]"<<std::endl;
    // TODO(lxf)   最终下发的时候需要插值成一定的距离
    intersection_center_line_body = GenerateBezierCurve(enter_point_last, P1, P2, exit_point_first, two_point_distance);

        {
        std::stringstream ss, ss1;
        ss<<"cross_center_line_x = [";
        ss1<<"cross_center_line_y = [";
        for(int j=0; j<intersection_center_line_body.size();j++){
            if(j == intersection_center_line_body.size()-1){
                ss<< intersection_center_line_body[j].x;
            }else{
                ss<< intersection_center_line_body[j].x<<" ,";
            }            
        }
        for(int j=0; j<intersection_center_line_body.size();j++){
            if(j == intersection_center_line_body.size()-1){
                ss1<< intersection_center_line_body[j].y;
            }else{
                ss1<< intersection_center_line_body[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        } 

    return 1;
}