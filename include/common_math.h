#pragma once
#include <vector>
#include <cmath>
#include <utility>
#include <limits>

#include "common_point.h"
#include "compile_config.h"

namespace CommuteMode {
namespace MapFusion {

#define M_2PI 6.28318530717958647692
#define M_PI 3.14159265358979323846 /* pi */
#define EARTH_RADIUS  6371.0

template <typename T>
T unify_theta(T theta) {
  while (theta > static_cast<T>(M_PI)) theta -= static_cast<T>(M_2PI);
  while (theta < static_cast<T>(-M_PI)) theta += static_cast<T>(M_2PI);
  return theta;
}

template <typename T>
T aver_theta(T theta1, T theta2, float weight = 0.5) {
  T min, max;
  if (theta1 > theta2) {
    max = theta1;
    min = theta2;
    weight = 1 - weight;
  } else {
    max = theta2;
    min = theta1;
  }
  while (fabs(max - min) > M_PI) {
    max -= static_cast<T>(M_2PI);
  }
  return unify_theta(min * (1 - weight) + max * weight);
}

template <typename T>
T Distance(const T& x1, const T& y1, const T& x2, const T& y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

template <typename T>
T DegToRad(const T& deg) {
  return deg * M_PI / 180.0;
}

template <typename T>
T CrossProduct(const T x0, const T y0, const T x1, const T y1) {
  return x0 * y1 - x1 * y0;
}

template <typename T>
T InnerProduct(const T x0, const T y0, const T x1, const T y1) {
  return x0 * x1 + y0 * y1;
}

template <typename T>
T DistanceSquare(const T& x1, const T& y1, const T& x2, const T& y2) {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

template <typename T>
T FindClosestPoint(const T& point, const std::vector<T>& lane) {
    double min_distance = std::numeric_limits<double>::max();
    T closest_point;
    for (const auto& p : lane) {
        double distance = DistanceSquare(point.x, point.y, p.x, p.y);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = p;
        }
    }
    return closest_point;
}

template <typename T>
T PointLineDistanceSquare(const T s_x, const T s_y, const T e_x, const T e_y,
                          const T x, const T y) {
  T line_length = Distance(s_x, s_y, e_x, e_y);
  if (line_length < 1e-6f) {
    return DistanceSquare(s_x, s_y, x, y);
  }
  T proj = InnerProduct(x - s_x, y - s_y, e_x - s_x, e_y - s_y) / line_length;
  if (proj <= 0.0f) {
    return DistanceSquare(s_x, s_y, x, y);
  }
  if (proj >= line_length) {
    return DistanceSquare(e_x, e_y, x, y);
  }
  T distance =
      CrossProduct(x - s_x, y - s_y, e_x - s_x, e_y - s_y) / line_length;
  return distance * distance;
}

template <typename T>
T LinearInterpolationNumber(const T& n1, const T& n2, const float& weight) {
  return (1.0f - weight) * n1 + weight * n2;
}

template <typename T>
T LinearInterpolationTheta(T t1, T t2, float weight) {
  if (t1 > t2) {
    std::swap(t1, t2);
    weight = 1 - weight;
  }
  while (std::abs(t2 - t1) > static_cast<T>(M_PI)) {
    t2 -= static_cast<T>(M_2PI);
  }
  return unify_theta((1.0f - weight) * t1 + weight * t2);
}

template <typename T>
T NormalVector(const T& point1, const T& point2) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double length = std::sqrt(dx * dx + dy * dy);
    return { -dy / length, dx / length };
}

template <typename T>
T DirectionVector(const T& point1, const T& point2) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double length = std::sqrt(dx * dx + dy * dy);
    return { dx / length, dy / length };
}

template <typename T>
T TranslatePoint(const T& p, const T& normal, double translate_distance) {
    return { p.x + normal.x * translate_distance, p.y + normal.y * translate_distance };
}

template <typename T>
T ExtendPoint(const T& p, const T& direction, double extend_distance) {
    return { p.x + direction.x * extend_distance, p.y + direction.y * extend_distance };
}

// 判断点q是否在点p1和p2之间
template <typename T>
bool onSegment(T p1, T q, T p2) {
    return q.x <= std::max(p1.x, p2.x) && q.x >= std::min(p1.x, p2.x) &&
           q.y <= std::max(p1.y, p2.y) && q.y >= std::min(p1.y, p2.y);
}

// 计算方向
template <typename T>
int orientation(T p, T q, T r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // 共线
    return (val > 0) ? 1 : 2;   // 顺时针或逆时针
}

// 判断两条线段是否相交
template <typename T>
bool IsLineCross(T p1, T q1, T p2, T q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // 一般情况
    if (o1 != o2 && o3 != o4) return true;

    // 特殊情况：共线
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}

template <typename T>
std::vector<T> TranslateLine(const std::vector<T>& line, double translate_distance);

template <typename T>
std::vector<T> ExtendLine(const std::vector<T>& line, double extend_distance);

template<typename T>
PosePoint<T> LinearInterpolationPoint(const PosePoint<T> &p1,
                                      const PosePoint<T> &p2,
                                      const float weight) {
  PosePoint<T> result;
  result.x = LinearInterpolationNumber(p1.x, p2.x, weight);
  result.y = LinearInterpolationNumber(p1.y, p2.y, weight);
  result.z = LinearInterpolationNumber(p1.z, p2.z, weight);
  result.yaw = LinearInterpolationTheta(p1.yaw, p2.yaw, weight);
  result.pitch = LinearInterpolationTheta(p1.pitch, p2.pitch, weight);
  result.roll = LinearInterpolationTheta(p1.roll, p2.roll, weight);
  return result;
}



template<typename T>
LinePoint<T> LinearInterpolationPoint(const LinePoint<T> &p1,
                                      const LinePoint<T> &p2,
                                      const float weight) {
  LinePoint<T> result;
  result.x = LinearInterpolationNumber(p1.x, p2.x, weight);
  result.y = LinearInterpolationNumber(p1.y, p2.y, weight);
  result.z = LinearInterpolationNumber(p1.z, p2.z, weight);
  result.yaw = LinearInterpolationTheta(p1.yaw, p2.yaw, weight);
  result.pitch = LinearInterpolationTheta(p1.pitch, p2.pitch, weight);
  result.roll = LinearInterpolationTheta(p1.roll, p2.roll, weight);
  result.s = LinearInterpolationNumber(p1.s, p2.s, weight);
  result.l = LinearInterpolationNumber(p1.l, p2.l, weight);
  result.dl_ds = LinearInterpolationNumber(p1.dl_ds, p2.dl_ds, weight);
  result.ddl_ds = LinearInterpolationNumber(p1.ddl_ds, p2.ddl_ds, weight);
  result.k = LinearInterpolationNumber(p1.k, p2.k, weight);
  result.dk_ds = LinearInterpolationNumber(p1.dk_ds, p2.dk_ds, weight);
  return result;
}

template <typename T>
bool ComputeCrossPoint(const T& point1,const T& point2, const T& point3, const T& point4, T& cross_point){
  //point1, point2 为一条直线, 不需要延长
  //point3, point4 为一条直线，不需要延长
  double den = (point1.x -point2.x)*(point3.y - point4.y) - (point1.y - point2.y)*(point3.x -point4.x);
  double num_x = (point1.x*point2.y - point1.y*point2.x)*(point3.x - point4.x) -(point1.x - point2.x)*(point3.x*point4.y - point3.y*point4.x);
  double num_y = (point1.x*point2.y - point1.y*point2.x)*(point3.y - point4.y) -(point1.y - point2.y)*(point3.x*point4.y - point3.y*point4.x);

  if(std::abs(den)<0.0001){
    //重合或平行
      return false;  
  }
  cross_point.x = num_x/den;
  cross_point.y = num_y/den;
  return true;
}
    
template <typename T>
bool ComputePathHeading(const std::vector<T>& xy_points,std::vector<double>& headings) {
  headings.clear();

  if (xy_points.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].x - xy_points[i].x);
      y_delta = (xy_points[i + 1].y - xy_points[i].y);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].x - xy_points[i - 1].x);
      y_delta = (xy_points[i].y - xy_points[i - 1].y);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].x - xy_points[i - 1].x);
      y_delta = 0.5 * (xy_points[i + 1].y - xy_points[i - 1].y);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }
  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings.push_back(std::atan2(dys[i], dxs[i]));
  }
  return true;
}
bool LeftOfLine(const Point2Dd& p, const Point2Dd& p1, const Point2Dd& p2);
bool CalPointSLBodyCoordinate(std::vector<Point2Dd> ref_line_points, Point2Dd tar_point, PointSLd& sl);
bool GetProjectPointBodyCoordinate(Point2Dd tar_point, std::vector<Point2Dd> ref_line_points,
                                                       Point2Dd& proj_point, bool& is_inside, int& nearest_index,int& sign);
bool GetNearestPointOnline(double utm_x, double utm_y, const std::vector<Point2Dd>& ref_line_points,int& nearest_index);
bool IsPointInsideBodyCoordinate(Point2Dd point1, Point2Dd point2, Point2Dd tar_point);
bool FixPathDensity(const std::vector<Point2Dd>& raw_reference_line, double sample_distance,
                                    std::vector<Point2Dd>& fixed_reference_line);
// 利用半正矢公式（Haversine公式）根据经纬度计算两点之间的距离
double Haversine(double lat1, double lon1, double lat2, double lon2);

void DougLasPeucker(const std::vector<Point2Dd>& points, double epsilon, std::vector<Point2Dd>* simplified_points);
std::vector<double> ComputeCurvatures(const std::vector<Point2Dd>& xy_points);

bool blh2xyz(const double lon, const double lat, const double alt, double& x, double& y, double& z);
bool calculateDistance(const double x0, const double y0, const double z0, const double x1,
                                       const double y1, const double z1, double& distance);
double calculateDistanceGC02(const double x0, const double y0, const double z0, const double x1,
                                           const double y1, const double z1);
}
}
