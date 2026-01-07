#include "common_math.h"
namespace CommuteMode {
namespace MapFusion {

template <typename T>
std::vector<T> TranslateLine(const std::vector<T>& line,
                             double translate_distance) {
  std::vector<T> translated_line{};
  for (size_t i = 0; i < line.size() - 1; ++i) {
    T p1 = line[i];
    T p2 = line[i + 1];
    T normal = NormalVector(p1, p2);
    translated_line.push_back(TranslatePoint(p1, normal, translate_distance));
  }
  // 处理最后一个点
  if (!line.empty()) {
    T p1 = line[line.size() - 2];
    T p2 = line[line.size() - 1];
    T normal = NormalVector(p1, p2);
    translated_line.push_back(TranslatePoint(p2, normal, translate_distance));
  }
  return translated_line;
}

template <typename T>
std::vector<T> ExtendLine(const std::vector<T>& line, double extend_distance) {
  if (line.size() < 2) {
    return line;  // 如果线的点数少于2，无法延伸
  }

  std::vector<T> extended_line = line;

  // 计算起点的延伸点
  T start_direction = DirectionVector(line[1], line[0]);
  T extended_start = ExtendPoint(line[0], start_direction, extend_distance);
  extended_line.insert(extended_line.begin(), extended_start);

  // 计算终点的延伸点
  T end_direction =
      DirectionVector(line[line.size() - 2], line[line.size() - 1]);
  T extended_end =
      ExtendPoint(line[line.size() - 1], end_direction, extend_distance);
  extended_line.push_back(extended_end);

  return extended_line;
}

bool CalPointSLBodyCoordinate(std::vector<Point2Dd> ref_line_points, Point2Dd tar_point, PointSLd& sl) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    Point2Dd proj_point;
    bool is_inside = false;
    int nearest_index = -1;
    int sign = 1;
    if (GetProjectPointBodyCoordinate(tar_point, ref_line_points, proj_point, is_inside, nearest_index, sign) == false) {
        return false;
    }
    if (nearest_index >= ref_line_points.size() || nearest_index < 0) {
        return false;
    }

    double accumulate_s = 0.0;
    double dist_temp = 0.0;
    for (int i = 1; i <= nearest_index && nearest_index < ref_line_points.size(); i++) {
        dist_temp = sqrt(pow(ref_line_points[i].x - ref_line_points[i - 1].x, 2) +
                         pow(ref_line_points[i].y - ref_line_points[i - 1].y, 2));
        accumulate_s += dist_temp;
    }

    double dist = sqrt(pow(ref_line_points[nearest_index].x - proj_point.x, 2) +
                       pow(ref_line_points[nearest_index].y - proj_point.y, 2));
    if (proj_point.x < ref_line_points[nearest_index].x) {
        accumulate_s = accumulate_s - dist;
    } else {
        accumulate_s = accumulate_s + dist;
    }

    sl.s = accumulate_s;
    sl.l = sqrt(pow(tar_point.x - proj_point.x, 2) + pow(tar_point.y - proj_point.y, 2));
    
    //dir 
    // Point2Dd dir_line_vec(ref_line_points[1].x -ref_line_points[0].x, ref_line_points[1].y -ref_line_points[0].y);
    // Point2Dd dir_point_vec(tar_point.x -ref_line_points[0].x, tar_point.y -ref_line_points[0].y);
    // double sign = 1;
    // double cross = dir_line_vec.x *dir_point_vec.y - dir_line_vec.y * dir_point_vec.x;
    if(sign<0){
        sl.l = -sl.l;
    }
    // if(LeftOfLine(tar_point, ref_line_points[0], ref_line_points[1])){
    //      sl.l = -sl.l;
    // }
    return true;
}

bool LeftOfLine(const Point2Dd& p, const Point2Dd& p1, const Point2Dd& p2){

    double tmpx = (p1.x - p2.x) / (p1.y - p2.y) * (p.y - p2.y) + p2.x;

    if (tmpx > p.x){
        //当tmpx>p.x的时候，说明点在线的左边，小于在右边，等于则在线上。
        return true;
    }
        
    return false;
}

bool GetProjectPointBodyCoordinate(Point2Dd tar_point, std::vector<Point2Dd> ref_line_points,
                                                       Point2Dd& proj_point, bool& is_inside, int& nearest_index, int& sign) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    nearest_index = -1;
    GetNearestPointOnline(tar_point.x, tar_point.y, ref_line_points, nearest_index);
    if (nearest_index >= ref_line_points.size()) {
        nearest_index = std::max(0, static_cast<int>(ref_line_points.size() - 1));
    } else if (nearest_index < 0) {
        nearest_index = 0;
    }
    Point2Dd match_point = ref_line_points[nearest_index];

    // judge project point use nearest_index+1 or nearest -1
    Point2Dd vector_d(tar_point.x - match_point.x, tar_point.y - match_point.y);
    double vector_d_norm = sqrt(pow(vector_d.x, 2) + pow(vector_d.y, 2));
    Point2Dd vector_s(0.0, 0.0);
    double vector_s_norm = 0.0;
    double cos_A = -1.0;

    if ((ref_line_points.size() > nearest_index + 1) &&
        ((nearest_index - 1) >= 0 && (nearest_index - 1) < ref_line_points.size())) {
        vector_s.x = ref_line_points[nearest_index + 1].x - ref_line_points[nearest_index].x;
        vector_s.y = ref_line_points[nearest_index + 1].y - ref_line_points[nearest_index].y;
        vector_s_norm = sqrt(pow(vector_s.x, 2) + pow(vector_s.y, 2));
        cos_A = (vector_s.x * vector_d.x + vector_s.y * vector_d.y) / (vector_s_norm * vector_d_norm);
        if (cos_A < 0) {
            // tar_point must between neareat+1 and nearest or between neareat-1 and nearest
            vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index - 1].x;
            vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index - 1].y;
        }
    } else if (nearest_index == 0 && (nearest_index + 1) < ref_line_points.size()) {
        vector_s.x = ref_line_points[nearest_index + 1].x - ref_line_points[nearest_index].x;
        vector_s.y = ref_line_points[nearest_index + 1].y - ref_line_points[nearest_index].y;
    } else if (nearest_index == (ref_line_points.size() - 1) && (nearest_index - 1) >= 0 &&
               (nearest_index - 1) < ref_line_points.size()) {
        vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index - 1].x;
        vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index - 1].y;
    } else {
        // abnormal
        return false;
    }

    // cal vector_d project to vector_s 's ds
    vector_s_norm = sqrt(pow(vector_s.x, 2) + pow(vector_s.y, 2));
    double project_s = 0;
    if (vector_d_norm < 0.0001 || vector_s_norm <0.0001) {
        project_s = 0;
        proj_point.x = match_point.x;
        proj_point.y = match_point.y;
    } else {
        project_s = (vector_s.x * vector_d.x + vector_s.y * vector_d.y) / vector_s_norm;
        proj_point.x = match_point.x + project_s * vector_s.x / vector_s_norm;
        proj_point.y = match_point.y + project_s * vector_s.y / vector_s_norm;
    }

    // judge is inside line or not
    is_inside = true;
    if (nearest_index == 0 && (nearest_index + 1) < ref_line_points.size()) {
        is_inside =
            IsPointInsideBodyCoordinate(ref_line_points[nearest_index], ref_line_points[nearest_index + 1], proj_point);
    } else if (nearest_index == (ref_line_points.size() - 1) && (nearest_index - 1) >= 0 &&
               (nearest_index - 1) < ref_line_points.size()) {
        is_inside =
            IsPointInsideBodyCoordinate(ref_line_points[nearest_index], ref_line_points[nearest_index - 1], proj_point);
    } else {
        // if nearest not first, and also not last, it must be in middle
    }

    //dir
    Point2Dd dir_point_vec(tar_point.x -proj_point.x, tar_point.y -proj_point.y);
    double cross = vector_s.x *dir_point_vec.y - vector_s.y * dir_point_vec.x;
    if(cross<0){
        sign = -1;
    }else{
        sign =1;
    }    

    return true;
}

bool GetNearestPointOnline(double utm_x, double utm_y, const std::vector<Point2Dd>& ref_line_points,
                                               int& nearest_index) {
    double min_dist = 65535;
    nearest_index = -1;
    Point2Dd ref_point;
    for (int i = 0; i < ref_line_points.size(); i++) {
        ref_point = ref_line_points[i];
        // printf("cartesian_point->x: %f ref_point.x: %f\n", cartesian_point->x, ref_point.x);
        // printf("cartesian_point->y: %f ref_point.y: %f\n", cartesian_point->y, ref_point.y);
        double dist = pow(utm_x - ref_point.x, 2) + pow(utm_y - ref_point.y, 2);
        dist = sqrt(dist);
        // printf("dist: %f i: %d\n", dist, i);
        if (dist <= min_dist) {
            nearest_index = i;
            min_dist = dist;
        }
    }
    return true;
}

bool IsPointInsideBodyCoordinate(Point2Dd point1, Point2Dd point2, Point2Dd tar_point) {
    double x_min = std::min(point1.x, point2.x);
    double x_max = std::max(point1.x, point2.x);
    double y_min = std::min(point1.y, point2.y);
    double y_max = std::max(point1.y, point2.y);

    bool is_inside = false;

    if (tar_point.x >= x_min && tar_point.x <= x_max && tar_point.y >= y_min && tar_point.y <= y_max) {
        // printf("point in the poins \n" );
        is_inside = true;
    }

    return is_inside;
}

bool FixPathDensity(const std::vector<Point2Dd>& raw_reference_line, double sample_distance,
                                    std::vector<Point2Dd>& fixed_reference_line) {
    if (raw_reference_line.size() <= 1 || sample_distance < 0.1) {
        // std::cout << "raw point num to less" << std::endl;
        return false;
    }

    // 路径密度优化
    fixed_reference_line.clear();
    //
    double accumu_s = 0;
    Point2Dd point0 = raw_reference_line[0];
    fixed_reference_line.emplace_back(point0);
    //
    const double accumu_s_threshold = 5000;
    // std::cout << "FixPathDensity, raw_reference_line.size(): " << raw_reference_line.size() << std::endl;
    for (int i = 1; i < raw_reference_line.size() && accumu_s < accumu_s_threshold; i++) {
        const Point2Dd& point1 = raw_reference_line[i];
        double distance = sqrt(pow(point1.x - point0.x, 2) + pow(point1.y - point0.y, 2));
        if (distance <= sample_distance) {
            // std::cout << "   , continue;" << std::endl;
            if(i == raw_reference_line.size()-1){//保存最后一个点
                fixed_reference_line.emplace_back(point1);
            }
            continue;
        }
        while (distance > sample_distance && accumu_s < accumu_s_threshold) {
            double coeff = sample_distance / distance;
            double x = coeff * (point1.x - point0.x) + point0.x;
            double y = coeff * (point1.y - point0.y) + point0.y;
            point0.x = x;
            point0.y = y;
            fixed_reference_line.emplace_back(point0);
            accumu_s += sample_distance;
            distance -= sample_distance;
            // std::cout << " accumu_s: " << accumu_s << " distance: " << distance << std::endl;
        }
    }

    return true;
}

double Haversine(double lat1, double lon1, double lat2, double lon2) {
    // 将纬度和经度从角度转换为弧度
    double lat1_rad = DegToRad(lat1);
    double lon1_rad = DegToRad(lon1);
    double lat2_rad = DegToRad(lat2);
    double lon2_rad = DegToRad(lon2);

    // 计算纬度和经度的差值
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    // 计算 Haversine 公式
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // 计算距离（单位：米）
    double distance = EARTH_RADIUS * c * 1000;
    return distance;
}

void DougLasPeucker(const std::vector<Point2Dd>& points, double epsilon, std::vector<Point2Dd>* simplified_points) {
    if (points.size() < 2) {
        return;
    }
    float max_distance_square = 0.0f;
    size_t index = 0;

    for (size_t i = 1; i < points.size() - 1; ++i) {
        float distance = PointLineDistanceSquare(points[0].x, points[0].y, points[points.size() - 1].x,
                         points[points.size() - 1].y, points[i].x,  points[i].y);
        if (distance > max_distance_square) {
            index = i;
            max_distance_square = distance;
        }
    }

    if (max_distance_square > epsilon * epsilon) {
        std::vector<Point2Dd> first_line(points.begin(), points.begin() + index + 1);
        std::vector<Point2Dd> second_line(points.begin() + index, points.end());
        std::vector<Point2Dd> first_line_simplified{}, second_line_simplified{};
        DougLasPeucker(first_line, epsilon, &first_line_simplified);
        DougLasPeucker(second_line, epsilon, &second_line_simplified);
        simplified_points->assign(first_line_simplified.begin(), first_line_simplified.end() - 1);
        simplified_points->insert(simplified_points->end(), second_line_simplified.begin(),
                                second_line_simplified.end());
    } else {
        simplified_points->clear();
        simplified_points->push_back(points[0]);
        simplified_points->push_back(points[points.size() - 1]);
    }

    return;
}

std::vector<double> ComputeCurvatures(const std::vector<Point2Dd>& xy_points) {
    std::vector<double> curvatures{};

    if (xy_points.size() < 2) {
        return curvatures;
    }

    std::vector<double> x_over_s_first_derivatives;
    std::vector<double> y_over_s_first_derivatives;
    std::vector<double> x_over_s_second_derivatives;
    std::vector<double> y_over_s_second_derivatives;

    std::size_t points_size = xy_points.size();

    // Get finite difference approximated first derivative of y and x respective to s for curvature calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = 0.0;
        double yds = 0.0;
        if (i == 0) {
            xds = (xy_points[i + 1].x - xy_points[i].x);
            yds = (xy_points[i + 1].y - xy_points[i].y);
        } else if (i == points_size - 1) {
            xds = (xy_points[i].x - xy_points[i - 1].x);
            yds = (xy_points[i].y - xy_points[i - 1].y);
        } else {
            xds = (xy_points[i + 1].x - xy_points[i - 1].x);
            yds = (xy_points[i + 1].y - xy_points[i - 1].y);
        }
        x_over_s_first_derivatives.push_back(xds);
        y_over_s_first_derivatives.push_back(yds);
    }

    // Get finite difference approximated second derivative of y and x respective to s for curvature calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xdds = 0.0;
        double ydds = 0.0;
        if (i == 0) {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]);
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]);
        } else if (i == points_size - 1) {
            xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]);
            ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]);
        } else {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]);
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]);
        }
        x_over_s_second_derivatives.push_back(xdds);
        y_over_s_second_derivatives.push_back(ydds);
    }

    // Curvature calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = x_over_s_first_derivatives[i];
        double yds = y_over_s_first_derivatives[i];
        double xdds = x_over_s_second_derivatives[i];
        double ydds = y_over_s_second_derivatives[i];
        double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        curvatures.push_back(kappa);
    }

    return curvatures;
}

bool blh2xyz(const double lon, const double lat, const double alt, double& x, double& y, double& z) {
    const double A_WGS84 = 6378137.0;
    const double F_WGS84 = 1 / 298.257223563;
    const double E2_WGS84 = (2 - F_WGS84) * F_WGS84;
    double lon_r = lon * M_PI / 180.0;
    double lat_r = lat * M_PI / 180.0;
    double sin_lat = sin(lat_r);
    double cos_lat = cos(lat_r);
    double N = A_WGS84 / sqrt(1 - E2_WGS84 * sin_lat * sin_lat);
    x = (alt + N) * cos_lat * cos(lon_r);
    y = (alt + N) * cos_lat * sin(lon_r);
    z = (N * (1 - E2_WGS84) + alt) * sin_lat;
    return true;
}

bool calculateDistance(const double x0, const double y0, const double z0, const double x1,
                                       const double y1, const double z1, double& distance) {
    distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    return true;
}

double calculateDistanceGC02(const double x0, const double y0, const double z0, const double x1,
                                           const double y1, const double z1) {
    double distance = 0.0;
    double temp_x0 = 0.0;
    double temp_y0 = 0.0;
    double temp_z0 = 0.0;
    blh2xyz(x0, y0, z0, temp_x0, temp_y0, temp_z0);

    double temp_x1 = 0.0;
    double temp_y1 = 0.0;
    double temp_z1 = 0.0;
    blh2xyz(x1, y1, z1, temp_x1, temp_y1, temp_z1);

    calculateDistance(temp_x0, temp_y0, temp_z0, temp_x1, temp_y1, temp_z1, distance);

    return distance;
}

}  // namespace MapFusion
}  // namespace CommuteMode
