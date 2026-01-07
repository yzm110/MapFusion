#pragma once

#include "message.h"
#include "topic/topic_trait.h"
// #include "common_math.h"

namespace CommuteMode {
namespace MapFusion {
using namespace earth::shell::framework;

template <class T>
    struct Point2D {
    T x, y;

    static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value
                        || std::is_same<T, double>::value,
                    "Type not iis nt or float or double!");
    Point2D() : Point2D(0, 0) {}
    Point2D(const T& x, const T& y) : x(x), y(y) {}
    Point2D(const Point2D<T>& point) : Point2D(point.x, point.y) {}
    ~Point2D() = default;
    void Reset(const Point2D<T>& point) {
        x = point.x;
        y = point.y;
    }
    Point2D<T>& operator=(const Point2D<T>& point) {
        Reset(point);
        return *this;
    }
    Point2D<T>& operator+=(const Point2D<T>& point) {
        this->x += point.x;
        this->y += point.y;
        return *this;
    }
    Point2D<T>& operator-=(const Point2D<T>& point) {
        this->x -= point.x;
        this->y -= point.y;
        return *this;
    }
    const Point2D<T> operator-() const {
        return Point2D<T>(-this->x, -this->y);
    }
    const Point2D<T> operator+(const Point2D<T>& point) const {
        return Point2D<T>(this->x + point.x, this->y + point.y);
    }
    const Point2D<T> operator-(const Point2D<T>& point) const {
        return Point2D<T>(this->x - point.x, this->y - point.y);
    }
    const Point2D<T> operator/(const Point2D<T>& point) const {
        return Point2D<T>(this->x / point.x, this->y / point.y);
    }
    const Point2D<T> operator/(const T& value) const {
        return Point2D<T>(this->x / value, this->y / value);
    }
    const Point2D<T> operator*(const Point2D<T>& point) const {
        return Point2D<T>(this->x * point.x, this->y * point.y);
    }
    const Point2D<T> operator*(const T& value) const {
        return Point2D<T>(this->x * value, this->y * value);
    }
    friend const Point2D<T> operator*(const T& value, const Point2D<T>& point) {
        return Point2D<T>(value * point.x, value * point.y);
    }
    // bool IsIn(const Rect<T> &rect) const {
    //     return x >= rect.l && x <= rect.r && y >= rect.t && y <= rect.b;
    // }
};
typedef Point2D<int> Point2Di;
typedef Point2D<float> Point2Df;
typedef Point2D<double> Point2Dd;

template <class T>
struct Point2DEx : public Point2D<T> {
 public:
  Point2DEx() {
    this->x = 0;
    this->y = 0;
    this->source_ = 0;
  }
  Point2DEx(T x, T y) {
    this->x = x; this->y = y;
    this->source_ = 0;
  }

  Point2DEx<T>& operator = (const Point2DEx<T> &pt) {
    this->x = pt.x;
    this->y = pt.y;
    this->source_ = pt.source_;
    return *this;
  }
  // bool IsIn(const Rect<T> &rect) const {
  //   return this->x >= rect.l && this->x <= rect.r
  //     && this->y >= rect.t && this->y <= rect.b;
  // }
  int source_;
};

template <typename T>
struct PointSL {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value|| std::is_same<T, double>::value,
      "Type is not int or float!");
  PointSL() : PointSL(0, 0) {}
  PointSL(const T& s, const T& l) : s(s), l(l) {}
  PointSL(const PointSL<T>& point) : PointSL(point.s, point.l) {}
  ~PointSL() = default;
  void Reset(const PointSL<T>& point) {
    s = point.s;
    l = point.l;
  }
  PointSL<T>& operator=(const PointSL<T>& point) {
    Reset(point);
    return *this;
  }
  T s, l;
};  
typedef PointSL<int> PointSLi;
typedef PointSL<float> PointSLf;
typedef PointSL<double> PointSLd;

template <typename T>
struct Point3D {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  Point3D() : Point3D(0, 0, 0) {}
  Point3D(const T& x, const T& y, const T& z) : x(x), y(y), z(z) {}
  Point3D(const Point3D<T>& point) {Reset(point);}
  ~Point3D() = default;
  float norm2d() const {return sqrt(x*x+y*y);}
  void Reset(const Point3D<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
  }
  Point3D<T>& operator=(const Point3D<T>& point) {
    Reset(point);
    return *this;
  }
  T x, y, z;
}; 

template <typename T>
struct EulerAngle {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  EulerAngle() : EulerAngle(0, 0, 0) {}
  EulerAngle(const T& yaw, const T& pitch, const T& roll) : yaw(yaw),
      pitch(pitch), roll(roll) {}
  EulerAngle(const T& w, const T& x, const T& y, const T& z) : EulerAngle(
      atan2f(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)),
      asinf(2.0 * (w*y - z*x)),
      atan2f(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))) {}
  EulerAngle(const EulerAngle<T>& euler_angle) : EulerAngle(euler_angle.yaw,
                                                            euler_angle.pitch,
                                                            euler_angle.roll) {}
  ~EulerAngle() = default;
  void Reset(const EulerAngle<T>& euler_angle) {
    yaw = euler_angle.yaw;
    pitch = euler_angle.pitch;
    roll = euler_angle.roll;
  }
  EulerAngle<T>& operator=(const EulerAngle<T>& euler_angle) {
    Reset(euler_angle);
    return *this;
  }
  T yaw, pitch, roll;
};  
typedef EulerAngle<int> EulerAnglei;
typedef EulerAngle<float> EulerAnglef;
typedef EulerAngle<double> DoubleEulerAngle;

template<typename T>
EulerAngle<T> LinearInterpolationEulerAngle(const EulerAngle<T> &p1,
                                            const EulerAngle<T> &p2,
                                            const float weight) {
  EulerAngle<T> result;
  result.yaw = LinearInterpolationTheta(p1.yaw, p2.yaw, weight);
  result.pitch = LinearInterpolationTheta(p1.pitch, p2.pitch, weight);
  result.roll = LinearInterpolationTheta(p1.roll, p2.roll, weight);
  return result;
}

template <typename T>
struct PosePoint {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  PosePoint() : PosePoint(0, 0, 0, 0, 0, 0) {}
  PosePoint(const T& x, const T& y, const T& z, const T& yaw, const T& pitch,
            const T& roll) : x(x), y(y), z(z),
                yaw(yaw), pitch(pitch), roll(roll) {}
  PosePoint(const Point3D<T>& point, const EulerAngle<T>& euler_angle)
  : PosePoint(point.x, point.y, point.z, euler_angle.yaw, euler_angle.pitch,
              euler_angle.roll) {}
  ~PosePoint() = default;
  float norm2d() const {return sqrt(x*x+y*y);}
  PosePoint<T>& operator=(const PosePoint<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  PosePoint<T>& operator=(const Point3D<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    return *this;
  }
  PosePoint<T>& operator=(const EulerAngle<T>& point) {
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  PosePoint<T>& operator+=(const PosePoint<T>& point) {
    x += point.x;
    y += point.y;
    z += point.z;
    yaw = UnifyTheta(yaw+point.yaw);
    pitch = UnifyTheta(pitch+point.pitch);
    roll = UnifyTheta(roll+point.roll);
    return *this;
  }
  PosePoint<T>& operator-=(const PosePoint<T>& point) {
    x -= point.x;
    y -= point.y;
    z -= point.z;
    yaw = UnifyTheta(yaw-point.yaw);
    pitch = UnifyTheta(pitch-point.pitch);
    roll = UnifyTheta(roll-point.roll);
    return *this;
  }
  T x, y, z, yaw, pitch, roll;
};  
typedef PosePoint<int> PosePointi;
typedef PosePoint<float> PosePointf;
typedef PosePoint<double> DoublePosePoint;

template <typename T>
struct LinePoint {
  LinePoint():x(0), y(0), z(0), yaw(0), pitch(0), roll(0), s(0), k(0),
      dk_ds(0), l(0), dl_ds(0), ddl_ds(0), c0(0) {}
  LinePoint(const T x, const T y, const T z):x(x), y(y), z(z), yaw(0),
      pitch(0), roll(0), s(0), k(0), dk_ds(0), l(0), dl_ds(0),
      ddl_ds(0), c0(0) {}
  // 显式拷贝构造函数
  LinePoint(const LinePoint<T>& point) : x(point.x), y(point.y), z(point.z),yaw(point.yaw), 
      pitch(point.pitch), roll(point.roll),s(point.s), k(point.k), dk_ds(point.dk_ds),
      l(point.l), dl_ds(point.dl_ds), ddl_ds(point.ddl_ds),c0(point.c0) {}
  ~LinePoint() = default;
  LinePoint<T>& operator=(const LinePoint<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    s = point.s;
    k = point.k;
    dk_ds = point.dk_ds;
    l = point.l;
    dl_ds = point.dl_ds;
    ddl_ds = point.ddl_ds;
    c0 = point.c0;
    return *this;
  }
  LinePoint<T>& operator=(const PosePoint<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  LinePoint<T>& operator=(const Point3D<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    return *this;
  }
  LinePoint<T>& operator=(const EulerAngle<T>& point) {
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  LinePoint<T>& operator=(const PointSL<T>& point) {
    s = point.s;
    l = point.l;
    return *this;
  }
  T x, y, z, yaw, pitch, roll;
  T s, k, dk_ds, l, dl_ds, ddl_ds;

  
  T c0;
}; 


}
}
