#ifndef _MATH_BEZIER_CURVE_H__
#define _MATH_BEZIER_CURVE_H__

#include <vector>
#include <functional>

namespace math {

template<typename Point, uint DIM>
struct BezierPoint {
  BezierPoint();
  BezierPoint(Point coords);
  BezierPoint(Point coords, float curvature_in);
  float curvature;
  Point coordinate;
  bool operator == (BezierPoint const& t) const;
  bool operator != (BezierPoint const& t) const;
};

// Beizer in vehicle coordinate, up to 5 order
template<typename Point, uint DIM>
class Bezier {
 public:
  Bezier();
  Bezier(uint curve_order);
  ~Bezier();
  void SetOrder(uint curve_order);
  uint GetOrder() const;
  void SetEndPt(BezierPoint<Point, DIM> end_point);
  BezierPoint<Point, DIM> GetEndPt() const;
  void SetStartPt(BezierPoint<Point, DIM> start_point);
  BezierPoint<Point, DIM> GetStartPt() const;
  bool SetCtrlPts(std::vector<BezierPoint<Point, DIM>> ctrl_points);
  std::vector<BezierPoint<Point, DIM>> GetCtrlPts() const;

  std::function<BezierPoint<Point, DIM>(float t)> GetBezierFunction() const;
  void PtsOnCurve(std::vector<BezierPoint<Point, DIM>>& curve_points, uint quantity);
 private:
  uint order_;
  float length_;
  uint dimension_;
  BezierPoint<Point, DIM> start_;
  BezierPoint<Point, DIM> end_;
  std::vector<BezierPoint<Point, DIM>> ctrl_pts_;
  // curvature is not calcualted in this function
  std::function<BezierPoint<Point, DIM>(float t)> bezier_curve_;
  std::function<std::vector<float>(float t)> deriv_1st_;
  std::function<std::vector<float>(float t)> deriv_2nd_;
};

}// namespace math

#endif
