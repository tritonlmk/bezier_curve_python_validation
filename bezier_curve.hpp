#ifndef _HOLO_PLANNING_MATH_BEZIER_CURVE_HPP_
#define _HOLO_PLANNING_MATH_BEZIER_CURVE_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

#include <glog/logging.h>

namespace holo {
namespace planning {
namespace math {

// ==, !=, [], size()
template<typename Point, uint DIM>
BezierPoint<Point, DIM>::BezierPoint() {
  coordinate.resize(DIM);
}

template<typename Point, uint DIM>
BezierPoint<Point, DIM>::BezierPoint(Point coords) {
  if (coords.size() == DIM) {
    coordinate = coords;
  } else {
    coordinate.resize(DIM);
    LOG(ERROR) << "Wrong in Bezier Curve: invalid coords size";
  }
}

template<typename Point, uint DIM>
BezierPoint<Point, DIM>::BezierPoint(Point coords, float curvature_in) {
  curvature = curvature_in;
  if (coords.size() == DIM) {
    coordinate = coords;
    curvature = curvature_in;
  } else {
    coordinate.resize(DIM);
    curvature = curvature_in;
    LOG(ERROR) << "Wrong in Bezier Curve: invalid coords size";
  }
}

template<typename Point, uint DIM>
bool BezierPoint<Point, DIM>::operator == (BezierPoint const& t) const {
  return ((t.coordinate == this->coordinate) && ((t.curvature - this->curvature) < 0.001));
}

template<typename Point, uint DIM>
bool BezierPoint<Point, DIM>::operator != (BezierPoint const& t) const {
  return (!(t.coordinate == this->coordinate) || ((t.curvature - this->curvature) > 0.001));
}

// Beizer
template<typename Point, uint DIM>
Bezier<Point, DIM>::Bezier(uint curve_order) {
  dimension_ = DIM;
  SetOrder(curve_order);
}

template<typename Point, uint DIM>
Bezier<Point, DIM>::~Bezier() {}

template<typename Point, uint DIM>
void Bezier<Point, DIM>::SetOrder(uint curve_order) {
  if (curve_order > 5) {
    LOG(ERROR) << "Wrong in Bezier Curve: Currently only support up to 5th order Bezier Curve";
    return;
  }
  order_ = curve_order;
  switch (curve_order) {
    case 1:
      bezier_curve_ = [this](float t) -> BezierPoint<Point, DIM> {
        BezierPoint<Point, DIM> temp;
        for (uint i = 0; i < DIM; ++i) {
          temp.coordinate[i] = (1 - t) * this->GetStartPt().coordinate[i] + t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_1st_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = -this->GetStartPt().coordinate[i] + this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_2nd_ = [](float_t) -> std::vector<float> {
        return std::vector<float>(2, 0.0);
      };
      break;
    case 2:
      bezier_curve_ = [this](float t) -> BezierPoint<Point, DIM> {
        BezierPoint<Point, DIM> temp;
        for (uint i = 0; i < DIM; ++i) {
          temp.coordinate[i] = (1 - t) * (1 - t) * this->GetStartPt().coordinate[i] +
                               2 * t * (1 - t) * this->GetCtrlPts()[0].coordinate[i] +
                               t * t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_1st_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = 2 * (t - 1) * this->GetStartPt().coordinate[i] +
                    (2 - 4 * t) * this->GetCtrlPts()[0].coordinate[i] +
                    2 * t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_2nd_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = 2 * this->GetStartPt().coordinate[i] - 4 * this->GetCtrlPts()[0].coordinate[i] - 2 * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      break;
    case 3:
      bezier_curve_ = [this](float t) -> BezierPoint<Point, DIM> {
        BezierPoint<Point, DIM> temp;
        for (uint i = 0; i < DIM; ++i) {
          temp.coordinate[i] = std::pow((1 - t), 3) * this->GetStartPt().coordinate[i] +
                               3 * t * (1 - t) * (1 - t) * this->GetCtrlPts()[0].coordinate[i] +
                               3 * t * t * (1 - t) * this->GetCtrlPts()[1].coordinate[i] +
                               t * t * t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_1st_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = -3 * (1 - t) * (1 - t) * this->GetStartPt().coordinate[i] + 
                    (9 * t * t - 12 * t + 3) * this->GetCtrlPts()[0].coordinate[i] +
                    (6 * t - 9 * t * t) * this->GetCtrlPts()[1].coordinate[i] +
                    6 * t * t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_2nd_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = -6 * (1 - t) * this->GetStartPt().coordinate[i] +
                    (18 * t - 12) * this->GetCtrlPts()[0].coordinate[i] +
                    (6 - 18 * t) * this->GetCtrlPts()[1].coordinate[i] +
                    6 * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      break;
    case 4:
      bezier_curve_ = [this](float t) -> BezierPoint<Point, DIM> {
        BezierPoint<Point, DIM> temp;
        for (uint i = 0; i < DIM; ++i) {
          temp.coordinate[i] = std::pow((1 - t), 4) * this->GetStartPt().coordinate[i] +
                               4 * std::pow((1 - t), 3) * t * this->GetCtrlPts()[0].coordinate[i] +
                               6 * (1 - t) * (1 - t) * t * t * this->GetCtrlPts()[1].coordinate[i] + 
                               4 * (1 - t) * std::pow(t, 3) * this->GetCtrlPts()[2].coordinate[i] +
                               std::pow(t, 4) * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_1st_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = 4 * std::pow((1 - t), 3) * this->GetStartPt().coordinate[i] +
                    (-8 * std::pow(t, 3) + 12 * t * t - 4) * this->GetCtrlPts()[0].coordinate[i] +
                    (24 * std::pow(t, 3) - 36 * t * t + 12 * t) * this->GetCtrlPts()[1].coordinate[i] +
                    (- 16 * std::pow(t, 3) + 12 * t * t) * this->GetCtrlPts()[2].coordinate[i] +
                    4 * std::pow(t, 3) * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_2nd_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = 12 * (1 - t) * (1 - t) * this->GetStartPt().coordinate[i] +
                    (-24 * t * t + 24 * t) * this->GetCtrlPts()[0].coordinate[i] +
                    (72 * t * t - 72 * t + 12) * this->GetCtrlPts()[1].coordinate[i] +
                    (-48 * t * t + 24 * t) * this->GetCtrlPts()[2].coordinate[i] +
                    12 * t * t * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      break;
    case 5:
      bezier_curve_ = [this](float t) -> BezierPoint<Point, DIM> {
        BezierPoint<Point, DIM> temp;
        for (uint i = 0; i < DIM; ++i) {
        temp.coordinate[i] = std::pow((1 - t), 5) * this->GetStartPt().coordinate[i] +
                             5 * std::pow((1 - t), 4) * t * this->GetCtrlPts()[0].coordinate[i] +
                             10 * std::pow((1 - t), 3) * t * t * this->GetCtrlPts()[1].coordinate[i] + 
                             10 * (1 - t) * (1 - t) * std::pow(t, 3) * this->GetCtrlPts()[2].coordinate[i] +
                             5 * (1 - t) * std::pow(t, 4) * this->GetCtrlPts()[3].coordinate[i] +
                             std::pow(t, 5) * this->GetEndPt().coordinate[i];
		    }
		    return temp;
      };
      deriv_1st_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = -5 * std::pow((1 - t), 4) * this->GetStartPt().coordinate[i] +
                    (20 * t * std::pow((t - 1), 3) + 5 * std::pow((1 - t), 4)) * this->GetCtrlPts()[0].coordinate[i] +
                    (20 * t * std::pow((1 - t), 3) - 30 * std::pow((1 - t), 2) * t * t) * this->GetCtrlPts()[1].coordinate[i] +
                    (30 * t * t * std::pow((t - 1), 2) + 20 * std::pow(t, 3) * (t - 1)) * this->GetCtrlPts()[2].coordinate[i] +
                    (-25 * std::pow(t, 4) + 20 * std::pow(t, 3)) * this->GetCtrlPts()[3].coordinate[i] +
                    5 * std::pow(t, 4) * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      deriv_2nd_ = [this](float t) -> std::vector<float> {
        std::vector<float> temp(DIM, 0.0);
        for (uint i = 0; i < DIM; ++i) {
          temp[i] = 20 * std::pow((1 - t), 3) * this->GetStartPt().coordinate[i] +
                    (40 * std::pow((t - 1), 3) + 60 * t * (t - 1) * (t - 1)) * this->GetCtrlPts()[0].coordinate[i] +
                    (20 * std::pow((1 - t), 3) - 80 * t * (t - 1) * (t - 1) - 60 * t * t * (t - 1)) * this->GetCtrlPts()[1].coordinate[i] +
                    (60 * t * (t - 1) * (t - 1) + 120 * t * t * (t - 1) + 20 * std::pow(t, 3)) * this->GetCtrlPts()[2].coordinate[i] +
                    (-100 * std::pow(t, 3) + 60 * t * t) * this->GetCtrlPts()[3].coordinate[i] +
                    20 * std::pow(t, 3) * this->GetEndPt().coordinate[i];
        }
        return temp;
      };
      break;
  }
}

template<typename Point, uint DIM>
uint Bezier<Point, DIM>::GetOrder() const {return order_;}

template<typename Point, uint DIM>
void Bezier<Point, DIM>::SetEndPt(BezierPoint<Point, DIM> end_point) {end_ = end_point;}

template<typename Point, uint DIM>
BezierPoint<Point, DIM> Bezier<Point, DIM>::GetEndPt() const {return end_;}

template<typename Point, uint DIM>
void Bezier<Point, DIM>::SetStartPt(BezierPoint<Point, DIM> start_point) {start_ = start_point;}

template<typename Point, uint DIM>
BezierPoint<Point, DIM> Bezier<Point, DIM>::GetStartPt() const {return start_;}

template<typename Point, uint DIM>
bool Bezier<Point, DIM>::SetCtrlPts(std::vector<BezierPoint<Point, DIM>> ctrl_points) {
  if (order_ == 0) {
    LOG(ERROR) << "Wrong in Beizer Curve: number of ctrl_pts != curve order.";
    return false;
  }
  if (order_ != ctrl_points.size() + 1) {
    LOG(ERROR) << "Wrong in Beizer Curve: wrong input size of ctrl_points";
    return false;
  }
  ctrl_pts_ = ctrl_points;
  return true;
}

template<typename Point, uint DIM>
std::vector<BezierPoint<Point, DIM>> Bezier<Point, DIM>::GetCtrlPts() const {return ctrl_pts_;}

template<typename Point, uint DIM>
std::function<BezierPoint<Point, DIM> (float t)> Bezier<Point, DIM>::GetBezierFunction() const {return bezier_curve_;}

template<typename Point, uint DIM>
void Bezier<Point, DIM>::PtsOnCurve(std::vector<BezierPoint<Point, DIM>>& curve_points, uint quantity) {
  if (curve_points.size() != quantity) {
    LOG(ERROR) << "Wrong in Bezier Curve: container size != quantity.";
    return;
  }
  if (order_ <= 0 || (ctrl_pts_.size() + 1) != order_) {
    LOG(ERROR) << "Wrong in Bezier Curve: not enough setup";
    return;
  }
  for (uint i = 0; i < quantity; ++i) {
    float t_temp = 1.0 * i / (quantity - 1);
    curve_points[i] = bezier_curve_(t_temp);
    std::vector<float> deriv_1 = deriv_1st_(t_temp);
    std::vector<float> deriv_2 = deriv_2nd_(t_temp);
    float norm_square = 0.0;
    float cross_product = 0.0;
    if (dimension_ == 2) {
      cross_product = deriv_1[0] * deriv_2[1] - deriv_1[1] * deriv_2[0];
      norm_square = std::pow(deriv_1[0], 2) + std::pow(deriv_1[1], 2);
    } else if (dimension_ == 3) {
      cross_product = std::pow((deriv_1[1] * deriv_2[2] - deriv_1[2] * deriv_2[1]), 2) +
                      std::pow((deriv_1[2] * deriv_2[0] - deriv_1[0] * deriv_2[2]), 2) +
                      std::pow((deriv_1[0] * deriv_2[1] - deriv_1[1] * deriv_2[0]), 2);
      norm_square = std::pow(deriv_1[0], 2) + std::pow(deriv_1[1], 2) + std::pow(deriv_1[2], 2);
    } else {
      LOG(ERROR) << "Wrong in Bezier Curve: Wrong dimension.";
    }
    curve_points[i].curvature = cross_product / std::pow(norm_square, 1.5);
  }
}

}// namespace math
}// namespace planning
}// namespace holo

#endif
