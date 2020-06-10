#include "./bezier_curve.h"

#include <iostream>

int main() {
  //holo::planning::math::Bezier test(2);
  holo::planning::math::Bezier<std::vector<float>, 2> test1(2);
  holo::planning::math::BezierPoint<std::vector<float>, 2> start({421.0, 381.0}, 0.0);
  holo::planning::math::BezierPoint<std::vector<float>, 2> end({428.0, 198.0}, 0.0);
  holo::planning::math::BezierPoint<std::vector<float>, 2> ctrl_pt({403.0, 372.0}, 0.0);
  test1.SetStartPt(start);
  test1.SetEndPt(end);
  test1.SetCtrlPts({ctrl_pt, ctrl_pt});
  test1.SetOrder(3);
  int output = test1.GetOrder();
  std::vector<holo::planning::math::BezierPoint<std::vector<float>, 2>> path_points(30, start);
  std::cout << output << std::endl;
  test1.PtsOnCurve(path_points, 30);
}
