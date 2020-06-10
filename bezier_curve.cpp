#include </home/holo/Desktop/cpp_py/bezier_curve.h>
#include </home/holo/Desktop/cpp_py/bezier_curve.hpp>

namespace math {

template class BezierPoint<std::vector<float>, 2>;
template class BezierPoint<std::vector<float>, 3>;
template class Bezier<std::vector<float>, 2>;
template class Bezier<std::vector<float>, 3>;

}  // namespace math


#include <boost/python.hpp>
#include <boost/python/def.hpp>
#include <boost/python/module.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// conversion between python list and stl vector
template<typename T>
inline std::vector<T> to_std_vector(const boost::python::object& iterable) {
  return std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                        boost::python::stl_input_iterator<T>());
}

template<typename T>
inline boost::python::list std_vector_to_py_list(std::vector<T> vector) {
  typename std::vector<T>::iterator iter;
  boost::python::list list;
  for (iter = vector.begin(); iter!= vector.end(); ++iter) {
    list.append(*iter);
  }
  return list;
}

BOOST_PYTHON_MODULE(libBezierTest) {
  using namespace boost::python;

  class_<math::Bezier<std::vector<float>, 2>>("BezierCurve", init<uint>())
    .def("SetOrder", &math::Bezier<std::vector<float>, 2>::SetOrder)
    .def("SetCtrlPts", &math::Bezier<std::vector<float>, 2>::SetCtrlPts)
    .def("PtsOnCurve", &math::Bezier<std::vector<float>, 2>::PtsOnCurve)
    .add_property("order_", &math::Bezier<std::vector<float>, 2>::GetOrder,
                  &math::Bezier<std::vector<float>, 2>::SetOrder)
    .add_property("start_", &math::Bezier<std::vector<float>, 2>::GetStartPt,
                  &math::Bezier<std::vector<float>, 2>::SetStartPt)
    .add_property("end_", &math::Bezier<std::vector<float>, 2>::GetEndPt,
                  &math::Bezier<std::vector<float>, 2>::SetEndPt)
    .add_property("ctrl_pts_", &math::Bezier<std::vector<float>, 2>::GetCtrlPts,
                  &math::Bezier<std::vector<float>, 2>::SetCtrlPts);

  // 3 dim curve
  class_<math::Bezier<std::vector<float>, 3>>("BezierCurve3d", init<uint>())
    .def("SetOrder3d", &math::Bezier<std::vector<float>, 3>::SetOrder)
    .def("SetCtrlPts3d", &math::Bezier<std::vector<float>, 3>::SetCtrlPts)
    .def("PtsOnCurve3d", &math::Bezier<std::vector<float>, 3>::PtsOnCurve)
    .add_property("order_3d", &math::Bezier<std::vector<float>, 3>::GetOrder,
                  &math::Bezier<std::vector<float>, 3>::SetOrder)
    .add_property("start_3d", &math::Bezier<std::vector<float>, 3>::GetStartPt,
                  &math::Bezier<std::vector<float>, 3>::SetStartPt)
    .add_property("end_3d", &math::Bezier<std::vector<float>, 3>::GetEndPt,
                  &math::Bezier<std::vector<float>, 3>::SetEndPt)
    .add_property("ctrl_pts_3d", &math::Bezier<std::vector<float>, 3>::GetCtrlPts,
                  &math::Bezier<std::vector<float>, 3>::SetCtrlPts);


  class_<math::BezierPoint<std::vector<float>, 2>>("BezierPoint", init<std::vector<float>>()) // init<Point> or init<std::vector<float>>
    .def_readwrite("coordinate", &math::BezierPoint<std::vector<float>, 2>::coordinate)
    .def_readonly("curvature", &math::BezierPoint<std::vector<float>, 2>::curvature);
  // 3 dim world point
  class_<math::BezierPoint<std::vector<float>, 3>>("BezierPoint3d", init<std::vector<float>>())
    .def_readwrite("coordinate3d", &math::BezierPoint<std::vector<float>, 3>::coordinate)
    .def_readonly("curvature3d", &math::BezierPoint<std::vector<float>, 3>::curvature);

  class_<std::vector<math::BezierPoint<std::vector<float>, 2>>>("cpp_vector")
    .def(vector_indexing_suite<std::vector<math::BezierPoint<std::vector<float>, 2> > >());
  // 3d BezierPoint Vector
  class_<std::vector<math::BezierPoint<std::vector<float>, 3>>>("cpp_vector3d")
    .def(vector_indexing_suite<std::vector<math::BezierPoint<std::vector<float>, 3> > >());

  class_<std::vector<float>>("float_vector")
    .def(vector_indexing_suite<std::vector<float>>());

  def("to_std_vector", to_std_vector<math::BezierPoint<std::vector<float>, 2> >);
  def("std_vecto_tp_py_list", std_vector_to_py_list<math::BezierPoint<std::vector<float>, 2> >);

  // vector <-> list of BezierPoint 3d
  def("to_std_vector3d", to_std_vector<math::BezierPoint<std::vector<float>, 3> >);
  def("std_vecto_tp_py_list3d", std_vector_to_py_list<math::BezierPoint<std::vector<float>, 3> >);

  def("list_to_vector", to_std_vector<float>);
  def("vector_to_list", std_vector_to_py_list<float>);
}
