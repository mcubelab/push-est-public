#pragma once

#include <isam/isam.h>
#include "sp_util.h"
#include "EllipsoidApproxLimitSurface.h"
#include "push_measurement.h"
#include "Point1d.h"

using namespace isam;
typedef NodeT<Point1d> Point1d_Node;

class PushPoseMeasurement : public PushMeasurement{
public:
  Pose2d _last, _current;
    PushPoseMeasurement(Pose2d last, Pose2d current, PushMeasurement& pm):
    _last(last), _current(current), PushMeasurement(pm){

  }
};

class PushPoseMeasurement_C_Factor : public FactorT<PushPoseMeasurement> {
public:
  Point1d_Node*  _c_node;    // c
  EllipsoidApproxLimitSurface* _eals;

  PushPoseMeasurement_C_Factor(
          Point1d_Node* c,
          const PushPoseMeasurement&  push_pose_measurement,
          const Noise&               noise,
          EllipsoidApproxLimitSurface* eals):
             FactorT<PushPoseMeasurement>("PushPoseMeasurement_C_Factor", 3, noise, push_pose_measurement),
             _c_node(c),
             _eals(eals)
  {
    // also need to update _nodes
    _nodes.resize(1);
    _nodes[0] = _c_node;
  }
  
  void initialize() {
  }
  
  VectorXd basic_error(Selector s = LINPOINT) const {
    Vector3d x_now(measurement()._last.vector());
    Vector3d x_next(measurement()._current.vector());
    Vector3d x_delta_global;
    _eals->_c = _c_node->vector(s)[0];
    
    _eals->pushit_slip_cylinder(
       x_now, 
       _measure._probe_center_now, 
       _measure._probe_center_next,
       _measure._contact_point, 
       _measure._contact_normal,
       
       x_delta_global
       );

    Vector3d predicted_delta = x_next - x_now;
    VectorXd err = predicted_delta - x_delta_global;
    err(2) = standardRad(err(2));
    return err;
  }

  Jacobian jacobian(){
    cout << "jacobian0\n" << FactorT<PushPoseMeasurement>::jacobian() << endl;
    cout << "jacobian1\n" << jacobian1() << endl;

    return FactorT<PushPoseMeasurement>::jacobian();
  }

  Jacobian jacobian1() {  // bad need debug
    ////
    Vector3d x_now(measurement()._last.vector());
    Vector3d x_next(measurement()._current.vector());
    Vector3d x_delta_global(0,0,0);
    Vector3d d_x_delta_global__d_c;
    _eals->_c = _c_node->vector0()[0];

    _eals->pushit_slip_cylinder_ad_c(
       x_now,
       _measure._probe_center_now,
       _measure._probe_center_next,
       _measure._contact_point,
       _measure._contact_normal,

       x_delta_global,
       d_x_delta_global__d_c
       );
    Vector3d predicted_delta = x_next - x_now;
    VectorXd err = predicted_delta - x_delta_global;
    ////

    // adding Jacobian term d_err__d_c
    //cout << "x_delta_global" << endl << x_delta_global << endl;
    VectorXd r = sqrtinf() * err;
    Jacobian jac(r);
    MatrixXd d_err__d_c = -d_x_delta_global__d_c;
    jac.add_term(_c_node, sqrtinf() * d_err__d_c);

    return jac;
  }
};
