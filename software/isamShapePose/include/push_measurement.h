#pragma once

#include <isam/isam.h>
#include "sp_util.h"
#include "EllipsoidApproxLimitSurface.h"
#include "GPMotion.h"

using namespace isam;


class PushMeasurement{
public:
  Vector2d _contact_point, _contact_normal;
  Vector2d _probe_center_now, _probe_center_next;

  PushMeasurement(
    Vector2d contact_point, 
    Vector2d contact_normal, 
    Vector2d probe_center_now, 
    Vector2d probe_center_next):
    
    _contact_point(contact_point),  
    _contact_normal(contact_normal), 
    _probe_center_now(probe_center_now),
    _probe_center_next(probe_center_next)
  {
    
  }



  void write(std::ostream &out) const {
    out << "(" <<
        " contact_point " << _contact_point <<
        " contact_normal " << _contact_normal <<
        " probe_center_now "  <<_probe_center_now <<
        " probe_center_next " << _probe_center_next << ")";
  }
  friend std::ostream& operator<< (std::ostream& out, const PushMeasurement& p){
    p.write(out);
    return out;
  }
};

class PushMeasurement_Factor : public FactorT<PushMeasurement> {
public:
  Pose2d_Node*  _pose_now;    // object pose now
  Pose2d_Node*  _pose_next;   // object pose next
  EllipsoidApproxLimitSurface* _eals;
  GPMotion* _gpm;
  int id;

  enum motion_model {USE_EALS,USE_VHGP} _motion_model_option;

  PushMeasurement_Factor(
    Pose2d_Node* pose_now, 
    Pose2d_Node* pose_next,
    const PushMeasurement&  push_measurement, 
    const Noise&               noise,
    EllipsoidApproxLimitSurface* eals,
    GPMotion* gpm,
    motion_model motion_model_option):
      FactorT<PushMeasurement>("PushMeasurement_Factor", 3, noise, push_measurement),
      _pose_now(pose_now),
      _pose_next(pose_next),
      _eals(eals),
      _gpm(gpm),
      _motion_model_option(motion_model_option)
  {
    // also need to update _nodes
    _nodes.resize(2);
    _nodes[0] = pose_now;
    _nodes[1] = pose_next;
  }
  
  void initialize() {
  }
  
  VectorXd basic_error(Selector s = ESTIMATE) const {
    Vector3d x_now(_pose_now->vector(s));
    Vector3d x_next(_pose_next->vector(s));
    Vector3d x_delta_global;

    if (_motion_model_option == USE_EALS){
      _eals->pushit_slip_cylinder(
              x_now,
              _measure._probe_center_now,
              _measure._probe_center_next,
              _measure._contact_point,
              _measure._contact_normal,

              x_delta_global
      );
    }
    else if(_motion_model_option == USE_VHGP) {
      Matrix3d d_x_delta_global__d_x_now;  // dummy, won't use
      _gpm->pushit_ad(
              x_now,
              _measure._probe_center_now,
              _measure._probe_center_next,
              _measure._contact_point,

              x_delta_global,
              d_x_delta_global__d_x_now
      );
    }

    Vector3d predicted_delta = _pose_next->vector(s) - _pose_now->vector(s);
    predicted_delta(2) = standardRad(predicted_delta(2));
    VectorXd err = predicted_delta - x_delta_global;
    err(2) = standardRad(err(2));
    return err;

  }

  Jacobian jacobian(){
    return jacobian_c();
  }

  Jacobian jacobian_c() const {
    ////
    Vector3d x_now(_pose_now->vector0());
    Vector3d x_next(_pose_next->vector0());
    Vector3d x_delta_global(0,0,0);
    Matrix3d d_x_delta_global__d_x_now;

    if(_motion_model_option == USE_EALS) {
      _eals->pushit_slip_cylinder_ad(
              x_now,
              _measure._probe_center_now,
              _measure._probe_center_next,
              _measure._contact_point,
              _measure._contact_normal,

              x_delta_global,
              d_x_delta_global__d_x_now
      );
    }
    else if(_motion_model_option == USE_VHGP) {
      _gpm->pushit_ad(
              x_now,
              _measure._probe_center_now,
              _measure._probe_center_next,
              _measure._contact_point,

              x_delta_global,
              d_x_delta_global__d_x_now
      );
    }
    Vector3d predicted_delta = x_next - x_now;
    predicted_delta(2) = standardRad(predicted_delta(2));
    VectorXd err = predicted_delta - x_delta_global;
    err(2) = standardRad(err(2));
    ////

    // adding Jacobian term d_err__d_x_now
    //cout << "x_delta_global" << endl << x_delta_global << endl;
    VectorXd r = sqrtinf() * err;
    Jacobian jac(r);
    MatrixXd d_err__d_x_now = -d_x_delta_global__d_x_now - MatrixXd::Identity(3,3);
    jac.add_term(_pose_now, sqrtinf() * d_err__d_x_now);
    
    // adding Jacobian term d_err__d_x_next
    MatrixXd d_err__d_x_next(3,3);
    d_err__d_x_next << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;
    
    jac.add_term(_pose_next, sqrtinf() * d_err__d_x_next);
    return jac;
  }

  void write(std::ostream &out) const {
    FactorT<PushMeasurement>::write(out);
    out << "\nerror\n" << error() << endl;
    out << "\njacobian\n" << jacobian_c() << endl;

  }
  friend std::ostream& operator<<(std::ostream& output, PushMeasurement_Factor& e) {
    e.write(output);
    return output;
  }
};
