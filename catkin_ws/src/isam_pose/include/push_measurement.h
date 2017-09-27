#pragma once

#include "sp_util.h"

using namespace isam;


class PushWForceMeasurement{
public:
    vector<VectorXd> _contact_point, _contact_normal;
    vector<VectorXd> _contact_force;
    PushWForceMeasurement(){}

    const int size() const{
      return _contact_point.size();
    }

    PushWForceMeasurement transform_to_frame2d(VectorXd frame) const{
      PushWForceMeasurement tmp;
      const int n = size();
      tmp._contact_point.resize(n);
      tmp._contact_normal.resize(n);
      tmp._contact_force.resize(n);

      for(int i=0; i<n; i++) {
        VectorXd tt;
        ::transform_to_frame2d(VectorXd(_contact_point[i]), frame, &tt);
        tmp._contact_point[i] = tt;
        ::rotate_to_frame2d(VectorXd(_contact_normal[i]), frame, &tt);
        tmp._contact_normal[i] = tt;
        ::rotate_to_frame2d(VectorXd(_contact_force[i]), frame, &tt);
        tmp._contact_force[i] = tt;
      }
      return tmp;
    }


    void write(std::ostream &out) const {
      const int n = size();

      for(int i=0; i<n; i++) {
        out << "["<<i<<"]"<< "(" <<
            " contact_point " << _contact_point[i] <<
            " contact_normal " << _contact_normal[i] <<
            " contact_force " << _contact_force[i] << ")";
      }
    }

    friend std::ostream& operator<< (std::ostream& out, const PushWForceMeasurement& p){
      p.write(out);
      return out;
    }
};


void VectorXd_to_VectorXa(const VectorXd& a, VectorXa& b){
  b.resize(a.size());
  for(int i=0;i<a.size();i++)
    b[i] = a[i];
}

class PushWForceMeasurement_ad{
public:
    vector<VectorXa> _contact_point, _contact_normal;
    vector<VectorXa> _contact_force;
    PushWForceMeasurement_ad(){}
    PushWForceMeasurement_ad(const PushWForceMeasurement& b){
      const int n = b.size();
      _contact_point.resize(n);
      _contact_normal.resize(n);
      _contact_force.resize(n);

      for(int i=0; i<n; i++) {
        VectorXd_to_VectorXa(b._contact_point[i], _contact_point[i]);
        VectorXd_to_VectorXa(b._contact_normal[i], _contact_normal[i]);
        VectorXd_to_VectorXa(b._contact_force[i], _contact_force[i]);
      }
    }

    PushWForceMeasurement_ad transform_to_frame2d_ad(VectorXa frame) const{
      PushWForceMeasurement_ad tmp;
      const int n = size();
      tmp._contact_point.resize(n);
      tmp._contact_normal.resize(n);
      tmp._contact_force.resize(n);

      for(int i=0; i<n; i++) {
        VectorXa tt;
        ::transform_to_frame2d_ad(VectorXa(_contact_point[i]), frame, &tt);
        tmp._contact_point[i] = tt;
        ::rotate_to_frame2d_ad(VectorXa(_contact_normal[i]), frame, &tt);
        tmp._contact_normal[i] = tt;
        ::rotate_to_frame2d_ad(VectorXa(_contact_force[i]), frame, &tt);
        tmp._contact_force[i] = tt;
      }
      return tmp;
    }

    const int size() const{
      return _contact_point.size();
    }

    void write(std::ostream &out) const {
      const int n = size();

      for(int i=0; i<n; i++) {
        out << "["<<i<<"]"<< "(" <<
            " contact_point " << _contact_point[i] <<
            " contact_normal " << _contact_normal[i] <<
            " contact_force " << _contact_force[i] << ")";
      }
    }

    friend std::ostream& operator<< (std::ostream& out, const PushWForceMeasurement_ad& p){
      p.write(out);
      return out;
    }
};


class PushWForceMeasurement_Factor : public FactorT<PushWForceMeasurement> {
public:
    const double ls_c = 0.0383;
    Pose2d_Node*  _pose_now, *_pose_next;    // object pose now

    PushWForceMeasurement_Factor(
            Pose2d_Node* pose_now, Pose2d_Node* pose_next,
            const PushWForceMeasurement& push_measurement, const Noise& noise):
              FactorT<PushWForceMeasurement>("PushWForceMeasurement_Factor", 2, noise, push_measurement),
              _pose_now(pose_now), _pose_next(pose_next)
    {
      // also need to update _nodes
      _nodes.resize(2);
      _nodes[0] = pose_now;
      _nodes[1] = pose_next;
    }

    void initialize() {}

    VectorXd basic_error(Selector s = ESTIMATE) const {
      // 0. prepare object frame
      VectorXd x_now, x_next;
      x_now = _pose_now->vector(s); // (x,y,theta)
      x_next = _pose_next->vector(s);

      // 1. transform measurement to object frame
      PushWForceMeasurement meas__obj;
      meas__obj = _measure.transform_to_frame2d(x_now);

      // 2. sum the forces
      VectorXd f = VectorXd2(0,0); double m = 0;
      for(int i=0; i<meas__obj.size(); i++){
        f = f + meas__obj._contact_force[i];
        m += cross2d(meas__obj._contact_point[i], meas__obj._contact_force[i]);  // r cross F
      }

      // 3. compute velocity
      VectorXd v; double omega;
      rotate_to_frame2d(VectorXd3_to_VectorXd2(x_next - x_now), x_now, &v);
      omega = standardRad(x_next[2] - x_now[2]);

      // 4. apply equation
      double f_x = f[0], f_y = f[1], v_x = v[0], v_y = v[1], c2 = sq(ls_c);
      //cout << "fx" << f_x << " fy" << f_y << " vx" << v_x << " vy" << v_y << " m" << m << " c2" << c2 << " omega" << omega << endl;

      return VectorXd2(v_x * m - f_x * omega * c2,
                       v_y * m - f_y * omega * c2);
    }
    Jacobian jacobian(){
      return jacobian_c();
    }

    Jacobian jacobian_c() const {

      // 0. prepare object frame
      const int size = 3;
      VectorXd x_now, x_next;
      VectorXd x_all(size*2);
      x_now = _pose_now->vector0(); // (x,y,theta)
      x_next = _pose_next->vector0();
      x_all << x_now, x_next;

      // 0.0 put into adouble variable
      VectorXa x_now_a(size), x_next_a(size);
      VectorXa x_all_a(size*2);
      for (int i = 0; i < size; i++)
        x_all_a[i] = x_now[i];

      for (int i = 0; i < size; i++)
        x_all_a[size+i] = x_next[i];
      CppAD::Independent(x_all_a);
      x_now_a = x_all_a.head(size);
      x_next_a = x_all_a.tail(size);

      // 1. transform measurement to object frame
      PushWForceMeasurement_ad _measure_a(_measure), meas__obj_a;
      meas__obj_a = _measure_a.transform_to_frame2d_ad(x_now_a);

           // 2. sum the forces
      VectorXa f_a = VectorXd2_ad(0,0); adouble m_a = 0;
      for(int i=0; i<meas__obj_a.size(); i++){
        f_a = f_a + meas__obj_a._contact_force[i];
        m_a += cross2d_ad(meas__obj_a._contact_point[i], meas__obj_a._contact_force[i]);  // r cross F
      }

      // 3. compute velocity
      VectorXa v_a; adouble omega_a;
      rotate_to_frame2d_ad(VectorXd3_to_VectorXd2_ad(x_next_a - x_now_a), x_now_a, &v_a);
      omega_a = standardRad_a(x_next_a[2] - x_now_a[2]);

      // 4. apply equation
      adouble f_x_a = f_a[0], f_y_a = f_a[1], v_x_a = v_a[0], v_y_a = v_a[1], c2_a = sq(ls_c);
      //cout << "f_x_a" << f_x_a << " f_y_a" << f_y_a << " v_x_a" << v_x_a << " v_y_a" << v_y_a << " m_a" << m_a
      //     << " c2_a" << c2_a << " omega_a" << omega_a << endl;

      VectorXa err_a = VectorXd2_ad(v_x_a * m_a - f_x_a * omega_a * c2_a,
                                    v_y_a * m_a - f_y_a * omega_a * c2_a);
      const int size_r = 2;
      VectorXa r_a = MatrixXd_to_MatrixXa(sqrtinf()) * err_a;
      VectorXd r = sqrtinf() * basic_error(LINPOINT);
      //VectorXd r = error(ESTIMATE);
      MatrixXd d_r__d_x_now(size_r,size);
      MatrixXd d_r__d_x_next(size_r,size);

      // create f: x -> y and stop tape recording
      CppAD::ADFun<double> f1(x_all_a, r_a);

      //for (int i = 0; i < size_r; i++)
      //  r[i] = CppAD::Value(r_a[i]);

      Jacobian jac(r);
      VectorXd jac1_ = f1.Jacobian(x_all);
      for (int i = 0; i < size_r; i++)
        for (int j = 0; j < size; j++) {
          d_r__d_x_now(i, j) = jac1_[i * size*2 + j];
          d_r__d_x_next(i, j) = jac1_[i * size*2 + (j+size)];
        }

      jac.add_term(_pose_now, d_r__d_x_now);
      jac.add_term(_pose_next, d_r__d_x_next);

      return jac;

    }
    void write(std::ostream &out) {
      FactorT<PushWForceMeasurement>::write(out);
      out << "\nerror\n" << error() << endl;
      out << "\nJacobian\n" << this->jacobian_c() << endl;
      out << "\nJacobian_numerical\n" << FactorT<PushWForceMeasurement>::jacobian() << endl;
    }
    friend std::ostream& operator<<(std::ostream& output, PushWForceMeasurement_Factor& e) {
      e.write(output);
      return output;
    }
};
//
//class PushMeasurement_Factor : public FactorT<PushMeasurement> {
//public:
//  Pose2d_Node*  _pose_now;    // object pose now
//  Pose2d_Node*  _pose_next;   // object pose next
//  EllipsoidApproxLimitSurface* _eals;
//  GPMotion* _gpm;
//  int id;
//
//  enum motion_model {USE_EALS,USE_VHGP} _motion_model_option;
//
//  PushMeasurement_Factor(
//    Pose2d_Node* pose_now,
//    Pose2d_Node* pose_next,
//    const PushMeasurement&  push_measurement,
//    const Noise&               noise,
//    EllipsoidApproxLimitSurface* eals,
//    GPMotion* gpm,
//    motion_model motion_model_option):
//      FactorT<PushMeasurement>("PushMeasurement_Factor", 3, noise, push_measurement),
//      _pose_now(pose_now),
//      _pose_next(pose_next),
//      _eals(eals),
//      _gpm(gpm),
//      _motion_model_option(motion_model_option)
//  {
//    // also need to update _nodes
//    _nodes.resize(2);
//    _nodes[0] = pose_now;
//    _nodes[1] = pose_next;
//  }
//
//  void initialize() {
//  }
//
//  VectorXd basic_error(Selector s = ESTIMATE) const {
//    Vector3d x_now(_pose_now->vector(s));
//    Vector3d x_next(_pose_next->vector(s));
//    Vector3d x_delta_global;
//
//    if (_motion_model_option == USE_EALS){
//      _eals->pushit_slip_cylinder(
//              x_now,
//              _measure._probe_center_now,
//              _measure._probe_center_next,
//              _measure._contact_point,
//              _measure._contact_normal,
//
//              x_delta_global
//      );
//    }
//    else if(_motion_model_option == USE_VHGP) {
//      Matrix3d d_x_delta_global__d_x_now;  // dummy, won't use
//      _gpm->pushit_ad(
//              x_now,
//              _measure._probe_center_now,
//              _measure._probe_center_next,
//              _measure._contact_point,
//
//              x_delta_global,
//              d_x_delta_global__d_x_now
//      );
//    }
//
//    Vector3d predicted_delta = _pose_next->vector(s) - _pose_now->vector(s);
//    predicted_delta(2) = standardRad(predicted_delta(2));
//    VectorXd err = predicted_delta - x_delta_global;
//    err(2) = standardRad(err(2));
//    return err;
//
//  }
//
//  Jacobian jacobian(){
//    return jacobian_c();
//  }
//
//  Jacobian jacobian_c() const {
//    ////
//    Vector3d x_now(_pose_now->vector0());
//    Vector3d x_next(_pose_next->vector0());
//    Vector3d x_delta_global(0,0,0);
//    Matrix3d d_x_delta_global__d_x_now;
//
//    if(_motion_model_option == USE_EALS) {
//      _eals->pushit_slip_cylinder_ad(
//              x_now,
//              _measure._probe_center_now,
//              _measure._probe_center_next,
//              _measure._contact_point,
//              _measure._contact_normal,
//
//              x_delta_global,
//              d_x_delta_global__d_x_now
//      );
//    }
//    else if(_motion_model_option == USE_VHGP) {
//      _gpm->pushit_ad(
//              x_now,
//              _measure._probe_center_now,
//              _measure._probe_center_next,
//              _measure._contact_point,
//
//              x_delta_global,
//              d_x_delta_global__d_x_now
//      );
//    }
//    Vector3d predicted_delta = x_next - x_now;
//    predicted_delta(2) = standardRad(predicted_delta(2));
//    VectorXd err = predicted_delta - x_delta_global;
//    err(2) = standardRad(err(2));
//    ////
//
//    // adding Jacobian term d_err__d_x_now
//    //cout << "x_delta_global" << endl << x_delta_global << endl;
//    VectorXd r = sqrtinf() * err;
//    Jacobian jac(r);
//    MatrixXd d_err__d_x_now = -d_x_delta_global__d_x_now - MatrixXd::Identity(3,3);
//    jac.add_term(_pose_now, sqrtinf() * d_err__d_x_now);
//
//    // adding Jacobian term d_err__d_x_next
//    MatrixXd d_err__d_x_next(3,3);
//    d_err__d_x_next << 1, 0, 0,
//                       0, 1, 0,
//                       0, 0, 1;
//
//    jac.add_term(_pose_next, sqrtinf() * d_err__d_x_next);
//    return jac;
//  }
//
//  void write(std::ostream &out) const {
//    FactorT<PushMeasurement>::write(out);
//    out << "\nerror\n" << error() << endl;
//    out << "\njacobian\n" << jacobian_c() << endl;
//
//  }
//  friend std::ostream& operator<<(std::ostream& output, PushMeasurement_Factor& e) {
//    e.write(output);
//    return output;
//  }
//};

