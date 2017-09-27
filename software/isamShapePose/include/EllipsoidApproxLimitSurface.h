#pragma once
#include "sp_util.h"

using namespace std;
using namespace Eigen;

class EllipsoidApproxLimitSurface{
public:
  double _muc;  // friction coefficient at the contact point
  
  // for ellipsoidal approximation
  double _c;  // hack assume known

  VectorXa _c_a;
  
  EllipsoidApproxLimitSurface(const double muc, const double c);
  
  
  //function [x_new, xc, slip, normvec_now, incontact] = pushit_slip_cylinder(obj,x,xcylinder,xcylinder_end, toviz, xc, normvec)
  
  // x_now: current object pose x_t [x,y,theta]
  // x_cylinder: pusher pose x_t [x,y]       (in global frame)
  // x_cylinder_end: pusher pose x_t+1 [x,y] (in global frame)
  // x_contact: contact point [x,y]                (in global frame)
  // normvec: contact normal () [nx,ny]            (in global frame)
  
  // return 
  // x_delta
  // d_x_delta__d_x_now
  void pushit_slip_cylinder(
     const Vector3d& x_now, 
     const Vector2d& x_cylinder, 
     const Vector2d& x_cylinder_end, 
     const Vector2d& x_contact, 
     const Vector2d& normvec,
     Vector3d& x_delta
     ) ;


  void pushit_slip_cylinder_ad(
     const Vector3d& x_now,
     const Vector2d& x_cylinder,
     const Vector2d& x_cylinder_end,
     const Vector2d& x_contact,
     const Vector2d& normvec,
     Vector3d& x_delta,  //not used
     Matrix3d& d_x_delta__d_xnow
     ) ;


  void pushit_slip_cylinder_ad_c(
          const Vector3d& x_now,
          const Vector2d& x_cylinder,
          const Vector2d& x_cylinder_end,
          const Vector2d& x_contact,
          const Vector2d& normvec,
          Vector3d& x_delta,  //not used
          Vector3d& d_x_delta__d_c
  );
private:
  void get_motion_cone_wrt_object(
    const Vector2d& x_cylinder_object, 
    const Vector2d& x_contact_object, 
    const Vector2d& normvec_obj,
    Vector3d& ql, Vector3d& qr, 
    Vector2d& vl, Vector2d& vr
    );
  
  void get_motion_cone_wrt_object_ad(
    const Vector2a& x_cylinder_object, 
    const Vector2a& x_contact_object, 
    const Vector2a& normvec_obj,
    Vector3a& ql, Vector3a& qr, 
    Vector2a& vl, Vector2a& vr
    );


  void get_motion_cone_wrt_object_ad_c(
          const Vector2a& x_cylinder_object,
          const Vector2a& x_contact_object,
          const Vector2a& normvec_obj,
          Vector3a& ql, Vector3a& qr,
          Vector2a& vl, Vector2a& vr
  );

};

