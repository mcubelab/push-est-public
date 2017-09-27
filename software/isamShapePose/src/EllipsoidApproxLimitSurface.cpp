//
// Created by mcube on 1/17/17.
//

#include "EllipsoidApproxLimitSurface.h"




EllipsoidApproxLimitSurface::EllipsoidApproxLimitSurface(const double muc, const double c):
        _muc(muc), _c(c){
}


//function [x_new, xc, slip, normvec_now, incontact] = pushit_slip_cylinder(obj,x,xcylinder,xcylinder_end, toviz, xc, normvec)

// x_now: current object pose x_t [x,y,theta]
// x_cylinder: pusher pose x_t [x,y]       (in global frame)
// x_cylinder_end: pusher pose x_t+1 [x,y] (in global frame)
// x_contact: contact point [x,y]                (in global frame)
// normvec: contact normal () [nx,ny]            (in global frame)

// return
// x_delta
// d_x_delta__d_x_now
void EllipsoidApproxLimitSurface::pushit_slip_cylinder(
        const Vector3d& x_now,
        const Vector2d& x_cylinder,
        const Vector2d& x_cylinder_end,
        const Vector2d& x_contact,
        const Vector2d& normvec,
        Vector3d& x_delta
) {

  // 1. transform measurements into object frame

  Vector2d x_cylinder_object;
  MatrixXd d_cylinder_object__d_xnow;
  Vector2d x_cylinder_end_object;
  MatrixXd d_cylinder_end_object__d_xnow;
  Vector2d x_contact_object;
  Vector2d normvec_object;

  // 1.1 pusher position
  transform_to_frame2d(
          x_cylinder, x_now,
          & x_cylinder_object, NULL
  );

  // 1.2 rotate normvec to object frame

  rotate_to_frame2d(
          normvec, x_now,
          & normvec_object, NULL
  );

  // 1.3 transform contact point

  transform_to_frame2d(
          x_contact, x_now,
          & x_contact_object, NULL
  );

  // 2. get motion cone
  Vector3d ql, qr; Vector2d vl, vr;

  get_motion_cone_wrt_object (
          x_cylinder_object, x_contact_object, normvec_object,
          ql, qr, vl, vr);

  // 3. find pusher velocity in object frame
  Vector2d vp;
  rotate_to_frame2d(
          x_cylinder_end - x_cylinder, x_now,
          & vp, NULL);

  // 4. Branch based on motion cone constraint
  double vx, vy, omega;
  if (cross2d(vr, vp) >= 0 && cross2d(vl, vp) <= 0) { // inside motion cone
    //if(false){  //hack
    double c2 = sq(_c);
    double xc1_2 = sq(x_contact_object[0]);
    double xc2_2 = sq(x_contact_object[1]);
    double xc12 = x_contact_object[0] * x_contact_object[1];

    double denom = c2 + xc1_2 + xc2_2;
    vx = ((c2 + xc1_2) * vp[0]  +  xc12 * vp[1]) / denom;  // vx = ((c^2+xc(1)^2) * vp(1) + xc(1)*xc(2)*vp(2)) / denom;
    vy = ((c2 + xc1_2) * vp[1]  +  xc12 * vp[0]) / denom;  // vy = (xc(1)*xc(2)*vp(1) + (c^2+xc(2)^2) * vp(2)) / denom;
    omega = (x_contact_object[0] * vy - x_contact_object[1] * vx) / c2;                // omega = (xc(1)*vy - xc(2)*vx) / (c^2);

  }
  else{
    double kappa;
    Vector2d vb;
    Vector3d qb, q;
    if(cross2d(vp, vr) > 0){
      //if(true){ //hack // because we know the control policy
      vb = vr;
      qb = qr;
    }
    else{
      vb = vl;
      qb = ql;
    }
    kappa = vp.dot(normvec_object) / vb.dot(normvec_object);
    q = kappa * qb;
    vx = q[0];
    vy = q[1];
    omega = q[2];
  }

  // 5. move back to global frame
  Vector2d v_global;
  rotate_back_frame2d(
          Vector2d(vx, vy), x_now,
          & v_global, NULL);

  // 6. result
  x_delta = Vector3d(v_global[0], v_global[1], omega); // [v_global; omega];
}


void EllipsoidApproxLimitSurface::pushit_slip_cylinder_ad(
        const Vector3d& x_now,
        const Vector2d& x_cylinder,
        const Vector2d& x_cylinder_end,
        const Vector2d& x_contact,
        const Vector2d& normvec,
        Vector3d& x_delta,  //not used
        Matrix3d& d_x_delta__d_xnow
) {

  // 0. set and declare independent variables and start tape recording

  const int size = 3;
  VectorXa x_now_a(size), x_delta_a(size);
  for (int i = 0; i < size; i++) {
    x_now_a[i] = x_now[i];
  }

  CppAD::Independent(x_now_a);

  // 1. transform measurements into object frame

  Vector2a x_cylinder_a(x_cylinder[0], x_cylinder[1]);
  Vector2a x_cylinder_end_a(x_cylinder_end[0], x_cylinder_end[1]);
  Vector2a x_contact_a(x_contact[0], x_contact[1]);
  Vector2a x_contact_object_a;
  Vector2a x_cylinder_object_a;
  Vector2a normvec_object_a;

  // 1.1 pusher position
  transform_to_frame2d_ad(
          x_cylinder_a, x_now_a,
          &x_cylinder_object_a
  );

  // 1.2 rotate normvec to object frame

  Vector2a normvec_a(normvec[0], normvec[1]);
  rotate_to_frame2d_ad(
          normvec_a, x_now_a,
          &normvec_object_a
  );

  // 1.3 transform contact point

  transform_to_frame2d_ad(
          x_contact_a, x_now_a,
          &x_contact_object_a
  );

  // 2. get motion cone
  Vector3a ql, qr;
  Vector2a vl, vr;

  get_motion_cone_wrt_object_ad(
          x_cylinder_object_a, x_contact_object_a, normvec_object_a,
          ql, qr, vl, vr);

  // 3. find pusher velocity in object frame
  Vector2a vp;
  rotate_to_frame2d_ad(
          x_cylinder_end_a - x_cylinder_a, x_now_a,
          &vp);

  // 4. Branch based on motion cone constraint
  adouble vx, vy, omega;
  if (cross2d_ad(vr, vp) >= 0 && cross2d_ad(vl, vp) <= 0) { // inside motion cone
    //if(false){  //hack
    adouble c2 = sq_ad(_c);
    adouble xc1_2 = sq_ad(x_contact_object_a[0]);
    adouble xc2_2 = sq_ad(x_contact_object_a[1]);
    adouble xc12 = x_contact_object_a[0] * x_contact_object_a[1];

    adouble denom = c2 + xc1_2 + xc2_2;
    vx = ((c2 + xc1_2) * vp[0] + xc12 * vp[1]) / denom;  // vx = ((c^2+xc(1)^2) * vp(1) + xc(1)*xc(2)*vp(2)) / denom;
    vy = ((c2 + xc1_2) * vp[1] + xc12 * vp[0]) / denom;  // vy = (xc(1)*xc(2)*vp(1) + (c^2+xc(2)^2) * vp(2)) / denom;
    omega = (x_contact_object_a[0] * vy - x_contact_object_a[1] * vx) /
            c2;                // omega = (xc(1)*vy - xc(2)*vx) / (c^2);

  } else {
    adouble kappa;
    Vector2a vb;
    Vector3a qb, q;
    if (cross2d_ad(vp, vr) > 0) {
      //if(true){ //hack // because we know the control policy
      vb = vr;
      qb = qr;
    } else {
      vb = vl;
      qb = ql;
    }
    kappa = vp.dot(normvec_object_a) / vb.dot(normvec_object_a);
    q = kappa * qb;
    vx = q[0];
    vy = q[1];
    omega = q[2];
  }

  // 5. move back to global frame
  Vector2a v_global;
  rotate_back_frame2d_ad(
          Vector2a(vx, vy), x_now_a,
          &v_global);



  // 6. result
  x_delta_a = Vector3a(v_global[0], v_global[1], omega); // [v_global; omega];


  // 6.1 create f: x -> y and stop tape recording
  CppAD::ADFun<double> f(x_now_a, x_delta_a);
  for (int i = 0; i < x_delta.rows(); i++)
    x_delta[i] = CppAD::Value(x_delta_a[i]);

  VectorXd x_now_tmp(x_now);
  VectorXd jac = f.Jacobian(x_now_tmp);
  d_x_delta__d_xnow.resize(size, size);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      d_x_delta__d_xnow(i, j) = jac[i * size + j];
    }
  }
}


void EllipsoidApproxLimitSurface::pushit_slip_cylinder_ad_c(
        const Vector3d& x_now,
        const Vector2d& x_cylinder,
        const Vector2d& x_cylinder_end,
        const Vector2d& x_contact,
        const Vector2d& normvec,
        Vector3d& x_delta,  //not used
        Vector3d& d_x_delta__d_c
)  {

  // 0. set and declare independent variables and start tape recording

  const int size = 3;
  VectorXa x_now_a(size), x_delta_a(size);
  for(int i = 0; i < size; i++){
    x_now_a[i] = x_now[i];
  }

  _c_a.resize(1,1);
  _c_a[0] = _c;
  CppAD::Independent(_c_a);

  // 1. transform measurements into object frame

  Vector2a x_cylinder_a(x_cylinder[0], x_cylinder[1]);
  Vector2a x_cylinder_end_a(x_cylinder_end[0], x_cylinder_end[1]);
  Vector2a x_contact_a(x_contact[0], x_contact[1]);
  Vector2a x_contact_object_a;
  Vector2a x_cylinder_object_a;
  Vector2a normvec_object_a;

  // 1.1 pusher position
  transform_to_frame2d_ad(
          x_cylinder_a, x_now_a,
          & x_cylinder_object_a
  );

  // 1.2 rotate normvec to object frame

  Vector2a normvec_a(normvec[0], normvec[1]);
  rotate_to_frame2d_ad(
          normvec_a, x_now_a,
          & normvec_object_a
  );

  // 1.3 transform contact point

  transform_to_frame2d_ad(
          x_contact_a, x_now_a,
          & x_contact_object_a
  );

  // 2. get motion cone
  Vector3a ql, qr; Vector2a vl, vr;

  get_motion_cone_wrt_object_ad_c(
          x_cylinder_object_a, x_contact_object_a, normvec_object_a,
          ql, qr, vl, vr);

  // 3. find pusher velocity in object frame
  Vector2a vp;
  rotate_to_frame2d_ad(
          x_cylinder_end_a - x_cylinder_a, x_now_a,
          & vp);

  // 4. Branch based on motion cone constraint
  adouble vx, vy, omega;
  if (cross2d_ad(vr, vp) >= 0 && cross2d_ad(vl, vp) <= 0) { // inside motion cone
    //if(false){  //hack
    adouble c2 = sq_ad(_c_a[0]);
    adouble xc1_2 = sq_ad(x_contact_object_a[0]);
    adouble xc2_2 = sq_ad(x_contact_object_a[1]);
    adouble xc12 = x_contact_object_a[0] * x_contact_object_a[1];

    adouble denom = c2 + xc1_2 + xc2_2;
    vx = ((c2 + xc1_2) * vp[0]  +  xc12 * vp[1]) / denom;  // vx = ((c^2+xc(1)^2) * vp(1) + xc(1)*xc(2)*vp(2)) / denom;
    vy = ((c2 + xc1_2) * vp[1]  +  xc12 * vp[0]) / denom;  // vy = (xc(1)*xc(2)*vp(1) + (c^2+xc(2)^2) * vp(2)) / denom;
    omega = (x_contact_object_a[0] * vy - x_contact_object_a[1] * vx) / c2;                // omega = (xc(1)*vy - xc(2)*vx) / (c^2);

  }
  else{
    adouble kappa;
    Vector2a vb;
    Vector3a qb, q;
    if(cross2d_ad(vp, vr) > 0){
      //if(true){ //hack // because we know the control policy
      vb = vr;
      qb = qr;
    }
    else{
      vb = vl;
      qb = ql;
    }
    kappa = vp.dot(normvec_object_a) / vb.dot(normvec_object_a);
    q = kappa * qb;
    vx = q[0];
    vy = q[1];
    omega = q[2];
  }

  // 5. move back to global frame
  Vector2a v_global;
  rotate_back_frame2d_ad(
          Vector2a(vx, vy), x_now_a,
          & v_global);



  // 6. result
  x_delta_a = Vector3a(v_global[0], v_global[1], omega); // [v_global; omega];

  //adouble c2 = sq_ad(c_a[0]);
//  x_delta_a[0] = sq_ad(_c_a[0]);
//  x_delta_a[1] = sq_ad(_c_a[0]);
//  x_delta_a[2] = sq_ad(_c_a[0]);

  // 6.1 create f: x -> y and stop tape recording
  CppAD::ADFun<double> f(_c_a, x_delta_a);
  for(int i=0; i < x_delta.rows(); i++)
    x_delta[i] = CppAD::Value(x_delta_a[i]);

  VectorXd _c_vec; _c_vec.resize(1,1); _c_vec << _c;
  VectorXd jac = f.Jacobian(_c_vec);

  d_x_delta__d_c.resize(size,1);
  for(int i=0; i<size; i++){
    for(int j=0; j<1; j++){
      d_x_delta__d_c(i,j) = jac[i * 1 + j];
    }
  }
  cout << "_c_vec\n" << _c_vec << endl;
  cout << "d_x_delta__d_c\n" << d_x_delta__d_c << endl;
}

void EllipsoidApproxLimitSurface::get_motion_cone_wrt_object(
        const Vector2d& x_cylinder_object,
        const Vector2d& x_contact_object,
        const Vector2d& normvec_obj,
        Vector3d& ql, Vector3d& qr,
        Vector2d& vl, Vector2d& vr
){

  const double theta = atan(_muc);
  Vector2d fr, fl;


  //fr = rotate_back_frame2d(normvec, [0,0,-theta]);  // friction cone right
  //fl = rotate_back_frame2d(normvec, [0,0,theta]);   // friction cone left

  //~ void rotate_back_frame2d(
  //~ const Vector2d& pt, const Vector3d& x,      // input
  //~ Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  rotate_back_frame2d(
          normvec_obj, Vector3d(0,0,-theta),
          &fr, NULL);

  rotate_back_frame2d(
          normvec_obj, Vector3d(0,0,theta),
          &fl, NULL);


  Vector3d pr(fr[0], fr[1], cross2d(x_contact_object, fr) );  // [fr; cross2d(xc, fr)];
  Vector3d pl(fl[0], fl[1], cross2d(x_contact_object, fl) );

  const double c2 = _c*_c;
  qr = Vector3d(c2 * fr[0], c2 * fr[1], pr[2]);  //qr = [obj.c^2 * fr ; pr(3)];
  ql = Vector3d(c2 * fl[0], c2 * fl[1], pl[2] ); //ql = [obj.c^2 * fl ; pl(3)];

  Vector3d xc3(x_contact_object[0], x_contact_object[1], 0);
  Vector3d vr_tmp = Vector3d(0,0,qr[2]).cross( xc3 ) ;  // vr_tmp = cross([0;0;qr(3)], [xc; 0]); % whole body rotational velocity
  Vector3d vl_tmp = Vector3d(0,0,ql[2]).cross( xc3 );   // vl_tmp = cross([0;0;ql(3)], [xc; 0]);

  vr = qr.head(2) + vr_tmp.head(2); // vr = qr(1:2) + vr_tmp(1:2);  % add translation
  vl = ql.head(2) + vl_tmp.head(2); // vl = ql(1:2) + vl_tmp(1:2);
}

void EllipsoidApproxLimitSurface::get_motion_cone_wrt_object_ad(
        const Vector2a& x_cylinder_object,
        const Vector2a& x_contact_object,
        const Vector2a& normvec_obj,
        Vector3a& ql, Vector3a& qr,
        Vector2a& vl, Vector2a& vr
){

  const adouble theta = atan(_muc);
  Vector2a fr, fl;


  //fr = rotate_back_frame2d(normvec, [0,0,-theta]);  // friction cone right
  //fl = rotate_back_frame2d(normvec, [0,0,theta]);   // friction cone left

  //~ void rotate_back_frame2d(
  //~ const Vector2d& pt, const Vector3d& x,      // input
  //~ Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  rotate_back_frame2d_ad(
          normvec_obj, Vector3a(0,0,-theta),
          &fr);

  rotate_back_frame2d_ad(
          normvec_obj, Vector3a(0,0,theta),
          &fl);


  Vector3a pr(fr[0], fr[1], cross2d_ad(x_contact_object, fr) );  // [fr; cross2d(xc, fr)];
  Vector3a pl(fl[0], fl[1], cross2d_ad(x_contact_object, fl) );

  const adouble c2 = _c*_c;
  qr = Vector3a(c2 * fr[0], c2 * fr[1], pr[2]);  //qr = [obj.c^2 * fr ; pr(3)];
  ql = Vector3a(c2 * fl[0], c2 * fl[1], pl[2] ); //ql = [obj.c^2 * fl ; pl(3)];

  Vector3a xc3(x_contact_object[0], x_contact_object[1], 0);
  Vector3a vr_tmp = Vector3a(0,0,qr[2]).cross( xc3 ) ;  // vr_tmp = cross([0;0;qr(3)], [xc; 0]); % whole body rotational velocity
  Vector3a vl_tmp = Vector3a(0,0,ql[2]).cross( xc3 );   // vl_tmp = cross([0;0;ql(3)], [xc; 0]);

  vr = qr.head(2) + vr_tmp.head(2); // vr = qr(1:2) + vr_tmp(1:2);  % add translation
  vl = ql.head(2) + vl_tmp.head(2); // vl = ql(1:2) + vl_tmp(1:2);
}


void EllipsoidApproxLimitSurface::get_motion_cone_wrt_object_ad_c(
        const Vector2a& x_cylinder_object,
        const Vector2a& x_contact_object,
        const Vector2a& normvec_obj,
        Vector3a& ql, Vector3a& qr,
        Vector2a& vl, Vector2a& vr
){

  const adouble theta = atan(_muc);
  Vector2a fr, fl;


  //fr = rotate_back_frame2d(normvec, [0,0,-theta]);  // friction cone right
  //fl = rotate_back_frame2d(normvec, [0,0,theta]);   // friction cone left

  //~ void rotate_back_frame2d(
  //~ const Vector2d& pt, const Vector3d& x,      // input
  //~ Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  rotate_back_frame2d_ad(
          normvec_obj, Vector3a(0,0,-theta),
          &fr);

  rotate_back_frame2d_ad(
          normvec_obj, Vector3a(0,0,theta),
          &fl);


  Vector3a pr(fr[0], fr[1], cross2d_ad(x_contact_object, fr) );  // [fr; cross2d(xc, fr)];
  Vector3a pl(fl[0], fl[1], cross2d_ad(x_contact_object, fl) );

  const adouble c2 = _c_a[0]*_c_a[0];
  qr = Vector3a(c2 * fr[0], c2 * fr[1], pr[2]);  //qr = [obj.c^2 * fr ; pr(3)];
  ql = Vector3a(c2 * fl[0], c2 * fl[1], pl[2] ); //ql = [obj.c^2 * fl ; pl(3)];

  Vector3a xc3(x_contact_object[0], x_contact_object[1], 0);
  Vector3a vr_tmp = Vector3a(0,0,qr[2]).cross( xc3 ) ;  // vr_tmp = cross([0;0;qr(3)], [xc; 0]); % whole body rotational velocity
  Vector3a vl_tmp = Vector3a(0,0,ql[2]).cross( xc3 );   // vl_tmp = cross([0;0;ql(3)], [xc; 0]);

  vr = qr.head(2) + vr_tmp.head(2); // vr = qr(1:2) + vr_tmp(1:2);  % add translation
  vl = ql.head(2) + vl_tmp.head(2); // vl = ql(1:2) + vl_tmp(1:2);
}
