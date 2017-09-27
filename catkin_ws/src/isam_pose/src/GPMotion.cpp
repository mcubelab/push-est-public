//
// Created by mcube on 2/16/17.
//


#include <GPMotion.h>

GPMotion::GPMotion(string filename){
  // load data from json
  // n_c, n_beta
  // c_lb, c_ub
  // beta_lb, beta_ub
  Json::Value root;
  read_data(filename, root);
  MatrixXd tmp_data_mean, tmp_data_var;  // n*3 matrix
  parse_json_list_into_matrix(root["grid_mean"], tmp_data_mean);  // todo modify data
  parse_json_list_into_matrix(root["grid_variance"], tmp_data_var);  // todo modify data
  _n_c = root["n_c"].asInt();
  _n_beta = root["n_beta"].asInt();
  for(int i=0;i<_n_c;i++)
    for(int j=0;j<_n_beta;j++) {
      _mean_x[i][j] = tmp_data_mean(i * _n_beta + j,0);
      _mean_y[i][j] = tmp_data_mean(i * _n_beta + j,1);
      _mean_theta[i][j] = tmp_data_mean(i * _n_beta + j,2);
      _var_x[i][j] = tmp_data_var(i * _n_beta + j,0);
      _var_y[i][j] = tmp_data_var(i * _n_beta + j,1);
      _var_theta[i][j] = tmp_data_var(i * _n_beta + j,2);
    }

  _c_ub = root["c_ub"].asDouble();
  _c_lb = root["c_lb"].asDouble();
  _beta_ub = root["beta_ub"].asDouble();
  _beta_lb = root["beta_lb"].asDouble();
  _c_step = (_c_ub - _c_lb) / (_n_c-1);
  _beta_step = (_beta_ub - _beta_lb) / (_n_beta-1);
}


void GPMotion::pushit_ad(
        const Vector3d& x_now,
        const Vector2d& x_cylinder,
        const Vector2d& x_cylinder_end,
        const Vector2d& x_contact,
        Vector3d& x_delta,
        Matrix3d& d_x_delta__d_xnow
) {

  // 0. set and declare independent variables and start tape recording
  const int size = 3;
  VectorXa x_now_a(size), x_delta_a(size);
  for (int i = 0; i < size; i++) {
    x_now_a[i] = x_now[i];
  }
  assert(isfinite(x_now[0]));
  CppAD::Independent(x_now_a);

  // 1. transform measurements into object frame

  Vector2a x_cylinder_a(x_cylinder[0], x_cylinder[1]);
  Vector2a x_cylinder_end_a(x_cylinder_end[0], x_cylinder_end[1]);
  Vector2a x_contact_a(x_contact[0], x_contact[1]);
  Vector2a x_contact_object_a;
  Vector2a x_cylinder_object_a;

  // 1.1 pusher position
  transform_to_frame2d_ad(
          x_cylinder_a, x_now_a,
          &x_cylinder_object_a
  );

  // 1.2 transform contact point
  transform_to_frame2d_ad(
          x_contact_a, x_now_a,
          &x_contact_object_a
  );

  // 2. find pusher velocity in object frame
  Vector2a vp;
  rotate_to_frame2d_ad(
          x_cylinder_end_a - x_cylinder_a, x_now_a,
          &vp);

  // 3. compute c and beta
  // 3.1 find which side of rect1, (shape dependent)
  adouble c, beta;
  double sth = 0;
  Vector2a& x = x_cylinder_object_a;
  Vector2a& vel = vp;
  if(x[0] + x[1] >= 0 and x[0] - x[1] <= 0)
    sth = -PI/2;
  else if(x[0] + x[1] <= 0 and x[0] - x[1] <= 0)
    sth = 0;
  else if(x[0] + x[1] <= 0 and x[0] - x[1] >= 0)
    sth = PI/2;
  else if(x[0] + x[1] >= 0 and x[0] - x[1] >= 0)
    sth = PI;

  // 3.1 find beta from vel
  Vector2a x_side0, vel_side0;
  Vector3a frame_side(0, 0, sth);
  rotate_to_frame2d_ad(vel, frame_side, &vel_side0);     // convert it to side 0
  beta = standardRad_a(atan2(vel_side0[1], vel_side0[0]));

  // 3.1.1 special case at the corner
  if(beta > HALFPI or beta < -HALFPI) {
    if (beta > HALFPI)
      sth += HALFPI;
    if (beta < -HALFPI)
      sth -= HALFPI;

    //find beta again
    frame_side = Vector3a(0, 0, sth);
    rotate_to_frame2d_ad(vel, frame_side, &vel_side0);     // convert it to side 0
    beta = standardRad_a(atan2(vel_side0[1], vel_side0[0]));
  }

  // 3.2 find c
  rotate_to_frame2d_ad(x, frame_side, &x_side0);     // convert it to side 0
  c = (x_side0[1] + 0.045) / 0.09;  // todo: need to be moved out
  // cap in [0,1]
  if(c < 0.) c = 0.;
  if(c > 1.) c = 1.;

  //motion_side0 = rotate_to_frame2d(motion[0:2], [0,0,sth])
  //motion_side0 = motion_side0 + [wraptopi(motion[2])]

  // 4. lookup from table
  Vector3a x_delta_side0, x_delta_3a_obj;
  query_GP_ad(c, beta, x_delta_side0);
  // 4.1 put it back to the correct side
  rotate_back_frame2d_ad(x_delta_side0, frame_side, &x_delta_3a_obj);

  // 5. move back to global frame
  Vector3a x_delta_3a;
  rotate_back_frame2d_ad(
          x_delta_3a_obj, x_now_a,
          &x_delta_3a);
  x_delta_a = x_delta_3a;

  // 5. compute the x_delta and d_x_delta__d_xnow
  // 5.1 create f: x -> y and stop tape recording
  CppAD::ADFun<double> f(x_now_a, x_delta_a);
  for (int i = 0; i < x_delta.rows(); i++)
    x_delta[i] = CppAD::Value(x_delta_a[i]);

  VectorXd x_now_tmp(x_now);  // to convert 3d to Xd type
  VectorXd jac = f.Jacobian(x_now_tmp);
  d_x_delta__d_xnow.resize(size, size);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      d_x_delta__d_xnow(i, j) = jac[i * size + j];
    }
  }
}


void GPMotion::query_GP_ad(
        const adouble& c,
        const adouble& beta,
        Vector3a& x_delta
){
  int c_ind = CppAD::Integer((c - _c_lb) / _c_step);
  int beta_ind = CppAD::Integer((beta - _beta_lb) / _beta_step);
//  cout << "c_ind " << c_ind << endl;
//  cout << "CppAD::Value(c) " << c << endl;
//  cout << "beta_ind " << beta_ind << endl;
//  cout << "CppAD::Value(beta) " << beta << endl;

  adouble c1 = c_ind * _c_step + _c_lb;
  adouble c2 = (c_ind+1) * _c_step + _c_lb;
  adouble beta1 = beta_ind * _beta_step + _beta_lb;
  adouble beta2 = (beta_ind+1) * _beta_step + _beta_lb;

  adouble s_c = (c - c1) / _c_step;
  adouble s_beta = (beta - beta1) / _beta_step;

  Vector3a v11 = Vector3a(_mean_x[c_ind][beta_ind], _mean_y[c_ind][beta_ind], _mean_theta[c_ind][beta_ind]);
  Vector3a v12 = Vector3a(_mean_x[c_ind][beta_ind+1], _mean_y[c_ind][beta_ind+1], _mean_theta[c_ind][beta_ind+1]);
  Vector3a v21 = Vector3a(_mean_x[c_ind+1][beta_ind], _mean_y[c_ind+1][beta_ind], _mean_theta[c_ind+1][beta_ind]);
  Vector3a v22 = Vector3a(_mean_x[c_ind+1][beta_ind+1], _mean_y[c_ind+1][beta_ind+1], _mean_theta[c_ind+1][beta_ind+1]);
  x_delta = ( v11 * (c2-c) * (beta2-beta) +
                     v21 * (c-c1) * (beta2-beta) +
                     v12 * (c2-c) * (beta-beta1) +
                     v22 * (c-c1) * (beta-beta1))
                  / (_c_step*_beta_step);

//  cout << "v11 " << v11 << endl;
//  cout << "v21 " << v21 << endl;
//  cout << "v12 " << v12 << endl;
//  cout << "v22 " << v22 << endl;
//  cout << "v11- " << v11 * (c2-c) * (beta2-beta) << endl;
//  cout << "v21- " << v21 * (c-c1) * (beta2-beta) << endl;
//  cout << "v12- " << v12 * (c2-c) * (beta-beta1) << endl;
//  cout << "v22- " << v22 * (c-c1) * (beta-beta1) << endl;
//  cout << "c12- " << c1 << " " << c2 << endl;
//  cout << "beta12- " << beta1 << " " << beta2 << endl;
//  cout << "denom- " << (_c_step*_beta_step) << endl;
//
//  cout << "x_delta " << x_delta << endl;
}