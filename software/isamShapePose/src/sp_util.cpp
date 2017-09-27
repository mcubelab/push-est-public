//
// Created by mcube on 1/6/17.
//

#include <Eigen/Dense>
#include <iostream>
#include <cppad/example/cppad_eigen.hpp>
#include <json/json.h>
#include "sp_util.h"
#include <sys/time.h>
using namespace isam;

// % pt: col vector
// % x: col vector [x,y,theta]' which defines a 2d frame;
void transform_to_frame2d(
        // input
        const Vector2d& pt,
        const Vector3d& x,
        // output
        Vector2d* pt_ret,
        MatrixXd* dpt_ret_dx) {

  double theta = x(2);
  double c = cos(theta);
  double s = sin(theta);

  Matrix2d T;
  T << c,  s,
          -s, c ;

  Vector2d p = pt - x.head(2);
  *pt_ret = T * p;

  if(dpt_ret_dx){
    dpt_ret_dx->resize(2,3);
    *dpt_ret_dx << -c, -s, (-s*p(0) + c*p(1)),
            s, -c, (-c*p(0) - s*p(1));
  }
}

void transform_to_frame2d_ad(
        // input
        const Vector2a& pt,
        const Vector3a& x,
        // output
        Vector2a* pt_ret) {

  adouble theta = x(2);
  adouble c = cos(theta);
  adouble s = sin(theta);

  Matrix2a T;
  T << c,  s,
          -s, c ;

  Vector2a p = pt - x.head(2);
  *pt_ret = T * p;

}

void rotate_to_frame2d(
        const Vector2d& pt, const Vector3d& x,      // input
        Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  double theta = x(2);
  double c = cos(theta);
  double s = sin(theta);

  Matrix2d T;
  T << c,  s,
          -s, c ;

  Vector2d p = pt;
  *pt_ret = T * p;

  if(dpt_ret_dx){
    dpt_ret_dx->resize(2,3);
    *dpt_ret_dx << 0, 0, (-s*p(0) + c*p(1)),
            0, 0, (-c*p(0) - s*p(1));
  }
}


void rotate_to_frame2d_ad(
        const Vector2a& pt, const Vector3a& x,      // input
        Vector2a* pt_ret) {   // output

  adouble theta = x(2);
  adouble c = cos(theta);
  adouble s = sin(theta);

  Matrix2a T;
  T << c,  s,
          -s, c ;

  Vector2a p = pt;
  *pt_ret = T * p;

}


void rotate_back_frame2d(
        const Vector2d& pt, const Vector3d& x,      // input
        Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  double theta = x(2);
  double c = cos(theta);
  double s = sin(theta);

  Matrix2d T;
  T << c, -s,
          s,  c ;

  Vector2d p = pt;
  *pt_ret = T * p;

  if(dpt_ret_dx){
    dpt_ret_dx->resize(2,3);
    *dpt_ret_dx << 0, 0, (-s*p(0) - c*p(1)),
            0, 0,  (c*p(0) - s*p(1));
  }
}

void rotate_back_frame2d_ad(
        const Vector2a& pt, const Vector3a& x,      // input
        Vector2a* pt_ret) {   // output

  adouble theta = x(2);
  adouble c = cos(theta);
  adouble s = sin(theta);

  Matrix2a T;
  T << c, -s,
          s,  c ;

  Vector2a p = pt;
  *pt_ret = T * p;

}

void rotate_back_frame2d_ad(
        const Vector3a& pt, const Vector3a& x,      // input
        Vector3a* pt_ret) {   // output
  Vector2a pt2(pt[0], pt[1]);
  Vector2a pt2_ret;
  rotate_back_frame2d_ad(pt2, x, &pt2_ret);
  (*pt_ret)[0] = pt2_ret[0];
  (*pt_ret)[1] = pt2_ret[1];
  (*pt_ret)[2] = pt[2];
}

double cross2d(const Vector2d& a, const Vector2d& b){
  return a[0]*b[1] - a[1]*b[0];
}

adouble cross2d_ad(const Vector2a& a, const Vector2a& b){
  return a[0]*b[1] - a[1]*b[0];
}

template<typename T>
T sqr(T x){
  return x*x;
}

double sq(double x){
  return x*x;
}

adouble sq_ad(adouble x){
  return x*x;
}

std::ostream& operator<< (std::ostream& out, const Vector2d& p){
  out << "[" << p[0] << "," << p[1] << "]";
  return out;
}

void read_data(string filename, Json::Value& root){
  ifstream fin(filename);
  fin >> root;
}

void parse_json_list_into_matrix(Json::Value jsonvalue, MatrixXd& output) {
  int n = jsonvalue.size();
  int nn = jsonvalue[0u].size();
  output.resize(n, nn);

  for (int i = 0; i < n; i++)  // Iterates over the sequence elements.
    for (int j = 0; j < nn; j++)
      output(i, j) = jsonvalue[i][j].asDouble();

  //cout << all_contact << endl;
}

// subrate will be like 25;
void downsample(const MatrixXd& input, const int subrate, MatrixXd& output) {
  int n = input.rows();
  int output_n = (n + subrate - 1) / subrate;
  output.resize(output_n, input.cols());
  for (int i = 0; i < output_n; i++){
    output.row(i) = input.row(subrate*i);
  }

}

Json::Value matrix_to_json(const MatrixXd& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.rows();
  int m = input.cols();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    v_json[i] = Json::Value(Json::arrayValue);
    v_json[i].resize(m);
    for (int j = 0; j < m; j++)
      v_json[i][j] = input(i,j);
  }
  return v_json;
}


Json::Value vector_to_json(const vector<double>& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.size();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    v_json[i] = input[i];
  }
  return v_json;
}

void save_to_json(const vector<Point2d_Node*>& shape_nodes,
                  const vector<Pose2d_Node*>& pose_nodes,
                  const vector<vector<VectorXd> >& shape_inc,
                  const vector<VectorXd>& pose_inc,
                  const vector<MatrixXd>& pose_inc_cov,
                  const vector<VectorXd>& shape_input,
                  const vector<VectorXd>& pose_input,
                  const vector<double>& inc_time,
                  const MatrixXd& d,
                  const double probe_radius,
                  const string label,
                  const string startdate,
                  const string filename,
                  const string shape_id,
                  const VectorXd& offset,
                  const MatrixXd& turned_norm,
                  const MatrixXd& vicon_norm){
  Json::Value jsonvalue;
//  jsonvalue["M_star"];  // (2, 8)
//  jsonvalue["x_star"];  // (3, 400)
//  jsonvalue["x_true"];  // (3, 400)  // repetitive in d
//  jsonvalue["d"];       // (16, 400)
//  jsonvalue["radius"];  //
//  jsonvalue["startdate"];  //
  {
    Json::Value M_star_json(Json::arrayValue);
    int nM = shape_nodes.size();
    int nMdim = 2;
    M_star_json.resize(nMdim);
    for (int i = 0; i < nMdim; i++) {
      M_star_json[i] = Json::Value(Json::arrayValue);
      M_star_json[i].resize(nM);
      for (int j = 0; j < nM; j++)
        M_star_json[i][j] = shape_nodes[j]->vector()[i];
    }
    jsonvalue["M_star"] = M_star_json;
  }
  {
    Json::Value x_star_json(Json::arrayValue);
    int nx = pose_nodes.size();
    int nxdim = 3;
    x_star_json.resize(nxdim);
    for (int i = 0; i < nxdim; i++) {
      x_star_json[i] = Json::Value(Json::arrayValue);
      x_star_json[i].resize(nx);
      for (int j = 0; j < nx; j++)
        x_star_json[i][j] = pose_nodes[j]->vector()[i];
    }
    jsonvalue["x_star"] = x_star_json;
  }

  {
    Json::Value shape_inc_json(Json::arrayValue);
    int nt = shape_inc.size();
    int nM = shape_nodes.size();
    int nMdim = 2;
    shape_inc_json.resize(nt);
    for (int t = 0; t < nt; t++) {
      shape_inc_json[t] = Json::Value(Json::arrayValue);
      shape_inc_json[t].resize(nM);
      for (int i = 0; i < nM; i++) {
        shape_inc_json[t][i] = Json::Value(Json::arrayValue);
        shape_inc_json[t][i].resize(nMdim);
        for (int j = 0; j < nMdim; j++) {
          shape_inc_json[t][i][j] = shape_inc[t][i][j];
        }
      }
    }
    jsonvalue["shape_inc"] = shape_inc_json;  // t * nM * nMdim
  }

  {
    Json::Value pose_inc_json(Json::arrayValue);
    int nt = pose_inc.size();
    int ndim = 3;
    pose_inc_json.resize(nt);
    for (int t = 0; t < nt; t++) {
      pose_inc_json[t] = Json::Value(Json::arrayValue);
      pose_inc_json[t].resize(ndim);
      for (int i = 0; i < ndim; i++) {
        pose_inc_json[t][i] = pose_inc[t][i];
      }
    }
    jsonvalue["pose_inc"] = pose_inc_json;  // t * ndim(3)
  }

  {
    Json::Value pose_inc_cov_json(Json::arrayValue);
    int nt = pose_inc_cov.size();
    int ndim = 3;
    pose_inc_cov_json.resize(nt);
    for (int t = 0; t < nt; t++) {
      pose_inc_cov_json[t] = Json::Value(Json::arrayValue);
      pose_inc_cov_json[t].resize(ndim);
      for (int i = 0; i < ndim; i++) {
        pose_inc_cov_json[t][i] = Json::Value(Json::arrayValue);
        pose_inc_cov_json[t][i].resize(ndim);
        for (int j = 0; j < ndim; j++) {
          pose_inc_cov_json[t][i][j] = pose_inc_cov[t](i,j);
        }
      }
    }
    jsonvalue["pose_inc_cov"] = pose_inc_cov_json;  // t * ndim(3)*ndim(3)
  }

  {
    Json::Value shape_input_json(Json::arrayValue);
    int nM = shape_nodes.size();
    int nMdim = 2;
    shape_input_json = Json::Value(Json::arrayValue);
    shape_input_json.resize(nM);
    for (int i = 0; i < nM; i++) {
      shape_input_json[i] = Json::Value(Json::arrayValue);
      shape_input_json[i].resize(nMdim);
      for (int j = 0; j < nMdim; j++) {
        shape_input_json[i][j] = shape_input[i][j];
      }
    }
    jsonvalue["shape_input"] = shape_input_json;  // t * nM * nMdim
  }

  {
    Json::Value pose_input_json(Json::arrayValue);
    int nt = pose_input.size();
    int ndim = 3;
    pose_input_json.resize(nt);
    for (int t = 0; t < nt; t++) {
      pose_input_json[t] = Json::Value(Json::arrayValue);
      pose_input_json[t].resize(ndim);
      for (int i = 0; i < ndim; i++) {
        pose_input_json[t][i] = pose_input[t][i];
      }
    }
    jsonvalue["pose_input"] = pose_input_json;  // t * ndim(3)
  }

  jsonvalue["inc_time"] = vector_to_json(inc_time);

  jsonvalue["d"] = matrix_to_json(d);
  jsonvalue["turned_norm"] = matrix_to_json(turned_norm);
  jsonvalue["vicon_norm"] = matrix_to_json(vicon_norm);


  jsonvalue["probe_radius"] = probe_radius;
  jsonvalue["label"] = label;
  jsonvalue["startdate"] = startdate;
  jsonvalue["shape_id"] = shape_id;
  Json::Value offset_json(Json::arrayValue);
  offset_json.resize(2);
  offset_json[0u] = offset[0];
  offset_json[1] = offset[1];
  jsonvalue["offset"] = offset_json;

  ofstream fout(filename);
  fout << jsonvalue;
}

string get_timestamp(){
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];
  char buffer2 [80];

  time (&rawtime);
  struct timeval tp;
  gettimeofday(&tp, NULL);

  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"%Y%m%d_%H%M%S_",timeinfo);

  sprintf(buffer2, "%03ld", tp.tv_usec/1000);

  return string(buffer)  // till second
         + string(buffer2);     // add millisecond

}

void pk(){
  cout << "Press any key to continue" << endl;
  cin.get();
}

double quat_to_yaw(Quaterniond q_now) {
  double yaw, pitch, roll;
  Rot3d::quat_to_euler(q_now, yaw, pitch, roll);
  return yaw;
}

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
adouble standardRad_a(adouble t) {
  while (t > PI)
    t -= TWOPI;
  while (t <= -PI)
    t += TWOPI;
  return t;
}


// prepare some random numbers ~ Multivariate gaussian distribution
MatrixXd get_randn(int data_len){

  int size = 3; // Dimensionality (rows)
  int nn=data_len;     // How many samples (columns) to draw
  Eigen::internal::scalar_normal_dist_op<double> randN; // Gaussian functor
  Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1); // Seed the rng
  // Define mean and covariance of the distribution
  Eigen::VectorXd mean(size);
  Eigen::MatrixXd covar(size,size);

  mean  <<  0,  0,  0;
  covar <<  sq(0.001),  0, 0,
          0,  sq(0.001), 0,
          0, 0, sq(0.001/(0.289*0.09));
  Eigen::MatrixXd normTransform(size,size);

  Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);

  // We can only use the cholesky decomposition if
  // the covariance matrix is symmetric, pos-definite.
  // But a covariance matrix might be pos-semi-definite.
  // In that case, we'll go to an EigenSolver
  if (cholSolver.info()==Eigen::Success) {
    // Use cholesky solver
    normTransform = cholSolver.matrixL();
  } else {
    // Use eigen solver
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    normTransform = eigenSolver.eigenvectors()
                    * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }
  Eigen::MatrixXd randn = (normTransform
                           * Eigen::MatrixXd::NullaryExpr(size,nn,randN)).colwise()
                          + mean;
  return randn;
  //// end of random numbers
}