//
// Created by mcube on 1/6/17.
//

#include "sp_util.h"
#include <iostream>
#include <cppad/example/cppad_eigen.hpp>
#include <sys/time.h>
#include <isam/robust.h>
#include <isam_pose_param.h>
#include <isam_pose_logger.h>

#include <Eigen/Geometry> // for quaternion

using namespace isam;

// % pt: col vector
// % x: col vector [x,y,theta]' which defines a 2d frame;
void transform_to_frame2d( const Vector2d& pt, const Vector3d& x,// input
                           Vector2d* pt_ret, MatrixXd* dpt_ret_dx) { // output
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

void transform_to_frame2d(const VectorXd pt /*(x,y)*/, const VectorXd x /*(x,y,th)*/ , // input
                          VectorXd* pt_ret /*(x,y)*/                      // output
) {

  const double theta = x(2), c = cos(theta), s = sin(theta);

  Matrix2d T;
  T << c,  s,
       -s, c ;

  VectorXd p = pt - x.head(2);
  *pt_ret = T * p;
}

void transform_to_frame2d_ad( const Vector2a& pt, const Vector3a& x,  // input
                              Vector2a* pt_ret) { // output

  adouble theta = x(2);
  adouble c = cos(theta);
  adouble s = sin(theta);

  Matrix2a T;
  T << c,  s,
      -s, c ;

  Vector2a p = pt - x.head(2);
  *pt_ret = T * p;
}


void transform_to_frame2d_ad( const VectorXa& pt, const VectorXa& x,  // input
                              VectorXa* pt_ret) { // output
  adouble theta = x(2);
  adouble c = cos(theta), s = sin(theta);

  MatrixXa T(2,2);
  T << c,  s,
       -s, c ;

  VectorXa p = pt - x.head(2);
  *pt_ret = T * p;
}

void rotate_to_frame2d( const Vector2d& pt, const Vector3d& x,      // input
                        Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  const double theta = x(2), c = cos(theta), s = sin(theta);

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

void rotate_to_frame2d( const VectorXd pt, const VectorXd x,      // input
                        VectorXd* pt_ret) {   // output
  const double theta = x(2), c = cos(theta), s = sin(theta);

  Matrix2d T;
  T << c,  s,
          -s, c ;

  VectorXd p = pt;
  *pt_ret = T * p;
}

void rotate_to_frame2d_ad( const Vector2a& pt, const Vector3a& x,      // input
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

void rotate_to_frame2d_ad( const VectorXa& pt, const VectorXa& x,      // input
                           VectorXa* pt_ret) {   // output
  adouble theta = x(2);
  adouble c = cos(theta), s = sin(theta);

  MatrixXa T(2,2);
  T << c,  s,
          -s, c ;

  VectorXa p = pt;
  *pt_ret = T * p;
}

void rotate_back_frame2d( const Vector2d& pt, const Vector3d& x,      // input
                          Vector2d* pt_ret, MatrixXd* dpt_ret_dx) {   // output

  const double theta = x(2), c = cos(theta), s = sin(theta);

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

void rotate_back_frame2d_ad( const Vector2a& pt, const Vector3a& x,      // input
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

void rotate_back_frame2d_ad( const Vector3a& pt, const Vector3a& x,      // input
                             Vector3a* pt_ret) {   // output
  Vector2a pt2(pt[0], pt[1]);
  Vector2a pt2_ret;
  rotate_back_frame2d_ad(pt2, x, &pt2_ret);
  (*pt_ret)[0] = pt2_ret[0];
  (*pt_ret)[1] = pt2_ret[1];
  (*pt_ret)[2] = pt[2];
}

double cross2d( const Vector2d& a, const Vector2d& b){
  return a[0]*b[1] - a[1]*b[0];
}

double cross2d( const VectorXd& a, const VectorXd& b){
  return a[0]*b[1] - a[1]*b[0];
}

adouble cross2d_ad( const Vector2a& a, const Vector2a& b){
  return a[0]*b[1] - a[1]*b[0];
}

VectorXd rospose_to_posevec( geometry_msgs::Pose a){
  return VectorXd7(a.position.x, a.position.y, a.position.z,
               a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
}

geometry_msgs::Pose posevec_to_rospose( VectorXd a){
  geometry_msgs::Pose b;
  geometry_msgs::Point pt;
  geometry_msgs::Quaternion q;
  pt.x = a[0]; pt.y = a[1]; pt.z = a[2];
  q.x = a[3]; q.y = a[4]; q.z = a[5]; q.w = a[6];
  b.position = pt; b.orientation = q;

  return b;
}

Matrix3d posevec_to_rotMat( VectorXd a){
  return Rot3d::quat_to_wRo(Quaterniond(a[6],a[3],a[4],a[5]));
}

VectorXd transform_back(VectorXd a, VectorXd frame){
  // a = [x,y,z,qx,qy,qz,qw]
  // frame = [x,y,z,qx,qy,qz,qw]
  //return Tframe * Ta;
  Matrix3d R_a = posevec_to_rotMat(a);
  Matrix3d R_frame = posevec_to_rotMat(frame);
  Quaterniond q_new = Rot3d::wRo_to_quat(R_frame * R_a);
  VectorXd t_new = R_frame * a.head(3) + frame.head(3);
  return VectorXd7(t_new[0],t_new[1],t_new[2],q_new.x(),q_new.y(),q_new.z(),q_new.w());
}

VectorXd transform_to(VectorXd a, VectorXd frame){
  // a = [x,y,z,qx,qy,qz,qw]
  // frame = [x,y,z,qx,qy,qz,qw]
  //return Ta/Tframe;
  Matrix3d R_a = posevec_to_rotMat(a);
  Matrix3d R_frame_inv = posevec_to_rotMat(frame).transpose();
  Quaterniond q_new = Rot3d::wRo_to_quat(R_frame_inv * R_a);
  VectorXd t_new = R_frame_inv * (a.head(3) - frame.head(3));
  return VectorXd7(t_new[0],t_new[1],t_new[2],q_new.x(),q_new.y(),q_new.z(),q_new.w());
}

VectorXd poseTransform(VectorXd a, string sourceFrame, string targetFrame, tf::TransformListener &lr){
  geometry_msgs::Pose a_ros = posevec_to_rospose(a);
  geometry_msgs::PoseStamped a_ps;
  a_ps.pose = a_ros;
  a_ps.header.frame_id = sourceFrame;

  geometry_msgs::PoseStamped b_ps;
  geometry_msgs::Pose b_ros;
  lr.transformPose(targetFrame, a_ps, b_ps);
  return rospose_to_posevec(b_ps.pose);
}

// a = (x,y,z)
VectorXd pointTransform(VectorXd a, string sourceFrame, string targetFrame, tf::TransformListener &lr){
  a = VectorXd7(a[0],a[1],a[2],0,0,0,1);
  geometry_msgs::Pose a_ros = posevec_to_rospose(a);
  geometry_msgs::PoseStamped a_ps;
  a_ps.pose = a_ros;
  a_ps.header.frame_id = sourceFrame;

  geometry_msgs::PoseStamped b_ps;
  geometry_msgs::Pose b_ros;
  lr.transformPose(targetFrame, a_ps, b_ps);
  return VectorXd3(b_ps.pose.position.x, b_ps.pose.position.y, b_ps.pose.position.z);
}


VectorXd lookupTransform(string sourceFrame, string targetFrame, tf::TransformListener &lr){
  tf::StampedTransform st;
  lr.lookupTransform(targetFrame, sourceFrame, ros::Time(), st);
  tf::Quaternion q = st.getRotation();
  tf::Vector3 p = st.getOrigin();
  return VectorXd7(p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
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

VectorXd parse_jsonval_into_vector(Json::Value jsonvalue) {
  int n = jsonvalue.size();
  VectorXd output(n);

  for (int i = 0; i < n; i++)  // Iterates over the sequence elements.
    output(i) = jsonvalue[i].asDouble();
  return output;
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

double quat_to_yaw(VectorXd q_now) {
  double yaw, pitch, roll;
  Quaterniond q(q_now[3], q_now[0], q_now[1], q_now[2]);
  Rot3d::quat_to_euler(q, yaw, pitch, roll);
  return yaw;
}

bool quat_is_good(VectorXd q_now) {
  double yaw;
  Quaterniond q(q_now[3], q_now[0], q_now[1], q_now[2]);
  //Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  //yaw = euler[0];
  if((q.toRotationMatrix().row(2) * VectorXd3(0,0,1))(0,0) < 0.9){
    return false;
  }
  return true;
}

double norm(const geometry_msgs::Vector3& a)
{
  return a.x*a.x + a.y*a.y + a.z*a.z;
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


#include <Eigen/Dense>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

/*
  We need a functor that can pretend it's const,
  but to be a good random number generator
  it needs mutable state.
*/
namespace Eigen {
    namespace internal {
        template<typename Scalar>
        struct scalar_normal_dist_op
        {
            static boost::mt19937 rng;    // The uniform pseudo-random algorithm
            mutable boost::normal_distribution<Scalar> norm;  // The gaussian combinator

            EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

            template<typename Index>
            inline const Scalar operator() (Index, Index = 0) const { return norm(rng); }
        };

        template<typename Scalar> boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

        template<typename Scalar>
        struct functor_traits<scalar_normal_dist_op<Scalar> >
        { enum { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false }; };
    } // end namespace internal
} // end namespace Eigen
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

MatrixXd VarianceCalculator::calculate(){
  MatrixXd mat(size(), at(0).rows());
  for(int i=0;i<size();i++){
    MatrixXd tmp = at(i);
    mat.row(i) = tmp.transpose();
  }
  //MatrixXd t1 = mat.colwise().mean();
  //MatrixXd centered = mat.rowwise() - mat.colwise().mean();
  MatrixXd centered = mat;
  MatrixXd cov = (centered.transpose() * centered) / double(mat.rows() - 1);
  return cov;
}

MatrixXd VarianceCalculator::calculateMean(){
  MatrixXd mat(size(), at(0).rows());
  for(int i=0;i<size();i++){
    MatrixXd tmp = at(i);
    mat.row(i) = tmp.transpose();
  }
  //MatrixXd t1 = mat.colwise().mean();
  return mat.colwise().mean();
}

MatrixXd VarianceCalculator::calculateInfo(){
  return calculate().inverse();
}


void VarianceCalculator::save(string filename){
  ofstream fout(filename);
  fout << vectorVector_to_json(*this);
}

void ROSVector3_to_vecpoint2D(const geometry_msgs::Vector3& b, VectorXd& a){
  a = VectorXd(2);
  a << b.x, b.y;
}

VectorXd VectorXd2(const double& a, const double& b){
  VectorXd vec(2);
  vec << a, b;
  return vec;
}

VectorXd VectorXd3(const double& a, const double& b, const double& c){
  VectorXd vec(3);
  vec << a, b, c;
  return vec;
}

VectorXd VectorXd7(const double& a1, const double& a2, const double& a3, const double& a4, const double& a5, const double& a6, const double& a7){
  VectorXd vec(7);
  vec << a1, a2, a3, a4, a5, a6, a7;
  return vec;
}

VectorXd VectorXd3_to_VectorXd2(const VectorXd& a){
  return VectorXd2(a[0], a[1]);
}

VectorXa VectorXd3_to_VectorXd2_ad(const VectorXa& a){
  return VectorXd2_ad(a[0], a[1]);
}

VectorXa VectorXd2_ad(const double& a, const double& b){
  VectorXa vec(2);
  vec << a, b;
  return vec;
}

VectorXa VectorXd2_ad(const adouble& a, const adouble& b){
  VectorXa vec(2);
  vec << a, b;
  return vec;
}

MatrixXa MatrixXd_to_MatrixXa(const MatrixXd& a){
  const int nr = a.rows(), nc = a.cols();
  MatrixXa m(a.rows(), a.cols());
  for(int i=0; i<nr; i++)
    for(int j=0; j<nc; j++)
      m(i,j) = a(i,j);
  return m;
}

double robust_cam_cost_function(double d) {
  return cost_huber(d, 1.0);
}

