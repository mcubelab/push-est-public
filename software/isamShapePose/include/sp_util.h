#ifndef SP_UTIL_H
#define SP_UTIL_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <cppad/example/cppad_eigen.hpp>
#include <json/json.h>
#include <isam/isam.h>
#include <time.h>

using namespace isam;
using namespace std;
using namespace Eigen;
using CppAD::AD;

typedef Matrix<bool, Dynamic, 1 > VectorXb;
typedef Matrix<double, 2, Dynamic> Matrix2Xd;
typedef Matrix<double, 1, Dynamic> Matrix1Xd;
typedef AD<double> adouble;

typedef Matrix<adouble, Dynamic, Dynamic> MatrixXa;
typedef Matrix<adouble, 4, 1> Vector4a;
typedef Matrix<adouble, 2, 2> Matrix2a;
typedef Matrix<adouble, 3, 3> Matrix3a;

typedef Matrix< double ,   Dynamic , 1>  VectorXd;
typedef Matrix< adouble , Dynamic , 1>  VectorXa;
typedef Matrix< adouble , 3 , 1>  Vector3a;
typedef Matrix< adouble , 2 , 1>  Vector2a;
typedef Matrix< adouble , 1 , 1>  Vector1a;

// % pt: col vector
// % x: col vector [x,y,theta]' which defines a 2d frame;
void transform_to_frame2d(
  // input
    const Vector2d& pt, 
    const Vector3d& x,      
  // output
    Vector2d* pt_ret, 
    MatrixXd* dpt_ret_dx) ;

void transform_to_frame2d_ad(
  // input
    const Vector2a& pt, 
    const Vector3a& x,      
  // output
    Vector2a* pt_ret) ;

void rotate_to_frame2d(
  const Vector2d& pt, const Vector3d& x,      // input
  Vector2d* pt_ret, MatrixXd* dpt_ret_dx);


void rotate_to_frame2d_ad(
  const Vector2a& pt, const Vector3a& x,      // input
  Vector2a* pt_ret) ;


void rotate_back_frame2d(
  const Vector2d& pt, const Vector3d& x,      // input
  Vector2d* pt_ret, MatrixXd* dpt_ret_dx) ;

void rotate_back_frame2d_ad(
  const Vector2a& pt, const Vector3a& x,      // input
  Vector2a* pt_ret) ;

void rotate_back_frame2d_ad(
  const Vector3a& pt, const Vector3a& x,      // input
  Vector3a* pt_ret);   // won't change the third element

double cross2d(const Vector2d& a, const Vector2d& b);

adouble cross2d_ad(const Vector2a& a, const Vector2a& b);

template<typename T>
T sqr(T x);

double sq(double x);

adouble sq_ad(adouble x);

std::ostream& operator<< (std::ostream& out, const Vector2d& p);

void read_data(string filename, Json::Value& root);

void parse_json_list_into_matrix(Json::Value value, MatrixXd& output) ;

// subrate will be like 25;
void downsample(const MatrixXd& input, const int subrate, MatrixXd& output) ;


void save_to_json(const vector<Point2d_Node*>& shape_nodes,
                  const vector<Pose2d_Node*>& pose_nodes,
                  const vector<vector<VectorXd> >& shape_inc,
                  const vector<VectorXd>& pose_inc,
                  const vector<MatrixXd>& pose_cov_inc,
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
                  const MatrixXd& vicon_norm);

string get_timestamp();

void pk(); // press any key to continue

double quat_to_yaw(Quaterniond q_now);

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

class VarianceCalculator: public vector<VectorXd> {
public:
    MatrixXd calculate(){
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

    MatrixXd calculateMean(){
      MatrixXd mat(size(), at(0).rows());
      for(int i=0;i<size();i++){
        MatrixXd tmp = at(i);
        mat.row(i) = tmp.transpose();
      }
      //MatrixXd t1 = mat.colwise().mean();
      return mat.colwise().mean();
    }

    MatrixXd calculateInfo(){
      return calculate().inverse();
    }
};

adouble standardRad_a(adouble t);

MatrixXd get_randn(int data_len);
#endif /* !SP_UTIL_H */
