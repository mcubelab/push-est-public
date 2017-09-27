#ifndef SP_UTIL_H
#define SP_UTIL_H

#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <cmath>
#include <iostream>
#include <cppad/example/cppad_eigen.hpp>
#include <json/json.h>
#include <time.h>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <isam/Slam.h>
#include <isam/slam2d.h>
#include <isam/Point2d.h>
#include <isam/Pose2d.h>
#include <isam/util.h>
#include <isam/Noise.h>
#include <isam/Properties.h>

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

typedef vector<VectorXd> vecVec;

// % pt: col vector
// % x: col vector [x,y,theta]' which defines a 2d frame;
void transform_to_frame2d( const Vector2d& pt, const Vector3d& x, // input
                           Vector2d* pt_ret, MatrixXd* dpt_ret_dx) ;  // output

void transform_to_frame2d( const VectorXd pt, const VectorXd x,  // input
                           VectorXd* pt_ret);                        // output

void rotate_to_frame2d( const VectorXd pt, const VectorXd x,  // input
                        VectorXd* pt_ret);                        // output

void transform_to_frame2d_ad( const Vector2a& pt, const Vector3a& x,      // input
                              Vector2a* pt_ret);                          // output

void transform_to_frame2d_ad( const VectorXa& pt, const VectorXa& x,  // input
                              VectorXa* pt_ret); // output

void rotate_to_frame2d( const Vector2d& pt, const Vector3d& x,      // input
                        Vector2d* pt_ret, MatrixXd* dpt_ret_dx);

void rotate_to_frame2d_ad( const Vector2a& pt, const Vector3a& x,      // input
                           Vector2a* pt_ret) ;

void rotate_to_frame2d_ad( const VectorXa& pt, const VectorXa& x,      // input
                             VectorXa* pt_ret) ;

void rotate_back_frame2d( const Vector2d& pt, const Vector3d& x,      // input
                          Vector2d* pt_ret, MatrixXd* dpt_ret_dx) ;

void rotate_back_frame2d_ad( const Vector2a& pt, const Vector3a& x,      // input
                             Vector2a* pt_ret) ;

void rotate_back_frame2d_ad( const Vector3a& pt, const Vector3a& x,      // input
                             Vector3a* pt_ret);   // won't change the third element

double cross2d(const Vector2d& a, const Vector2d& b);

adouble cross2d_ad(const Vector2a& a, const Vector2a& b);

VectorXd rospose_to_posevec(geometry_msgs::Pose a);

geometry_msgs::Pose posevec_to_rospose(VectorXd a);

VectorXd transform_back(VectorXd a, VectorXd frame);

VectorXd transform_to(VectorXd a, VectorXd frame);

VectorXd poseTransform(VectorXd a, string sourceFrame, string targetFrame, tf::TransformListener &lr);

VectorXd pointTransform(VectorXd a, string sourceFrame, string targetFrame, tf::TransformListener &lr);

VectorXd lookupTransform(string sourceFrame, string targetFrame, tf::TransformListener &lr);

double sq(double x);

adouble sq_ad(adouble x);

std::ostream& operator<< (std::ostream& out, const Vector2d& p);

void read_data(string filename, Json::Value& root);

void parse_json_list_into_matrix(Json::Value value, MatrixXd& output) ;

VectorXd parse_jsonval_into_vector(Json::Value jsonvalue);

string get_timestamp();

void pk(); // press any key to continue

double quat_to_yaw(VectorXd q_now);

bool quat_is_good(VectorXd q_now);

double norm(const geometry_msgs::Vector3& a);

class VarianceCalculator: public vector<VectorXd> {
public:
    MatrixXd calculate();
    MatrixXd calculateMean();
    MatrixXd calculateInfo();
    void save(string filename);
};

adouble standardRad_a(adouble t);

MatrixXd get_randn(int data_len);

void ROSVector3_to_VectorXd(const geometry_msgs::Vector3& b, VectorXd& a);

VectorXd VectorXd2(const double& a, const double& b);
VectorXd VectorXd3(const double& a, const double& b, const double& c);
VectorXd VectorXd7(const double& a1, const double& a2, const double& a3, const double& a4, const double& a5, const double& a6, const double& a7);
VectorXd VectorXd3_to_VectorXd2(const VectorXd& a);
VectorXa VectorXd3_to_VectorXd2_ad(const VectorXa& a);
MatrixXa MatrixXd_to_MatrixXa(const MatrixXd& a);
VectorXa VectorXd2_ad(const double& a, const double& b);
VectorXa VectorXd2_ad(const adouble& a, const adouble& b);

enum {  LEFT = 0, RIGHT = 1 };

double robust_cam_cost_function(double d);

#endif /* !SP_UTIL_H */
