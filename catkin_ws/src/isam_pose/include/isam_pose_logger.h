//
// Created by mcube on 4/16/17.
//

#ifndef ISAM_POSE_ISAM_POSE_LOGGER_H
#define ISAM_POSE_ISAM_POSE_LOGGER_H

#include <isam/slam2d.h>
#include <isam/Point2d.h>
#include <isam/Pose2d.h>
#include <Eigen/Dense>
#include <isam_pose_param.h>
#include <vector>
#include <json/json.h>

using namespace std;
using namespace Eigen;
using namespace isam;

typedef Matrix< double ,   Dynamic , 1>  VectorXd;
//typedef Matrix< double ,   Dynamic , Dynamic>  MatrixXd;
struct Logger{
  vector<VectorXd> pose_inc;
  vector<MatrixXd> pose_inc_cov;
  vector<VectorXd> pose_true;
  vector<vector<VectorXd> > shape_inc;

  vector<VectorXd> pose_input;
  vector<VectorXd> shape_input;

  vector<VectorXd> pusher2d__world[2];
  vector<VectorXd> contact_normal__world[2];
  vector<VectorXd> contact_point__world[2];
  vector<VectorXd> contact_force__world[2];

  vector<VectorXd> pose_ekf;
  vector<MatrixXd> pose_ekf_cov;

  vector<bool> has_contact[2];
  vector<bool> has_apriltag;

  vector<double> inc_time;

  vector<Point2d_Node *>* shape_nodes_;
  vector<Pose2d_Node *>* pose_nodes_;

  Logger(){}
  Logger(vector<Point2d_Node *>* shape_nodes, vector<Pose2d_Node *>* pose_nodes){
    shape_nodes_ = shape_nodes;
    pose_nodes_ = pose_nodes;
  }
  void save_to_json(Isam_param * isam_param);

  void render_result(Isam_param * isam_param){
    string cmd30 = string("/home/mcube/push/software/isamShapePose/script/displayResultFingers.py ") +
                   string(isam_param->outfilename) + string(" 30");
    cout << cmd30 << endl;
    string cmd1 = string("/home/mcube/push/software/isamShapePose/script/displayResultFingers.py ") +
                  string(isam_param->outfilename) + string(" 1");
    cout << cmd1 << endl;
    string cmd1_pygame = string("/home/mcube/push/software/isamShapePose/script/displayResultFingers_pygame.py ") +
                  string(isam_param->outfilename) + string(" 1");
    cout << cmd1_pygame << endl;
    //int ret0 = system(cmd30.c_str());
    //int ret1 = system(cmd1.c_str());
  }

};


Json::Value vectorVector_to_json(const vector<VectorXd>& input);

Json::Value vectorMatrix_to_json(const vector<MatrixXd>& input);

Json::Value matrix_to_json(const MatrixXd& input);

Json::Value VectorXd_to_json(const VectorXd& input);

Json::Value vector_to_json(const vector<double>& input);

Json::Value vectorbool_to_json(const vector<bool>& input);

Json::Value jsonArray(int n);

#endif //ISAM_POSE_ISAM_POSE_LOGGER_H
