//
// Created by mcube on 4/16/17.
//

#ifndef ISAM_POSE_ISAM_POSE_PARAM_H
#define ISAM_POSE_ISAM_POSE_PARAM_H

#include <Eigen/Dense>
#include <isam/Noise.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace Eigen;
using namespace isam;

typedef Matrix< double ,   Dynamic , 1>  VectorXd;

struct Isam_param {
  VectorXd offset;
  Noise noise3_pose, noise3_pose_rel, noise3_pose_apriltag;
  double probe_radius = 0.00316;  // two finger probe need take out

  Noise noise3_motion, noise2_motion;
  Noise noise3_measure;
  int nM = 4;
  string shape_id = "rect1";
  string surface_id = "plywood";
  VectorXd tag_obj;
  int tag_id = 0;

  const int sub_butter_len = 10;
  int rep_id = 0;

  string label;

  string data_path;
  string startdate;
  string outfilename, vcoutfilename_prefix;
  int butter_shape_len;
  vector<vector<double> > butter_shape;

  bool to_calculate_var;
  bool to_inc;

  bool doEKF;
  bool pause;

  bool use_json_input;
  string json_input_path;

  Isam_param();

  Isam_param(int argc, char* argv[]);


  bool add_stationary_factor;
  bool add_apriltag_factor;
  bool add_contact_factor;
  bool add_push_factor;

  int maxstep;
};

#endif //ISAM_POSE_ISAM_POSE_PARAM_H
