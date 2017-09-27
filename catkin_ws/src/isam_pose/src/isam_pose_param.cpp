//
// Created by peterkty on 4/16/17.
//


#include <isam_pose_param.h>
#include <sp_util.h>

namespace butter{
  #include "shape_db_butter_shape.h"
}
Isam_param::Isam_param(int argc, char* argv[])
{

  // 1. parsing options
  using namespace boost::program_options;
  options_description desc{"Options"};
  desc.add_options()
          ("var", "To calculate mean & variance or not")
          ("jsonin", value< string >(), "Use json as input")
          ("label", value< string >(), "label")
          ("no-stationary", "Disable stationary cost")
          ("no-apriltag", "Disable apriltag cost")
          ("no-contact", "Disable contact cost")
          ("no-push", "Disable push cost")
          ("shape-id", value< string >(), "shape_id")
          ("surface-id", value< string >(), "surface_id")
          ("maxstep", value< int >(), "maxstep")
          ("doEKF", "To use EKF or not")
          ("pause", "To use pause when using jsonin or not");

  variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);


  to_calculate_var = vm.count("var") ;

  use_json_input = vm.count("jsonin");

  doEKF = vm.count("doEKF") ;

  pause = vm.count("pause") ;

  if(use_json_input) {
    json_input_path = vm["jsonin"].as<string>();
  }

  if(vm.count("label")) {
    label = vm["label"].as<string>();
  }

  shape_id = "rect1";
  if(vm.count("shape-id")){
    shape_id = vm["shape-id"].as<string>();
    if(shape_id == "ellip2"){
      tag_id = 2;
      tag_obj = VectorXd7(0,0.03893,0.009, 0,0,0,1);
    }
    if(shape_id == "butter"){
      tag_id = 1;
      tag_obj = VectorXd7(0,0.03661,0.009, 0,0,0,1);
    }
    if(shape_id == "rect1"){
      tag_id = 0;
      tag_obj = VectorXd7(0.02,0.02,0.009, 0,0,0,1);
    }
  }

  if(vm.count("surface-id")){
    surface_id = vm["surface-id"].as<string>();
  }
  maxstep = 4000;
  if(vm.count("maxstep")){
    maxstep = vm["maxstep"].as<int>();
  }


  add_stationary_factor = !vm.count("no-stationary");
  add_apriltag_factor = !vm.count("no-apriltag");
  add_contact_factor = !vm.count("no-contact");
  add_push_factor = !vm.count("no-push");
  if(!add_stationary_factor) label += "no-stationary";
  if(!add_apriltag_factor) label += "no-apriltag";
  if(!add_contact_factor) label += "no-contact";
  if(!add_push_factor) label += "no-push";

  to_inc = !to_calculate_var;
  ///////////////////

  offset.resize(2);
  offset << 0.35, -0.03;

  // 1. preparing noise matrices
  MatrixXd noise3_pose_mat(3,3), noise3_pose_rel_mat(3,3), noise3_pose_apriltag_mat(3,3);
  noise3_pose_mat << sq(0.01), 0, 0,
          0, sq(0.01), 0,
          0, 0, sq(0.01 / (0.289 * 0.09));

//  noise3_pose_rel_mat << sq(0.0002), 0, 0,
//          0, sq(0.0002), 0,
//          0, 0, sq(0.0002 / (0.289 * 0.09));
  noise3_pose_rel_mat << 7.72677e+07, 0,     0,
                         0 , 6.14872e+07,       0,
                         0 ,     0   ,  1979.96;



  noise3_pose_apriltag_mat << 47622,  2900.41, -1351.28,
                              2900.41,   144017, -1626.07,
                              -1351.28, -1626.07,  431.782;

  noise3_pose = Covariance(noise3_pose_mat);
  noise3_pose_rel = Information(noise3_pose_rel_mat);
  noise3_pose_apriltag = Information(noise3_pose_apriltag_mat);


  MatrixXd noise2_motion_mat(2,2);
  noise2_motion_mat <<   5e+10, 0,
                         0,  5e+10;

  MatrixXd noise3_measure_mat(3,3);
  noise3_measure_mat <<  1, 0, 0,
          0,   2e+6,  0,
          0,  0,  2e+6;

  noise2_motion = Information(noise2_motion_mat);
  noise3_measure = Information(noise3_measure_mat);



  // 2. setting shape
  char filename_buff[1024];
  sprintf(filename_buff, "multipush_shape=%s_surface=%s_rep=%04d",
          shape_id.c_str(),
          surface_id.c_str(),
          rep_id);
  label = string("isam-") + filename_buff + (label == "" ? "": "-" + label);

  string data_path = std::getenv("DATA_BASE");
  startdate = get_timestamp();
  outfilename = data_path + "/result/shapeRecon-" + startdate + "-" + label + ".json";
  vcoutfilename_prefix = data_path + "/result/shapeRecon-" + startdate + "-" + label;

  butter_shape_len = butter::butter_shape_len_;
  butter_shape.resize(butter_shape_len);

  for(int i=0;i<butter_shape_len;i++){
    butter_shape[i].resize(2);
    butter_shape[i].assign(butter::butter_shape_[i], butter::butter_shape_[i]+2);
  }

  // 3. setting nM
  if(shape_id == "rect1")
    nM = 4;
  else if(shape_id == "butter" || shape_id == "ellip2")
    nM = butter_shape_len/sub_butter_len+1;

}
