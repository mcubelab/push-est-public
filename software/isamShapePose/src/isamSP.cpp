/**
 * @file isamSP.cpp
 * @brief The entry point for tactile shape and pose estimation.
 * @author Peter KT Yu
 *
 */
 
#include <isam/isam.h>
#include "contact_measurement.h"
#include "push_measurement.h"
#include "bot_lcmgl_client/lcmgl.h"
#include "viz.h"
#include "shape_db_butter_shape.h"
#include <cstdlib>

using namespace std;
using namespace isam;
using namespace Eigen;


int main(int argc, char* argv[]) {
  tic("setup");
  const string data_path = std::getenv("DATA_BASE");
  //string filename = data_path + "/inputdata/all_contact_real_shape=rect1_rep=0000.json";
  const string shape_id = "rect1";  //"circle", "rect1", "butter", "ellip2"
  string surface_id = "plywood";
  if(argc >= 2) surface_id = argv[1];
  int rep_id = 0;
  if(argc >= 4) rep_id = atoi(argv[3]);
  const double stepsize = 0.001;
  char filename_buff[1024];
  bool use_apriltag = true;
  string apriltag_suff = use_apriltag ? "_april": "";
  sprintf(filename_buff, "multipush_shape=%s_surface=%s_rep=%04d_step=%.04f%s",
                                     shape_id.c_str(),
                                     surface_id.c_str(),
                                     rep_id,
                                     stepsize,
                                     apriltag_suff.c_str());
  string filename = data_path+"/inputdata/multi_pushes/"+filename_buff+".json";
  cout << "input filename:" << filename << endl;

  // solution related parameters
  const int sub_butter_len = 10;
  int nM = butter_shape_len/sub_butter_len+1;   // number of control points
  const string init_shape = shape_id;  //"circle", "rect1", "butter", "ellip2"
  if(init_shape == "rect1") nM = 4;
  const double guess_shape_radius = 0.045;
  const double ls_c = 0.0383;  // hack  calculated: 0.0383
  const int subrate = 1; // subsample rate
  bool to_calculate_var = false;  // do inc/batch should be false
  if(argc >= 3) to_calculate_var = argv[2][0] == 't';
  const bool inc = !to_calculate_var;
  const bool batchopt = !to_calculate_var;
  const bool add_motion_factor = true;
  const bool add_contact_factor = true;
  const bool add_pose_prior_factor = true;
  const double cam_keep_rate = 1.0;
  string init_pose = "apriltag";  //gt //"last_estimate", "groundtruth", "groundtruth_w_noise", "apriltag"
  string cont_init_pose = "apriltag";               //gt
  string camera_input_source = "apriltag";  //"groundtruth_w_noise"
  if(to_calculate_var)
    init_pose = cont_init_pose = "groundtruth";
  //const string gpmfile = "/home/mcube/pnpushdata/contour_following/vhgp_grid.json";
  const string gpmfile = "/home/mcube/pnpushdata/straight_line/plywood/rect1/vhgp_grid_400.json";
  const PushMeasurement_Factor::motion_model motion_model_option =
          PushMeasurement_Factor::USE_EALS;
          //PushMeasurement_Factor::USE_VHGP;
  string motion_model = motion_model_option == PushMeasurement_Factor::USE_EALS ? "eals" : "vhgp";

  // output file names
  string label = string("isam-") + filename_buff + "-" + motion_model + "-different_surface";

  const string startdate = get_timestamp();
  const string outfilename = data_path + "/result/shapeRecon-" + startdate + "-" + label + ".json";
  const string outsummaryfilename = data_path + "/result/summary-" + startdate + "-" + label + ".txt";

  // visualization
  const bool toprint = false;
  const bool toviz = true;
  const bool topause = false;
  const bool render_result = !to_calculate_var && true;
  const bool to_print_graph = false;
  // "normal correction"
  const bool fromVicon = false;
  const bool turn = false;
  const unsigned int sleep_microseconds = 10000;

  VarianceCalculator vc_norm, vc_push, vc_pose;  // for identify variance
  // reading measurement data
  Json::Value data_root;
  read_data(filename, data_root);
  const double probe_radius = data_root["probe_radius"].asDouble();
  const double muc = data_root["muc"].asDouble();  // hack, need to find the right value
  MatrixXd all_contact, dtmp, d;
  parse_json_list_into_matrix(data_root["all_contact"], all_contact);
  // todo use gaussian filter
  downsample(all_contact, subrate, dtmp);
  d = dtmp.transpose();
  const int data_len = d.cols() ;
  const double scale_measure = 1;
  //const int data_len = 10;  //hack
  const bool isreal = data_root["isreal"].asBool();
  //const string surface_id = data_root["surface_id"].asString();
  //const string shape_id = data_root["shape_id"].asString();
  const string probe_id = data_root["probe_id"].asString();
  const Vector2d offset(data_root["offset"][0u].asDouble(), data_root["offset"][1].asDouble());

  Eigen::MatrixXd randn = get_randn(data_len);

  // useful noises
  Noise noise3 = Information(1. * eye(3));
  MatrixXd noise3_motion_mat(3,3);
  if(motion_model_option == PushMeasurement_Factor::USE_VHGP) {
    noise3_motion_mat <<      2.33352e+07,     -655344,    -92724.4,
                              -655344, 1.81109e+07,    -44787.9,
                              -92724.4,    -44787.9,     3778.67;  // calculated from multipush rect1 data0
  }
  else// if (motion_model_option == PushMeasurement_Factor::USE_EALS)
  {
    noise3_motion_mat << 4.33414e+06,      102853,     9117.95,
                         102853,      678287,    -588.467,
                         9117.95,    -588.467,     1789.78;  // calculated from multipush rect1 data0
    if(init_shape == "butter") {
      noise3_motion_mat <<  2.16839e+08, -5.26921e+07  ,    -581208,
                            -5.26921e+07 , 5.04049e+07,   -1.402e+06,
                            -581208  , -1.402e+06  ,     115360;
    }
  }
  if(argc >= 5 && argv[4][0] == 't') {
    double tmp[9];
    for (int i = 0; i < 9; i++) cin >> tmp[i];
    noise3_motion_mat = Map<Matrix<double, 3, 3, RowMajor> >(tmp);
  }

  MatrixXd noise3_measure_mat(3,3);

  noise3_measure_mat << 2.37957e+06, 2.20583e+06,      354091,
                        2.20583e+06, 2.98992e+02,      277646,
                        354091,      277646, 1.61678e+02;  // calculated from multipush

  noise3_measure_mat <<  1, 0, 0,
                         0,   2e+10,  0,
                         0,  0,  2e+10;

  if(init_shape == "butter") {
    noise3_measure_mat <<  1.68414e+07, 1.04477e+07, 8.71477e+06,
                          1.04477e+07, 1.07232e+07,   3.001e+06,
                          8.71477e+06,   3.001e+06, 1.24938e+07;
  }
  if(argc >= 6 && argv[5][0] == 't') {
    double tmp[9];
    for (int i = 0; i < 9; i++) cin >> tmp[i];
    noise3_measure_mat = Map<Matrix<double, 3, 3, RowMajor> >(tmp);
    cout << "noise3_measure_mat\n" << noise3_measure_mat << endl;
  }

  Noise noise3_motion = Information(noise3_motion_mat);
  Noise noise3_measure = Information(noise3_measure_mat);
  cout << "noise3_measure_sqrtinfo\n" << noise3_measure.sqrtinf() << endl;
  MatrixXd noise2_shape_mat(2,2);
  noise2_shape_mat << sq(0.00001), 0,
                      0, sq(0.00001);

  MatrixXd noise3_pose_mat(3,3), noise3_pose_rel_mat(3,3), noise3_pose_apriltag_mat(3,3);
  noise3_pose_mat << sq(0.001), 0, 0,
                     0, sq(0.001), 0,
                     0, 0, sq(0.001/(0.289*0.09));
  noise3_pose_rel_mat << sq(0.001), 0, 0,
          0, sq(0.001), 0,
          0, 0, sq(0.0005/(0.289*0.09));
  noise3_pose_apriltag_mat <<      57795.2 , 61215.7  , 2643.8,
                                    61215.7 ,  169552 ,-3397.51,
                                    2643.8, -3397.51,  1124.31;
  if(argc >= 7 && argv[6][0] == 't') {
    double tmp[9];
    for (int i = 0; i < 9; i++) cin >> tmp[i];
    noise3_pose_apriltag_mat = Map<Matrix<double, 3, 3, RowMajor> >(tmp);
  }

  Noise noise2_shape = Covariance(noise2_shape_mat);
  Noise noise3_pose = Covariance(noise3_pose_mat);
  Noise noise3_pose_rel = Covariance(noise3_pose_rel_mat);
  Noise noise3_pose_apriltag = Information(noise3_pose_apriltag_mat);
  //Noise noise3_pose = Information(zero_mat);
  // Create the pose graph
  Slam slam;
  {
    Properties p = slam.properties();
    p.quiet = false;
    p.verbose = true;
    p.mod_batch = 100;
    slam.set_properties(p);
  }
  vector<Point2d_Node *> shape_nodes;
  vector<Pose2d_Node *> pose_nodes;

  vector<VectorXd> pose_inc;
  vector<MatrixXd> pose_inc_cov;
  vector<vector<VectorXd> > shape_inc;

  vector<VectorXd> pose_input;
  vector<VectorXd> shape_input;

  vector<Point2d *> shape_data;
  vector<ContactMeasurement *> contact_data;
  vector<PushMeasurement *> push_data;

  vector<double> inc_time;

  LcmGL_Visualizer visualizer(shape_nodes, pose_nodes, shape_data, contact_data,
                              toviz, topause, sleep_microseconds);
  Screen_Displayer screen_displayer(shape_nodes, pose_nodes, toprint);

  const Covariances& covariances = slam.covariances();

  // 1.1 add node for pose at t = 0
  Pose2d_Node *new_pose_node_0 = new Pose2d_Node();

  slam.add_node(new_pose_node_0);
  pose_nodes.push_back(new_pose_node_0);
  // 1.2 add data for pose prior for t = 0
  Pose2d origin;
  if (init_pose == "last_estimate")
    origin = Pose2d(offset[0], offset[1], 0.);
  else if (init_pose == "groundtruth") {
    int t = 0;
    Vector3d groundth_x_now(d(6, t), d(7, t), quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t))));
    origin = Pose2d(groundth_x_now);
  }
  else if (init_pose == "groundtruth_w_noise") {
    int t = 0;
    Vector3d groundth_x_now(d(6, t)+randn(0, t), d(7, t)+randn(1, t),
                            quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t)))+randn(2,t));
    origin = Pose2d(groundth_x_now);
  }
  else if (init_pose == "apriltag") {
    int t = 0;
    Vector3d groundth_x_now(d(17, t), d(18, t), d(24, t));
    origin = Pose2d(groundth_x_now);
  }

  Pose2d_Factor *pose_prior_factor_0 = new Pose2d_Factor(pose_nodes[0], origin, noise3_pose);
  // 1.3 add factor for pose prior for t = 0
  slam.add_factor(pose_prior_factor_0);

  // 2.1 add shape nodes
  for (int i = 0; i < nM; i++) {
    Point2d_Node *new_point_node = new Point2d_Node();
    //slam.add_node(new_point_node); // remove shape from estimation
    shape_nodes.push_back(new_point_node);
  }

  // 2.2 add data for shape prior
  for (int i = 0; i < nM; i++) {
    Point2d *new_point = NULL;
    if (init_shape == "circle") {
      const double rad = (i + 0.5) / nM * M_PI * 2;
      new_point = new Point2d(cos(rad) * guess_shape_radius,
                              sin(rad) * guess_shape_radius);
    }
    else if (init_shape == "ellip2") {
      const double rad = (i + 0.5) / nM * M_PI * 2;
      new_point = new Point2d(cos(rad) * 0.0525,
                              sin(rad) * 0.0654);
    }
    else if (init_shape == "rect1") {
      double side_2 = 0.045;
      double tick = 1.0 / float(nM / 4) * side_2 * 2;
      if (i < nM / 4)
        new_point = new Point2d(side_2 - i * tick, side_2);
      else if (i < nM / 4 * 2)
        new_point = new Point2d(-side_2, side_2 - (i - nM / 4) * tick);
      else if (i < nM / 4 * 3)
        new_point = new Point2d(-side_2 + (i - nM / 4 * 2) * tick, -side_2);
      else
        new_point = new Point2d(side_2, (i - nM / 4 * 3) * tick - side_2);
    } else if (init_shape == "butter") {
        new_point = new Point2d(butter_shape[i*sub_butter_len][0], butter_shape[i*sub_butter_len][1]);
    }
    shape_data.push_back(new_point);
  }


  // 2.3 add factor for shape prior
  for (int i = 0; i < nM; i++) {
    Point2d_Factor *prior = new Point2d_Factor(shape_nodes[i], *(shape_data[i]), noise2_shape);
    // add it to the graph
    shape_nodes[i]->init(*shape_data[i]);
    //slam.add_factor(prior); //remove shape from estimation
  }

  for (int i = 0; i < shape_nodes.size(); i++)
    shape_input.push_back(shape_nodes[i]->vector());

  toc("setup");
  tictoc_print();

  MatrixXd turned_norm(data_len, 2);
  MatrixXd vicon_norm(data_len, 2);
  EllipsoidApproxLimitSurface eals(muc, ls_c);
  GPMotion gpm(gpmfile);

  bool last_in_contact = false, now_in_contact = false;
  for (int t = 0; t < data_len; t++) {
    cout << "t=" << t << endl;
    bool factor_added = false, contact_factor_added = false, motion_factor_added = false, poseprior_factor_added = false;
    Vector2d contact_point = d.block(0, t, 2, 1);
    Vector2d contact_normal = d.block(3, t, 2, 1);
    Vector2d probe_center_now = d.block(13, t, 2, 1);
    last_in_contact = now_in_contact;
    now_in_contact = fabs(contact_normal[0])+fabs(contact_normal[1]) > 0.1;  // if not contact, contact_normal == [0,0]
    bool has_apriltag = d(25,t) > 0.1;
    bool add_pose_prior_factor_t;
    cout << "has_apriltag:" << has_apriltag << endl;
    if(last_in_contact && now_in_contact && rand() > cam_keep_rate * RAND_MAX)
      add_pose_prior_factor_t = false;
    else
      add_pose_prior_factor_t = add_pose_prior_factor;

    if(!has_apriltag && camera_input_source == "apriltag") add_pose_prior_factor_t = false;

    // 3.1 add node of pose if t >= 0
    Pose2d_Node *new_pose_node;
    if (t > 0) {
      new_pose_node = new Pose2d_Node();
      slam.add_node(new_pose_node);
      pose_nodes.push_back(new_pose_node);
    }


    // 3.2. add factor from pose prior from t > 0
    Pose2d_Factor *pose_prior_factor = NULL;
    if (t > 0) {
      // still init to something
      if (cont_init_pose == "last_estimate")
        new_pose_node->init(pose_nodes[t - 1]->value());
      else if (cont_init_pose == "groundtruth") {
        Vector3d groundth_x_now(d(6, t), d(7, t),
                                quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t))));
        new_pose_node->init(groundth_x_now);
      }
      else if (cont_init_pose == "groundtruth_w_noise") {
        Vector3d groundth_x_now(d(6, t)+randn(0, t), d(7, t)+randn(1, t),
                                quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t)))+randn(2,t));
        new_pose_node->init(groundth_x_now);
      }
      else if (cont_init_pose == "apriltag") {
        if (has_apriltag) {
          Vector3d groundth_x_now(d(17, t), d(18, t), d(24, t));
          new_pose_node->init(groundth_x_now);
        } else {
          new_pose_node->init(pose_nodes[t - 1]->value());
        }
      }


      if(add_pose_prior_factor_t) {
        if(camera_input_source == "groundtruth") {
          Vector3d camera_input(d(6, t), d(7, t),
                                quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t))));
          Pose2d pose_prior_now = Pose2d(camera_input);
          pose_prior_factor = new Pose2d_Factor(pose_nodes[t], pose_prior_now, noise3_pose);
        }
        else if(camera_input_source == "groundtruth_w_noise") {
          Vector3d camera_input(d(6, t) + randn(0, t), d(7, t) + randn(1, t),
                                quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t))) + randn(2, t));
          Pose2d pose_prior_now = Pose2d(camera_input);
          pose_prior_factor = new Pose2d_Factor(pose_nodes[t], pose_prior_now, noise3_pose);
        }
        else if(camera_input_source == "apriltag") {
          Vector3d camera_input(d(17, t), d(18, t), d(24, t));
          Pose2d pose_prior_now = Pose2d(camera_input);
          pose_prior_factor = new Pose2d_Factor(pose_nodes[t], pose_prior_now, noise3_pose_apriltag);
        }
        slam.add_factor(pose_prior_factor);
        poseprior_factor_added = true;
        factor_added = true;
      }

    }

    // 4.1 add data for measurement from t = 0

    if (isreal) {
      const double theta = atan(muc);
      //cout << normvec << endl;
      //pk();
      if (turn) {
        rotate_back_frame2d(
                contact_normal, Vector3d(0, 0, theta),
                &contact_normal, NULL);
      }
      turned_norm.row(t) = contact_normal.transpose();

      // compute vicon norm hack (for rect1 only)
      Vector2d pt_object;
      MatrixXd M;
      Vector2d viconnormvec_obj, viconnormvec;
      Vector3d groundth_x_now(d(6, t), d(7, t), quat_to_yaw(Quaterniond(d(9, t), d(10, t), d(11, t), d(12, t))));

      transform_to_frame2d(probe_center_now,
                           groundth_x_now, &pt_object, NULL);

      M.resize(2, 4);
      M << 0.045, -0.045, -0.045, 0.045,
              0.045, 0.045, -0.045, -0.045;

      shape__probe_obj__find_norm(pt_object, M, &viconnormvec_obj);

      rotate_back_frame2d(viconnormvec_obj,
                          groundth_x_now, &viconnormvec, NULL);
      vicon_norm.row(t) = viconnormvec.transpose();

      if (fromVicon) {
        contact_normal = viconnormvec;
      }

    }

    ContactMeasurement_Factor *measurement_factor = NULL;
    if (t >= 0 && now_in_contact  && add_contact_factor) {

      ContactMeasurement *contact_measurement =
              new ContactMeasurement(
                      Point2d(contact_point),  //contact_point
                      Point2d(contact_normal),  //contact_normal
                      Point2d(probe_center_now), //probe_center
                      probe_radius);
      contact_data.push_back(contact_measurement);
      // 4.2 add factor from measurement
      measurement_factor =
              new ContactMeasurement_Factor(
                      &shape_nodes,
                      pose_nodes[t],
                      *contact_measurement,
                      noise3_measure);
      measurement_factor->id = t;
      slam.add_factor(measurement_factor);
      factor_added = true;
      contact_factor_added = true;
    }


    // 5. add factor from motion from t > 0
    PushMeasurement_Factor *push_factor = NULL;
    if (t > 0 && last_in_contact && now_in_contact && add_motion_factor) {

      Vector2d probe_center_last = d.block(13, t - 1, 2, 1);
      PushMeasurement *push_measurement =
              new PushMeasurement(contact_point, contact_normal,
                                  probe_center_last, probe_center_now);

      push_factor =
              new PushMeasurement_Factor(pose_nodes[t - 1], pose_nodes[t],
                                         *push_measurement, noise3_motion, &eals, &gpm,
                                         motion_model_option);
      push_factor->id = t;
      slam.add_factor(push_factor);
      factor_added = true;
      motion_factor_added = true;

    }


    // 6. add factor if no factors added t > 0
    Pose2d_Pose2d_Factor *stationary_prior = NULL;
    if (t > 0 && !add_pose_prior_factor_t) {
      cout << "stationary_prior added" << endl;
      stationary_prior = new Pose2d_Pose2d_Factor(pose_nodes[t-1], pose_nodes[t], Pose2d(), noise3_pose_rel);
      slam.add_factor(stationary_prior);
      cout << "stationary_prior basic_error" << stationary_prior->basic_error().transpose() << endl;
      cout << "stationary_prior error" << stationary_prior->error().transpose() << endl;
    }


    //if(to_calculate_var){
      if(contact_factor_added) {
        vc_norm.push_back(measurement_factor->basic_error());
        cout << "measurement_factor->basic_error" << measurement_factor->basic_error().transpose() << endl;
        cout << "measurement_factor->error" << measurement_factor->error().transpose() << endl;
      }
      if(motion_factor_added) {
        vc_push.push_back(push_factor->basic_error());
        cout << "push_factor->basic_error" << push_factor->basic_error().transpose() << endl;
        cout << "push_factor->error" << push_factor->error().transpose() << endl;
      }
      if(poseprior_factor_added) {
        vc_pose.push_back(pose_prior_factor->basic_error());
        cout << "pose_prior_factor basic_error" << pose_prior_factor->basic_error().transpose() << endl;
      }
    //}

    // 6. do slam update

    if (t >= 0) {
      pose_input.push_back(pose_nodes[t]->vector());
      if (inc) {
        screen_displayer.print_before(t);
        visualizer.visualize();
        tic("incremental");
        //slam.print_graph();
        slam.update();
        double _inc_time = toc("incremental");
        inc_time.push_back(_inc_time);
        screen_displayer.print_after(t);

        if(measurement_factor) {
          cout << "measurement_factor->basic_error" << measurement_factor->basic_error().transpose() << endl;
          cout << "measurement_factor->error" << measurement_factor->error().transpose() << endl;
        }
        if(push_factor) {
          cout << "push_factor->basic_error" << push_factor->basic_error().transpose() << endl;
          cout << "push_factor->error" << push_factor->error().transpose() << endl;
        }
        if(stationary_prior){
          cout << "stationary_prior basic_error" << stationary_prior->basic_error().transpose() << endl;
          cout << "stationary_prior error" << stationary_prior->error().transpose() << endl;
        }
        visualizer.visualize();

        tic("incremental_cov");
        // compute variance
        Covariances::node_lists_t node_lists;
        std::list<Node*> nodes;
        nodes.push_back(pose_nodes[t]);
        node_lists.push_back(nodes);
        std::list<MatrixXd> cov_blocks = covariances.marginal(node_lists);
        int i =1;
        for (list<MatrixXd>::iterator it = cov_blocks.begin(); it!=cov_blocks.end(); it++, i++) {
          cout << "\nblock " << i << endl;
          cout << *it << endl;
          pose_inc_cov.push_back(*it);
        }
        double _inc_cov_time = toc("incremental_cov");
      }

      // record it
      vector<VectorXd> shape;
      for (int i = 0; i < shape_nodes.size(); i++)
        shape.push_back(shape_nodes[i]->vector());
      shape_inc.push_back(shape);
      pose_inc.push_back(pose_nodes[t]->vector());
    }
  }

  if(to_calculate_var){
    cout << "vc_push info\n" << vc_push.calculateInfo() << endl;
    cout << "vc_push mean\n" << vc_push.calculateMean() << endl;

    cout << "vc_norm info\n" << vc_norm.calculateInfo() << endl;
    cout << "vc_norm mean\n" << vc_norm.calculateMean() << endl;

    cout << "vc_pose info\n" << vc_pose.calculateInfo() << endl;
    cout << "vc_pose mean\n" << vc_pose.calculateMean() << endl;
  }

  cout << endl;
  tictoc_print();
  cout << endl;
  if(to_print_graph)
    slam.print_graph();

  slam.print_stats();
  // optimize the graph
  if (batchopt) {
    tic("batch");
    Properties p = slam.properties();
    p.method = LEVENBERG_MARQUARDT;
    p.max_iterations = 150;
    p.quiet = false;
    p.verbose = true;
    slam.set_properties(p);
    slam.batch_optimization();
    slam.print_stats();
    toc("batch");
  }
  // save to file
  save_to_json(shape_nodes, pose_nodes, shape_inc, pose_inc, pose_inc_cov, shape_input, pose_input, inc_time,
               d, probe_radius, label,
               startdate, outfilename, shape_id, offset, turned_norm, vicon_norm);
  tictoc_print();
  ofstream outsummaryfile(outsummaryfilename);
  tictoc_print_to_stream(outsummaryfile);

  string cmd30 = string("/home/mcube/push/software/isamShapePose/script/displayResult.py ") +
                 string(outfilename) + string(" 30");
  cout << cmd30 << endl;
  string cmd1 = string("/home/mcube/push/software/isamShapePose/script/displayResult.py ") +
                 string(outfilename) + string(" 1");
  cout << cmd1 << endl;
  if (render_result) {
    int ret;
    //ret = system(cmd30.c_str());
    ret = system(cmd1.c_str());
  }
}

