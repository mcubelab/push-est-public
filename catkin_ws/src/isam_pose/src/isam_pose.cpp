/**
 * @file isam_pose.cpp
 * @brief The entry point for tactile pose estimation.
 * @author Peter KT Yu
 *
 */

#include "contact_measurement.h"
#include "push_measurement.h"
#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltags/AprilTagDetections.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "RosMsgBuffer.h"
#include "sp_util.h"
#include <isam_pose_param.h>
#include <isam_pose_logger.h>
#include <Feeder.h>
#include <EKF.h>

using namespace std;
using namespace isam;
using namespace Eigen;

Isam_param * isam_param;

void updateApriltag(RosMsgBuffer<apriltags::AprilTagDetections>& rmbApriltag,
                    tf::TransformListener& lr, VectorXd& object_apriltag__world, bool& updated){
  apriltags::AprilTagDetections apriltag_detections_msg;
  updated = rmbApriltag.getmsg(&apriltag_detections_msg);
  if(!updated)
    return;

  for(int i=0; i<apriltag_detections_msg.detections.size(); i++){
    apriltags::AprilTagDetection& det = apriltag_detections_msg.detections[i];
    if(det.id == isam_param->tag_id){
      VectorXd objpose__cam = transform_back(isam_param->tag_obj, rospose_to_posevec(det.pose));
      object_apriltag__world = poseTransform(objpose__cam, "/observer_rgb_optical_frame", "/map", lr);
      // hack fix objpose__cam
      double z = 0.01158 + 0.013/2.0 + 0.02;
      VectorXd q1_world=VectorXd3(0.,-1.,z), q2_world=VectorXd3(0.,1.,z), q3_world=VectorXd3(1.,0,z);
      VectorXd p1_world = object_apriltag__world.head(3);
      VectorXd p2_world = lookupTransform("/observer_rgb_optical_frame","/map", lr).head(3);  // camera position

      VectorXd normal = VectorXd3(0,0,1);
      double a = (-normal.transpose() * p2_world + normal.transpose() * q3_world)(0,0);
      double b = (normal.transpose() * (p1_world-p2_world))(0,0);
      double t = a / b;
      VectorXd p3 = (p1_world - p2_world)*t + p2_world;
      //cout << "p1_world "<< p1_world << endl;
      //cout << "p2_world "<< p2_world << endl;
      //cout << "t "<< t << endl;
      //cout << "object_apriltag__world"<<endl<< object_apriltag__world.transpose() << endl << " p3" <<endl<< p3.transpose() << endl;

      object_apriltag__world(0) = p3(0);
      object_apriltag__world(1) = p3(1);
      object_apriltag__world(2) = p3(2);
      //// end of fixing

      updated = true;

      if(!quat_is_good(object_apriltag__world.tail(4)))
        updated = false;
      object_apriltag__world = VectorXd3(object_apriltag__world[0], object_apriltag__world[1], quat_to_yaw(object_apriltag__world.tail(4)));
      return;
    }
  }
  updated = false;  // desired Apriltag not found
}

void updateWrench(RosMsgBuffer<geometry_msgs::WrenchStamped>& rmbFT,
                  tf::TransformListener& lr, VectorXd& force__world, bool& updated, string ft_frame_id){
  geometry_msgs::WrenchStamped ws;
  bool received = rmbFT.getmsg(&ws);
  if(!received)
    return;
  geometry_msgs::Wrench wrench__world;
  double threshold = 2.5;
  if(ft_frame_id == "link_ft_left")
    threshold *= 2;
  //ws.wrench.force.z = 0;  // ignore vertical force
  double tx = ws.wrench.force.x;  // ignore vertical force
  double ty = ws.wrench.force.y;  // ignore vertical force
  double th = 0.0;
  if(ft_frame_id == "link_ft_left")
    th = 20.0 * 3.14 / 180;
  else
    th = 4.0 * 3.14 / 180;
  ws.wrench.force.x = tx * cos(th) - ty * sin(th);
  ws.wrench.force.y = tx * sin(th) + ty * cos(th);
  cout << ft_frame_id << " " << norm(ws.wrench.force) << endl;
  if(norm(ws.wrench.force) < threshold) {
    return;
  }
  updated = true;

  geometry_msgs::PointStamped f_ft, f_world;
  f_ft.header.frame_id = ft_frame_id;
  f_ft.point.x = ws.wrench.force.x; f_ft.point.y = ws.wrench.force.y; f_ft.point.z = ws.wrench.force.z;
  VectorXd ft_frame = lookupTransform("map", ft_frame_id, lr);
  ft_frame[0] = 0; ft_frame[1] = 0; ft_frame[2] = 0;
  VectorXd Mf_local = VectorXd7(ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z,0,0,0, 1.0);
  VectorXd Mf_global = transform_back(Mf_local, ft_frame);

  force__world = VectorXd2(Mf_global[0], Mf_global[1]);
}

void updateContactPoint(const VectorXd pusher__world, const VectorXd& force__world, const bool updated_ft,
                        VectorXd& contact_point__world,
                        VectorXd& contact_normal__world, bool& updated_contact_point, Isam_param* isam_param){
  if(!updated_ft) return;

  updated_contact_point = true;
  VectorXd tmp = force__world;

  tmp *=  1.0 / force__world.norm() * (-1.0);  // need to revert from pointing
  // to FT to pointing to object
  contact_normal__world = tmp;                                    // to FT to pointing to object

  tmp *= isam_param->probe_radius;

  contact_point__world = pusher__world + tmp;
}

void read_from_rosmsg(tf::TransformListener& lr,
                      RosMsgBuffer<apriltags::AprilTagDetections>* rmbApriltag,
                      RosMsgBuffer<geometry_msgs::WrenchStamped>* rmbWS[],
                      VectorXd& object_apriltag__world,
                      vecVec& pusher2d__world,
                      vecVec& contact_force__world, vecVec& contact_point__world,
                      vecVec& contact_normal__world,
                      VectorXd& Vector_object_vicon__world,
                      bool& updated_apriltag, bool updated_contact_point[], bool updated_ft[]) {

  // 1.1 get Apriltag in world frame
  updateApriltag(*rmbApriltag, lr, object_apriltag__world, updated_apriltag);

  // 1.2 get 2d pusher centers in world frame
  vecVec pusher__world(2); // 3d posevec
  pusher__world[LEFT] = lookupTransform("link_probe_left", "map", lr);
  pusher__world[RIGHT] = lookupTransform("link_probe_right", "map", lr);
  pusher2d__world[LEFT] = pusher__world[LEFT].head(2);
  pusher2d__world[RIGHT] = pusher__world[RIGHT].head(2);

  // 1.3 get contact forces in world frame
  contact_force__world[LEFT] = VectorXd2(0,0);
  contact_force__world[RIGHT] = VectorXd2(0,0);
  updateWrench(*rmbWS[LEFT], lr, contact_force__world[LEFT], updated_ft[LEFT], "link_ft_left");
  updateWrench(*rmbWS[RIGHT], lr, contact_force__world[RIGHT], updated_ft[RIGHT], "link_ft_right");

  // 1.4 get 2d contact locations in world frame
  for(int side=0; side<2; side++) {
    contact_point__world[side] = VectorXd2(0,0);
    contact_normal__world[side] = VectorXd2(0,0);
    updateContactPoint(pusher2d__world[side], contact_force__world[side], updated_ft[side], contact_point__world[side],
                       contact_normal__world[side], updated_contact_point[side], isam_param);
  }

  // 1.5 get vicon groundtruth pose
  VectorXd object_vicon__world = lookupTransform("vicon/StainlessSteel/StainlessSteel", "map", lr);
  Vector_object_vicon__world << object_vicon__world.x(), object_vicon__world.y(), quat_to_yaw(object_vicon__world.tail(4));

}

void init_shape_data(vector<Point2d *>& shape_data, Isam_param* isam_param){
  int nM = isam_param->nM;
  string shape_id =  isam_param->shape_id;
  for (int i = 0; i < nM; i++) {
    Point2d *new_point = NULL;
    if (shape_id == "circle") {
      const double rad = (i + 0.5) / nM * M_PI * 2;
      new_point = new Point2d(cos(rad) * 0.0525,
                              sin(rad) * 0.0525);
    }
    else if (shape_id == "ellip2") {
      const double rad = (i + 0.5) / nM * M_PI * 2;
      new_point = new Point2d(cos(rad) * 0.0525,
                              sin(rad) * 0.0654);
    }
    else if (shape_id == "rect1") {
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
    } else if (shape_id == "butter") {
      int ii = i*isam_param->sub_butter_len;
      new_point = new Point2d(isam_param->butter_shape[ii][0], isam_param->butter_shape[ii][1]);
    }
    shape_data.push_back(new_point);
  }
}

class ROSAgent{
public:
  ros::NodeHandle nh;
  ros::Publisher _pub;
  RosMsgBuffer<apriltags::AprilTagDetections> *rmbApriltag;
  RosMsgBuffer<geometry_msgs::WrenchStamped> *rmbWS[2];

  ROSAgent(int argc, char* argv[]){
    nh = ros::NodeHandle();
    _pub = nh.advertise<std_msgs::String>("pose_est", 1);

    rmbApriltag = new RosMsgBuffer<apriltags::AprilTagDetections>(std::string("/apriltags/detections"), nh);
    rmbWS[0] = new RosMsgBuffer<geometry_msgs::WrenchStamped>(std::string("/ft_left/netft_data"), nh);
    rmbWS[1] = new RosMsgBuffer<geometry_msgs::WrenchStamped>(std::string("/ft_right/netft_data"), nh);
  }

};

MatrixXd sqrtinf_2_cov(MatrixXd sqrtinf){
  return (sqrtinf.transpose() * sqrtinf).inverse();
}

VectorXd vcat(VectorXd a, VectorXd b){
  VectorXd c(a.rows()+b.rows());
  c << a, b;
  return c;
}

MatrixXd vcat(MatrixXd a, MatrixXd b){
  MatrixXd c(a.rows()+b.rows(), b.cols());
  c.topLeftCorner(a.rows(), a.cols()) = a;
  c.bottomLeftCorner(b.rows(), b.cols()) = b;
  return c;
}

MatrixXd cov_cat(MatrixXd a, MatrixXd b){
  MatrixXd c(a.rows() + b.rows(), a.cols() + b.cols());
  //c << a, MatrixXd::Zero(a.rows(), b.cols()), MatrixXd::Zero(b.rows(), a.cols()), b;
  c.topLeftCorner(a.rows(), a.rows()) = a;
  c.topRightCorner(a.rows(), b.cols()).setZero();
  c.bottomLeftCorner(b.rows(), a.cols()).setZero();
  c.bottomRightCorner(b.rows(), b.cols()) = b;
  return c;
}

int main(int argc, char* argv[]) {
  isam_param = new Isam_param(argc, argv);
  ros::init(argc, argv, "isam_pose");
  ROSAgent rosagent(argc, argv);

  tf::TransformListener lr;
  tf::TransformBroadcaster br;
  ros::Duration(1).sleep();

  ros::Rate r(80); // 80 hz

  VarianceCalculator vc_contact, vc_push, vc_apriltag, vc_static;  // for identify variance

  // 0. initialize slam object
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

  Logger logger(&shape_nodes, &pose_nodes);
  Feeder * jsonFeeder = NULL;
  if(isam_param->use_json_input)
    jsonFeeder = new Feeder(isam_param->json_input_path);

  vector<Point2d *> shape_data;
  vector<ContactMeasurement *> contact_data;
  vector<PushWForceMeasurement *> push_data;

  const Covariances& covariances = slam.covariances();

  // 0.1 add shape nodes
  for (int i = 0; i < isam_param->nM; i++) {
    Point2d_Node *new_point_node = new Point2d_Node();
    shape_nodes.push_back(new_point_node);
  }

  // 0.1.2 add data for shape prior
  init_shape_data(shape_data, isam_param);

  // 0.1.3 init factor by shape prior
  for (int i = 0; i < isam_param->nM; i++)
    shape_nodes[i]->init(*shape_data[i]);

  // 0.1.3 record shape prior to logger
  for (int i = 0; i < shape_nodes.size(); i++)
    logger.shape_input.push_back(shape_nodes[i]->vector());

  int t = 0;
  cost_func_t cam_cost_func = &robust_cam_cost_function;

  // 0.2 EKF_state
  VectorXd EKF_state;
  MatrixXd EKF_state_cov;
  MatrixXd EKF_H;
  MatrixXd EKF_Q;
  VectorXd EKF_h;
  Pose2d_Node *EKF_old_pose_node = NULL;
  Pose2d_Node *EKF_new_pose_node = NULL;

  // 0.3 The process loop
  VectorXd object_apriltag__world(3);
  while (rosagent.nh.ok() && t<isam_param->maxstep) {
    ros::spinOnce();

    // 1. get data
    // 1.0 define the variables

    vecVec pusher2d__world(2);
    vecVec contact_point__world(2), contact_normal__world(2), contact_force__world(2);

    VectorXd Vector_object_vicon__world(3);

    bool updated_apriltag = false;
    bool updated_contact_point[2] = {false, false}, updated_ft[2] = {false, false};
    if(isam_param->use_json_input) {
      bool flag = jsonFeeder->getValues(t, object_apriltag__world, pusher2d__world,
                           contact_force__world, contact_point__world, contact_normal__world,
                           Vector_object_vicon__world, updated_apriltag, updated_contact_point, updated_ft);
      if(!flag) break;
    }
    else{
      try {
        read_from_rosmsg(lr, rosagent.rmbApriltag, rosagent.rmbWS,
                         object_apriltag__world, pusher2d__world,
                         contact_force__world, contact_point__world, contact_normal__world,
                         Vector_object_vicon__world, updated_apriltag, updated_contact_point, updated_ft);
        //cout << updated_apriltag << " " << object_apriltag__world << endl;
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        break;
      }
    }

    // 1.6 record all to logger
    for(int side=0; side<2; side++) {
      logger.contact_point__world[side].push_back(contact_point__world[side]);
      logger.contact_normal__world[side].push_back(contact_normal__world[side]);
      logger.contact_force__world[side].push_back(contact_force__world[side]);
      logger.pusher2d__world[side].push_back(pusher2d__world[side]);
      logger.has_contact[side].push_back(updated_ft[side]);
    }
    logger.has_apriltag.push_back(updated_apriltag);
    logger.pose_true.push_back(Vector_object_vicon__world);

    // 1.7 Add pose node
    Pose2d_Node *new_pose_node = new Pose2d_Node();
    slam.add_node(new_pose_node);
    pose_nodes.push_back(new_pose_node);
    if(isam_param->to_calculate_var){
      new_pose_node->init(Pose2d(Vector_object_vicon__world));
    }

    // 1.8 EKF fake node
    EKF_old_pose_node = new Pose2d_Node();
    EKF_new_pose_node = new Pose2d_Node();

    // 2. Add cost functions

    // 2.1.1 add stationary prior regardless having apriltag or not
    if(t == 0) {
      Pose2d origin(isam_param->offset[0], isam_param->offset[1], 0.);
      Pose2d_Factor *pose_prior_factor = new Pose2d_Factor(pose_nodes[0], origin, isam_param->noise3_pose);
      if(isam_param->add_stationary_factor) {
        slam.add_factor(pose_prior_factor);

        // EKF init
        if(isam_param->doEKF) {
          //EKF_old_pose_node->init(Pose2d(EKF_state));
          EKF_state = VectorXd3(isam_param->offset[0], isam_param->offset[1], 0.);
          EKF_state_cov = sqrtinf_2_cov(isam_param->noise3_pose.sqrtinf());
          EKF_new_pose_node->init(Pose2d(EKF_state));
        }
      }
    }
    else { // t>0
      Pose2d_Pose2d_Factor *stationary_prior = NULL;
      //cout << "stationary_prior added" << endl;
      stationary_prior = new Pose2d_Pose2d_Factor(pose_nodes[t - 1], pose_nodes[t], Pose2d(0,0,0),
                                                  isam_param->noise3_pose_rel);
      if (isam_param->add_stationary_factor) {
        slam.add_factor(stationary_prior);
        vc_static.push_back(stationary_prior->basic_error(ESTIMATE));

        // EKF predict
        if(isam_param->doEKF) {
          EKF_old_pose_node->init(Pose2d(EKF_state));
          // EKF_state = EKF_state; // assume at the same place
          EKF_state_cov = EKF_state_cov + sqrtinf_2_cov(isam_param->noise3_pose_rel.sqrtinf());
          EKF_new_pose_node->init(Pose2d(EKF_state));
        }
      }
    }

    // EKF
    EKF_H = MatrixXd();
    EKF_Q = MatrixXd();
    EKF_h = VectorXd();

    // logging
    if(!updated_apriltag || !isam_param->add_apriltag_factor) {
      if(isam_param->add_stationary_factor)
        logger.pose_input.push_back(object_apriltag__world);
        //logger.pose_input.push_back(pose_nodes[t]->vector());
      else
        logger.pose_input.push_back(VectorXd3(0,0,0));
    }

    // 2.1 Add pose prior
    //     (apriltag or last step (if no touch then strongly at original place, if touch then weakly))
    if(updated_apriltag){
      //cout << "camera_prior added" << endl;
      Vector3d camera_input = object_apriltag__world;
      Pose2d_Factor* pose_prior_factor = new Pose2d_Factor(pose_nodes[t], camera_input, isam_param->noise3_pose_apriltag);

      pose_prior_factor->set_cost_function(&cam_cost_func);
      if(isam_param->add_apriltag_factor) {
        //cout << t << " " << isam_param->add_apriltag_factor << endl;
        slam.add_factor(pose_prior_factor);
        vc_apriltag.push_back(pose_prior_factor->basic_error(ESTIMATE));
        logger.pose_input.push_back(camera_input);

        // EKF making measurement matrix

        if(isam_param->doEKF) {
          Pose2d_Factor* pose_prior_factor_ekf = new Pose2d_Factor(EKF_new_pose_node, camera_input, isam_param->noise3_pose_apriltag);

//          EKF_h = vcat(EKF_h, pose_prior_factor->basic_error(ESTIMATE));
//          EKF_Q = cov_cat(EKF_Q, sqrtinf_2_cov(isam_param->noise3_pose_apriltag.sqrtinf()));
//          EKF_H = vcat(EKF_H, pose_prior_factor->jacobian().terms().front().term());

          EKF_h = vcat(EKF_h, pose_prior_factor_ekf->error(ESTIMATE));
          EKF_Q = cov_cat(EKF_Q, MatrixXd::Identity(3,3));
          EKF_H = vcat(EKF_H, pose_prior_factor_ekf->jacobian().terms().front().term());
          delete pose_prior_factor_ekf;
        }
      }

      // for visualization
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(object_apriltag__world[0], object_apriltag__world[1], 0.042) );
      transform.setRotation( tf::Quaternion(0, 0,  object_apriltag__world[2]) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "apriltag"));
    }

    std_msgs::String ss;
    ss.data = updated_apriltag ? "has_tag" : "no_tag";
    rosagent._pub.publish(ss);

    // 2.2 Add measurement costs (Left and Right)
    for(int side = 0 ; side < 2; side ++) {
      if (updated_contact_point[side]) {
        ContactMeasurement_Factor *measurement_factor = NULL;
        ContactMeasurement *contact_measurement = new ContactMeasurement(
                Point2d(contact_point__world[side]), Point2d(pusher2d__world[side]), isam_param->probe_radius);
        //  add factor from measurement
        measurement_factor = new ContactMeasurement_Factor(
                        &shape_nodes, pose_nodes[t], *contact_measurement, isam_param->noise3_measure);
        measurement_factor->id = t;
        if(isam_param->add_contact_factor){
          slam.add_factor(measurement_factor);
          vc_contact.push_back(measurement_factor->basic_error(ESTIMATE));

          // EKF making measurement matrix
          if(isam_param->doEKF) {
            ContactMeasurement_Factor* measurement_factor_ekf = new ContactMeasurement_Factor(
                &shape_nodes, EKF_new_pose_node, *contact_measurement, isam_param->noise3_measure);
//            EKF_h = vcat(EKF_h, measurement_factor->basic_error(ESTIMATE));
//            EKF_Q = cov_cat(EKF_Q, sqrtinf_2_cov(isam_param->noise3_measure.sqrtinf()));
//            EKF_H = vcat(EKF_H, measurement_factor->jacobian().terms().front().term());
            EKF_h = vcat(EKF_h, measurement_factor_ekf->error(ESTIMATE));
            EKF_Q = cov_cat(EKF_Q, MatrixXd::Identity(3,3));
            EKF_H = vcat(EKF_H, measurement_factor_ekf->jacobian().terms().front().term());
            delete measurement_factor_ekf;
          }
        }
      }
    }

    // 2.3 Add motion costs
    if(t>0 && (logger.has_contact[LEFT][t-1] || logger.has_contact[RIGHT][t-1])) {
      PushWForceMeasurement_Factor *motion_factor = NULL;
      PushWForceMeasurement *push_measurement = new PushWForceMeasurement();
      for (int side = 0; side < 2; side++) {
        if(logger.has_contact[side][t-1]) {
          push_measurement->_contact_point.push_back(logger.contact_point__world[side][t-1]);
          push_measurement->_contact_force.push_back(logger.contact_force__world[side][t-1]);
          push_measurement->_contact_normal.push_back(logger.contact_normal__world[side][t-1]);
        }
      }
      motion_factor = new PushWForceMeasurement_Factor(pose_nodes[t-1], pose_nodes[t], *push_measurement,
                                                       isam_param->noise2_motion);
      /*  for plotting the field
      VectorXd center = pose_nodes[t]->vector();
      VectorXd last = pose_nodes[t-1]->vector();
      double start_point = -0.1;
      double width = 0.2;
      for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
          VectorXd newcenter = VectorXd3(center[0], center[1]+start_point + (i/100.0) * 0.2, center[2]+start_point + (j/100.0) * 0.2);
          pose_nodes[t]->init(Pose2d(newcenter));
          VectorXd error = motion_factor->basic_error();
          printf("[%lf, %lf, %lf, %lf, %lf],\n", newcenter[0]-last[0], newcenter[1]-last[1], newcenter[2]-last[2], error[0], error[1]);
        }
      }
      pose_nodes[t]->init(Pose2d(center));*/

      if(isam_param->add_push_factor) {
        slam.add_factor(motion_factor);
        vc_push.push_back(motion_factor->basic_error(ESTIMATE));
        //cout << "motion_factor->basic_error orig" << motion_factor->basic_error(ESTIMATE) << endl;
        //cout << "motion_factor->jacobian orig" << motion_factor->jacobian() << endl;

        // EKF making measurement matrix
        if(isam_param->doEKF) {
          PushWForceMeasurement_Factor* motion_factor_ekf = new PushWForceMeasurement_Factor(EKF_old_pose_node, EKF_new_pose_node, *push_measurement,
                                                           isam_param->noise2_motion);
          //cout << "motion_factor->basic_error ekf" << motion_factor_ekf->basic_error(ESTIMATE) << endl;
          //cout << "motion_factor->jacobian ekf" << motion_factor_ekf->jacobian() << endl;
          //EKF_h = vcat(EKF_h, motion_factor->basic_error(ESTIMATE));
          //EKF_Q = cov_cat(EKF_Q, sqrtinf_2_cov(isam_param->noise2_motion.sqrtinf()));
          //EKF_H = vcat(EKF_H, motion_factor->jacobian().terms().back().term());
          EKF_h = vcat(EKF_h, motion_factor_ekf->error(ESTIMATE));
          EKF_Q = cov_cat(EKF_Q, MatrixXd::Identity(2,2));
          EKF_H = vcat(EKF_H, motion_factor_ekf->jacobian().terms().back().term());
//          cin.get();
          delete motion_factor_ekf;
        }
      }
    }

    // 3. optimize
    tic("incremental");
    //slam.print_graph();
    int n_nodes_to_keep = 200;
    if(isam_param->to_inc){
      if(t >= n_nodes_to_keep +100 && t%100 == 0) {
        for(int i=0; i<100; i++)
          slam.remove_node(pose_nodes[t - n_nodes_to_keep -i-1]);

        Pose2d origin(pose_nodes[t - n_nodes_to_keep]->vector());
        Pose2d_Factor *pose_prior_factor = new Pose2d_Factor(pose_nodes[t - n_nodes_to_keep], origin, isam_param->noise3_pose);
        if(isam_param->add_stationary_factor)
          slam.add_factor(pose_prior_factor);
      }
      slam.update();
    }
    logger.inc_time.push_back(toc("incremental"));

    // 4.1 record the pose estimation
    logger.pose_inc.push_back(pose_nodes[t]->vector());

    // 4.2 compute and save covariance
    Covariances::node_lists_t node_lists;
    std::list<Node*> nodes;
    nodes.push_back(pose_nodes[t]);
    node_lists.push_back(nodes);
    std::list<MatrixXd> cov_blocks = covariances.marginal(node_lists);
    for (list<MatrixXd>::iterator it = cov_blocks.begin(); it!=cov_blocks.end(); it++) {
      logger.pose_inc_cov.push_back(*it);
    }

    // publish the result
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose_nodes[t]->value().x(), pose_nodes[t]->value().y(), 0.042) );
    transform.setRotation( tf::Quaternion(0, 0,  pose_nodes[t]->value().t()) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "estimate"));
    // todo: _pub.publish(Pose2dWithCovariance);

    // 5. Do EKF for comparison purpose
    // need to come up with h and H and Q
    if(isam_param->doEKF){
      if(EKF_h.rows() > 0) { // measurement available

        tic("EKF");
        EKFupdate(EKF_state, EKF_state_cov, EKF_H, EKF_Q, EKF_h, EKF_state, EKF_state_cov);
        //cout << "EKF toc" << toc("EKF") << endl;
      }

      tf::Transform transform;
      transform.setOrigin( tf::Vector3(EKF_state(0), EKF_state(1), 0.042) );
      transform.setRotation( tf::Quaternion(0, 0,  EKF_state(2)) );
      //cout << "EKF_state" << endl << EKF_state << endl;
      //cout << "EKF_state_cov" << endl << EKF_state_cov << endl;
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "estimate_EKF"));
      //if(EKF_old_pose_node)
      //  delete EKF_old_pose_node;

      logger.pose_ekf.push_back(EKF_state);
      logger.pose_ekf_cov.push_back(EKF_state_cov);
    }

    if(!isam_param->use_json_input) {
      r.sleep();
    }
    if (isam_param->use_json_input && isam_param->pause){
      cin.get();
    }
    t++;
    //if(t>100)
    //  break;
  }


  if(isam_param->to_calculate_var){

    cout << "vc_static info\n" << vc_static.calculateInfo() << endl;
    cout << "vc_static mean\n" << vc_static.calculateMean() << endl;
    vc_static.save(isam_param->vcoutfilename_prefix + "vc_static.json");

    cout << "vc_apriltag info\n" << vc_apriltag.calculateInfo() << endl;
    cout << "vc_apriltag mean\n" << vc_apriltag.calculateMean() << endl;
    vc_apriltag.save(isam_param->vcoutfilename_prefix + "vc_apriltag.json");

    cout << "vc_contact info\n" << vc_contact.calculateInfo() << endl;
    cout << "vc_contact mean\n" << vc_contact.calculateMean() << endl;
    vc_contact.save(isam_param->vcoutfilename_prefix + "vc_contact.json");

    cout << "vc_push info\n" << vc_push.calculateInfo() << endl;
    cout << "vc_push mean\n" << vc_push.calculateMean() << endl;
    vc_push.save(isam_param->vcoutfilename_prefix + "vc_push.json");
  }

  // save to file
  cout << "done" << endl;
  if(t>10 && isam_param->to_inc) {
    logger.save_to_json(isam_param);
    logger.render_result(isam_param);
  }
  return 0;
}
