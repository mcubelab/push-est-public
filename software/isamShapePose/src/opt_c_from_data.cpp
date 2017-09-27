//
// Created by peterkty on 1/13/17.
//

#include <isam/isam.h>
#include "sp_util.h"
#include "Point1d.h"
#include "push_measurement_c.h"
#include "EllipsoidApproxLimitSurface.h"
#include "p_poly_dist.h"

using namespace std;
using namespace isam;
using namespace Eigen;


typedef NodeT<Point1d> Point1d_Node;

int main(int argc, char* argv[]){

  const string data_path = std::getenv("DATA_BASE");
  string filename = data_path + "/inputdata/all_contact_real_shape=rect1_rep=0000.json";
  if(argc >= 2)
    filename = data_path + string("/inputdata/") + argv[1];
  cout << "filename" << filename << endl;
  // 1. read groundtruth shape and poses into matrix d (column based)
  // shape;
  // poses;

  const int subrate = 1; // subsample rate

  Json::Value data_root;
  read_data(filename, data_root);
  const double probe_radius = data_root["probe_radius"].asDouble();
  const double muc = data_root["muc"].asDouble();  // hack, need to find the right value
  MatrixXd all_contact, dtmp, d;
  parse_json_list_into_matrix(data_root["all_contact"], all_contact);
  downsample(all_contact, subrate, dtmp);
  d = dtmp.transpose();
  const int data_len = d.cols() ;
  const bool isreal = data_root["isreal"].asBool();
  const string surface_id = data_root["surface_id"].asString();
  const string shape_id = data_root["shape_id"].asString();
  const string probe_id = data_root["probe_id"].asString();
  const Vector2d offset(data_root["offset"][0u].asDouble(), data_root["offset"][1].asDouble());

  Noise noise3 = Information(100. * eye(3));

  // 2. create isam problem
  Slam slam;

  // 3. create variable c
  Point1d_Node c_node;
  c_node.init(0.038);
  slam.add_node(&c_node);

  // 4. create constraints
  // for(int i=0;i<n;i++)
  //   find delta_x(from data) - delta_x(from pushing direction, shape, contact point, motion model)
  //
  EllipsoidApproxLimitSurface eals(muc, 0);  // c will be changed
  for(int t=0; t < data_len; t++) {

    Vector2d contact_point = d.block(0, t, 2, 1);
    Vector2d contact_normal = d.block(3, t, 2, 1);
    Vector2d probe_center_now = d.block(13, t, 2, 1);

    const double theta = atan(muc);
    rotate_back_frame2d(
            contact_normal, Vector3d(0, 0, theta),
            &contact_normal, NULL);
    bool fromVicon = true;
    if(fromVicon) {
      Vector2d pt_object;
      MatrixXd M;
      Vector2d normvec_obj, normvec;
      Quaterniond q_now(d(9,t), d(10,t), d(11,t), d(12,t));
      Vector3d groundth_x_now(d(6,t), d(7,t), standardRad(q_now.toRotationMatrix().eulerAngles(2, 1, 0)[0]));
      transform_to_frame2d(probe_center_now,
                           groundth_x_now, &pt_object, NULL);

      M.resize(2,4);
      M << 0.045, -0.045, -0.045, 0.045,
           0.045, 0.045, -0.045, -0.045;

      shape__probe_obj__find_norm(pt_object, M, &normvec_obj);

      rotate_back_frame2d(normvec_obj,
                          groundth_x_now, &normvec, NULL);
      //cout << "groundth_x_now\n" << groundth_x_now << endl;
      //cout << "euler210\n" << q_now.toRotationMatrix().eulerAngles(2, 1, 0).transpose() << endl;
      //cout << "euler012\n" << q_now.toRotationMatrix().eulerAngles(0, 1, 2) << endl;
      //cout << "pt_object\n" << pt_object << endl;
      cout << "normvec_obj\n" << normvec_obj << endl;
      cout << "contact_normal\n" << contact_normal.transpose() << endl;
      cout << "normvec\n" << normvec << endl;
      cout << "diff\n" << (normvec - contact_normal).norm() << endl << endl;
      contact_normal = normvec;
    }

    if (t > 0) {
      Vector2d probe_center_last = d.block(13, t - 1, 2, 1);
      Quaterniond q_now(d(9,t), d(10,t), d(11,t), d(12,t));
      Quaterniond q_last(d(9,t-1), d(10,t-1), d(11,t-1), d(12,t-1));
      Vector3d x_now(d(6,t), d(7,t), quat_to_yaw(q_now));
      Vector3d x_last(d(6,t-1), d(7,t-1), quat_to_yaw(q_last));
      PushMeasurement *push_measurement =
              new PushMeasurement(contact_point, contact_normal,
                                           probe_center_last, probe_center_now);
      PushPoseMeasurement *ppm_measurement =
              new PushPoseMeasurement(x_last, x_now, *push_measurement);

      PushPoseMeasurement_C_Factor *push_factor =
              new PushPoseMeasurement_C_Factor(&c_node, *ppm_measurement, noise3, &eals);
      slam.add_factor(push_factor);
    }
  }
  //slam.write(cout);
  double minn = 100, minc = 0;
  for(int i=1;i<100;i++) {
    c_node.init(i / 1000.0);
    cout << i / 1000.0 << " slam.chi2 " << slam.normalized_chi2() << endl;
    if (slam.normalized_chi2() < minn){
      minn = slam.normalized_chi2();
      minc = i / 1000.0;
    }
  }
  double i = 0.0383;
  c_node.init(i);
  //cout << i << " slam.chi2 " << slam.normalized_chi2() << endl;
  printf("minn %f, minc %f\n", slam.normalized_chi2(), i );
  printf("minn %f, minc %f\n", minn, minc);
  return 0;
  // 5. do optimization
  Properties p = slam.properties();
  p.method = LEVENBERG_MARQUARDT;
  //p.max_iterations = 150;
  p.quiet = false;
  p.verbose = true;
  slam.set_properties(p);
  slam.batch_optimization();
  cout << "slam.chi2" << slam.normalized_chi2() << endl;
  // 6. output c
  cout << "optimal c: " << c_node.vector() << endl;
}