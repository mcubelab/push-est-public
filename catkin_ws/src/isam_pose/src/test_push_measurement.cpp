#include <iostream>
#include <isam/isam.h>
#include "sp_util.h"
#include "push_measurement.h"
#include "EllipsoidApproxLimitSurface.h"

using namespace std;  
using namespace isam;  

int main(){
	Vector2d contact_point(1,1);
	Vector2d contact_normal(1,1);
	Vector2d probe_center(1.5,1);
	Vector2d probe_center_next(1.6,1);
	//double probe_radius(0.1);
	PushMeasurement* pm = new PushMeasurement(contact_point, contact_normal, 
                                            probe_center, probe_center_next);

  Noise noise3 = Information(100. * eye(3));
  Pose2d_Node* pose_now = new Pose2d_Node();
  Pose2d_Node* pose_next = new Pose2d_Node();
  const double muc = 0.1; const double c = 3;
  EllipsoidApproxLimitSurface eals(muc, c);

  pose_now->init(Pose2d(0,0,0));
  pose_next->init(Pose2d(0,0,0));

  PushMeasurement_Factor* pmf = new PushMeasurement_Factor(pose_now, pose_next, *pm, noise3, &eals);
  isam::Jacobian j1 = pmf->jacobian_internal(true);
  isam::Jacobian j2 = pmf->jacobian();
  cout << j1 << endl;
  cout << j2 << endl;
  Vector3d x_now();
  Vector3d x_next();
  Vector2d zz();
  Vector3d x_delta_global(0,0,0);
  Matrix3d d_x_delta_global__d_x_now;


  pmf += 1;
}
