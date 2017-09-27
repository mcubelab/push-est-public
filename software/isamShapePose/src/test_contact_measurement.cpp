#include <iostream>
#include <isam/isam.h>
#include "sp_util.h"
#include "contact_measurement.h"
#include "EllipsoidApproxLimitSurface.h"

using namespace std;  
using namespace isam;  

int main(){
	Point2d contact_point(1,1);
  Point2d contact_normal(1,1);
  Point2d probe_center(1.5,1);
	Vector2d probe_center_next(1.6,1);
	double probe_radius(0.09);
  const double guess_shape_radius = 0.025;
  vector<Point2d_Node*> shape_nodes;
	ContactMeasurement* cm = new ContactMeasurement(contact_point, contact_normal,
                                            probe_center, probe_radius);

  Noise noise3 = Information(100. * eye(3));
  Pose2d_Node* pose_now = new Pose2d_Node();
  Pose2d_Node* pose_next = new Pose2d_Node();
  const double muc = 0.1; const double c = 3;
  EllipsoidApproxLimitSurface eals(muc, c);

  pose_now->init(Pose2d(0,0,0));
  pose_next->init(Pose2d(0,0,0));
  int nM  = 4;
  for(int i=0; i<nM; i++){
    Point2d_Node* new_point_node = new Point2d_Node();
    shape_nodes.push_back(new_point_node);
  }

  // 2.2 add data for shape prior
  for(int i=0; i<nM; i++){
    const double rad = (i+ 0.5)/nM * M_PI*2;
    Point2d* new_point = new Point2d(cos(rad)*guess_shape_radius, sin(rad)*guess_shape_radius);  // should be moved out
    shape_nodes[i]->init(*new_point);
  }
  //&shape_nodes, pose_nodes[t], *contact_measurement, noise3
  ContactMeasurement_Factor* cmf = new ContactMeasurement_Factor(&shape_nodes, pose_now, *cm, noise3);
  isam::Jacobian j1 = cmf->jacobian_internal(true);
  isam::Jacobian j2 = cmf->jacobian();
  cout << j1 << endl;
  cout << j2 << endl;


}
