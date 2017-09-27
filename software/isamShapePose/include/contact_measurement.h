#pragma once

#include "p_poly_dist.h"
#include "sp_util.h"
#include <isam/isam.h>
#include <vector>

using namespace isam;
using namespace std;
using namespace Eigen;

class ContactMeasurement{
public:
  Point2d _contact_point, _contact_normal, _probe_center;
  double _probe_radius;
  
  ContactMeasurement(Point2d contact_point, Point2d contact_normal, Point2d probe_center, double probe_radius) : 
    _contact_point(contact_point), _contact_normal(contact_normal), 
    _probe_center(probe_center), _probe_radius(probe_radius) {}
  
  void write(std::ostream &out) const {
    out << "(" << "contact_point" << _contact_point << " "
                  "contact_normal" << _contact_normal << " "
                  "probe_center"  <<_probe_center << " "
                  "probe_radius" << "(" << _probe_radius << ")";
  }
  
  friend std::ostream& operator<<(std::ostream& out, const ContactMeasurement& p) {
    p.write(out);
    return out;
  }
};

typedef vector<Point2d_Node*> Shape2d_Node;



class ContactMeasurement_Factor : public FactorT<ContactMeasurement> {
  Shape2d_Node* _shape;   // object shape
  Pose2d_Node*  _pose;    // object pose
public:

    int id; // hack
  /**
   * Constructor.
   * @param shape The shape nodes.
   * @param pose The pose node.
   * @param contact_measurement The contact measurement.
   * @param noise The 2x2 square root information matrix (upper triangular).
   */
  ContactMeasurement_Factor(
     Shape2d_Node*              shape, 
     Pose2d_Node*               pose, 
     const ContactMeasurement&  contact_measurement, 
     const Noise&               noise)
       : FactorT<ContactMeasurement>("ContactMeasurement_Factor", 3, noise, contact_measurement), 
         _shape(shape), 
         _pose(pose) 
  {
    _nodes.resize(1 + _shape->size());
    _nodes[0] = pose;
    for(int i=0; i<_shape->size(); i++)
      _nodes[i+1] = _shape->at(i);
  }

  void initialize() {
  }

  VectorXd basic_error(Selector s = ESTIMATE) const {
    Vector3d x_now(_pose->vector(s));
    
    Vector2d pt_object; 
    MatrixXd dptobject_dxnow;
    transform_to_frame2d(_measure._probe_center.vector(),
       x_now, &pt_object, &dptobject_dxnow);
    
    Vector2d ptc_object; 
    MatrixXd dptcobject_dxnow;
    transform_to_frame2d(_measure._contact_point.vector(),
       x_now, &ptc_object, &dptcobject_dxnow);
      
    MatrixXd M(2, _shape->size());
    for(int i=0; i< _shape->size(); i++){
      M.col(i) = _shape->at(i)->vector(s);
    }
    double dist;
    MatrixXd dd_dxnow, dd_dM, dGi_dxnow, dGi_dM;  // prepare output for p_poly_dist
    Vector2d pt_poly;
    
    p_poly_dist(pt_object, M, dptobject_dxnow, dptcobject_dxnow, 
                &dist, 
                &dd_dxnow, &dd_dM, &dGi_dxnow, &dGi_dM, &pt_poly, NULL, NULL, id);
                
                 
    VectorXd error(3);
    error(0) = (dist - _measure._probe_radius);
    error.tail(2) = ptc_object - pt_poly;
    //cout << "\nptc_object\n" << ptc_object << endl;
    //cout << "\npt_poly\n" << pt_poly << endl;
    return error;
  }

  Jacobian jacobian(){
    return jacobian_c();
  }

  Jacobian jacobian_c() const {
    Vector3d x_now(_pose->vector0());

    Vector2d pt_object; 
    MatrixXd dptobject_dxnow;
    transform_to_frame2d(_measure._probe_center.vector(),
       x_now, &pt_object, &dptobject_dxnow);
    
    Vector2d ptc_object; 
    MatrixXd dptcobject_dxnow;
    transform_to_frame2d(_measure._contact_point.vector(),
       x_now, &ptc_object, &dptcobject_dxnow);
      
    MatrixXd M(2, _shape->size());
    for(int i=0; i< _shape->size(); i++){
      M.col(i) = _shape->at(i)->vector0();
    }
    double dist;
    MatrixXd dd_dxnow, dd_dM, dGi_dxnow, dGi_dM;  // prepare output for p_poly_dist
    Vector2d pt_poly;
    
    p_poly_dist(pt_object, M, dptobject_dxnow, dptcobject_dxnow, 
                &dist, 
                &dd_dxnow, &dd_dM, &dGi_dxnow, &dGi_dM, &pt_poly, NULL, NULL);
                
                 
    VectorXd err(3);
    err(0) = (dist - _measure._probe_radius);
    err.tail(2) = ptc_object - pt_poly;

    MatrixXd tt = sqrtinf();
    VectorXd r = sqrtinf() * err;
    Jacobian jac(r);
    MatrixXd ddG_dxnow(3,3);
    ddG_dxnow << dd_dxnow , dGi_dxnow;
    ddG_dxnow = sqrtinf() * ddG_dxnow;
    jac.add_term(_pose, ddG_dxnow);
    
    
    MatrixXd ddGi_dMi(3,2);
//  remove shape from estimation
//    for(int i=0; i<_shape->size(); i++){
//      ddGi_dMi << dd_dM.col(i).transpose(), dGi_dM.middleCols(i*2, 2);
//      ddGi_dMi = sqrtinf() * ddGi_dMi;
//      jac.add_term(_shape->at(i), ddGi_dMi);
//    }
    return jac;
  }
  void write(std::ostream &out) const {
    FactorT<ContactMeasurement>::write(out);
    out << "\nerror\n" << error() << endl;
    out << "\njacobian\n" << jacobian_c() << endl;
  }
  friend std::ostream& operator<<(std::ostream& output, ContactMeasurement_Factor& e) {
    e.write(output);
    return output;
  }
};
