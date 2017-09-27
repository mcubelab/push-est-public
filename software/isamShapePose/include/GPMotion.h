//
// Created by mcube on 2/16/17.
//

#ifndef ISAMSHAPEPOSE_GPMOTION_H
#define ISAMSHAPEPOSE_GPMOTION_H

#endif //ISAMSHAPEPOSE_GPMOTION_H
#include "sp_util.h"

using namespace std;
using namespace Eigen;

class GPMotion {
public:
    // data
    double _c_lb, _c_ub;
    double _beta_lb, _beta_ub;
    int _n_c, _n_beta;
    double _c_step, _beta_step;  // = (_c_ub - _c_lb) / (_n_c)
    static const int SIZEC = 201;
    static const int SIZEBETA = 641;
    double _mean_x[SIZEC][SIZEBETA], _mean_y[SIZEC][SIZEBETA], _mean_theta[SIZEC][SIZEBETA];  // don't exceed 200
    double _var_x[SIZEC][SIZEBETA], _var_y[SIZEC][SIZEBETA], _var_theta[SIZEC][SIZEBETA];  // don't exceed 200

    GPMotion(string filename);
    //function [x_new, xc, slip, normvec_now, incontact] = pushit_slip_cylinder(obj,x,xcylinder,xcylinder_end, toviz, xc, normvec)

    // x_now: current object pose x_t [x,y,theta]
    // x_cylinder: pusher pose x_t [x,y]       (in global frame)
    // x_cylinder_end: pusher pose x_t+1 [x,y] (in global frame)
    // x_contact: contact point [x,y]                (in global frame)

    // return
    // x_delta
    // d_x_delta__d_x_now
    void pushit_ad(
            const Vector3d& x_now,
            const Vector2d& x_cylinder,
            const Vector2d& x_cylinder_end,
            const Vector2d& x_contact,
            Vector3d& x_delta,
            Matrix3d& d_x_delta__d_xnow
    ) ;



    void query_GP_ad(
            const adouble& c,
            const adouble& beta,
            Vector3a& x_delta
    );
};