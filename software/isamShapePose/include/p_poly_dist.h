
// function [d,dd_dx,dd_dM,dG_dx,dG_dM,pt_poly,contact_type,normvec] = p_poly_dist2(pt, M, dpt_dx, dptc_dx) 

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "sp_util.h"

using namespace Eigen;
using namespace std;


double min(const VectorXd& a,
           int*  minI);

VectorXi find(const VectorXb& a);

void p_poly_dist( const Vector2d& pt,
                  const MatrixXd& M,
                  const MatrixXd& dpt_dx,
                  const MatrixXd& dptc_dx,

                  double *d,
                  MatrixXd* dd_dx,
                  MatrixXd* dd_dM,
                  MatrixXd* dG_dx,
                  MatrixXd* dG_dM,
                  Vector2d* pt_poly,
                  int*      contact_type,
                  Vector2d* normvec,
                  int hack = 1
);


void shape__probe_obj__find_norm(const Vector2d& pt_obj,
                                 const MatrixXd& M,

                                 Vector2d* normvec);