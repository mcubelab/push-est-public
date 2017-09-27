//
// Created by mcube on 8/17/17.
//

#ifndef ISAM_POSE_EKF_H
#define ISAM_POSE_EKF_H

#include <Eigen/Dense>
void EKFpredict(VectorXd old_state, MatrixXd old_state_cov, VectorXd control, VectorXd measurement,
                MatrixXd system_noise,
                VectorXd& new_state_pred, MatrixXd& new_state_cov_pred){

  new_state_pred = old_state;
  new_state_cov_pred = old_state_cov + system_noise;
}


void EKFupdate(VectorXd new_state_pred, MatrixXd new_state_cov_pred,
               MatrixXd H,  // dh / dx  n by 3
               MatrixXd Q,  // sensor noise n by n
               VectorXd h,
               VectorXd& new_state, MatrixXd& new_state_cov){

  MatrixXd K;  //Kalman Gain
  K = new_state_cov_pred * H.transpose() * (H * new_state_cov_pred * H.transpose() + Q).inverse();
  new_state = new_state_pred + K * (-h);
  new_state_cov = (MatrixXd::Identity(3, 3) - K * H) * new_state_cov_pred;
}

#endif //ISAM_POSE_EKF_H
