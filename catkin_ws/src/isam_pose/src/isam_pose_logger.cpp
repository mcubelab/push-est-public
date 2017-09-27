//
// Created by mcube on 4/16/17.
//

#include "isam_pose_logger.h"

#include <json/json.h>


Json::Value matrix_to_json(const MatrixXd& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.rows();
  int m = input.cols();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    v_json[i] = Json::Value(Json::arrayValue);
    v_json[i].resize(m);
    for (int j = 0; j < m; j++)
      v_json[i][j] = input(i,j);
  }
  return v_json;
}

Json::Value vectorVector_to_json(const vector<VectorXd>& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.size();
  if(n == 0)
    return v_json;

  int dim = input[0].size();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    Json::Value vv_json(Json::arrayValue);
    vv_json.resize(dim);
    for (int j = 0; j < dim; j++) {
      vv_json[j] = input[i][j];
    }
    v_json[i] = vv_json;
  }
  return v_json;
}

Json::Value vectorMatrix_to_json(const vector<MatrixXd>& input){
  int nt = input.size();
  Json::Value v_json = jsonArray(nt);
  for (int t = 0; t < nt; t++) {
    int nrow = input[t].rows();
    v_json[t] = jsonArray(nrow);

    for (int i = 0; i < nrow; i++) {
      int ncol = input[t].cols();
      v_json[t][i] = jsonArray(ncol);
      for (int j = 0; j < ncol; j++) {
        v_json[t][i][j] = input[t](i,j);
      }
    }
  }
  return v_json;
}

Json::Value VectorXd_to_json(const VectorXd& input){
  Json::Value v_json(Json::arrayValue);
  int dim = input.size();
  v_json.resize(dim);
  for (int j = 0; j < dim; j++) {
    v_json[j] = input[j];
  }
  return v_json;
}

Json::Value vector_to_json(const vector<double>& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.size();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    v_json[i] = input[i];
  }
  return v_json;
}

Json::Value vectorbool_to_json(const vector<bool>& input)  {
  Json::Value v_json(Json::arrayValue);
  int n = input.size();
  v_json.resize(n);
  for (int i = 0; i < n; i++) {
    v_json[i] = input[i];
  }
  return v_json;
}

Json::Value jsonArray(int n){
  Json::Value a;
  a.resize(n);
  return a;
}

void Logger::save_to_json(Isam_param * isam_param){
  Json::Value jsonvalue;
  // "shape_star_batch"
  {
    int nM = shape_nodes_->size();
    int nMdim = 2;
    Json::Value M_star_json = jsonArray(nM);
    for (int i = 0; i < nM; i++) {
      M_star_json[i] = jsonArray(nMdim);
      for (int j = 0; j < nMdim; j++)
        M_star_json[i][j] = shape_nodes_->at(i)->vector()[j];
    }
    jsonvalue["M_star_batch"] = M_star_json;
  }

  // "pose_star_batch"
  {
    int nx = pose_nodes_->size();
    int nxdim = 3;
    Json::Value x_star_json = jsonArray(nx);
    for (int i = 0; i < nx; i++) {
      x_star_json[i] = jsonArray(nxdim);
      for (int j = 0; j < nxdim; j++)
        x_star_json[i][j] = pose_nodes_->at(i)->vector()[j];
    }
    jsonvalue["x_star_batch"] = x_star_json;
  }

  // "shape_inc"
  {
    int nt = shape_inc.size();
    int nM = shape_nodes_->size();
    int nMdim = 2;
    Json::Value shape_inc_json = jsonArray(nt);
    for (int t = 0; t < nt; t++) {
      shape_inc_json[t] = jsonArray(nM);
      for (int i = 0; i < nM; i++) {
        shape_inc_json[t][i] = jsonArray(nMdim);
        for (int j = 0; j < nMdim; j++) {
          shape_inc_json[t][i][j] = shape_inc[t][i][j];
        }
      }
    }
    jsonvalue["shape_inc"] = shape_inc_json;  // t * nM * nMdim
  }

  // "pose_inc"
  jsonvalue["pose_inc"] = vectorVector_to_json(pose_inc);  // t * ndim(3)

  //"pose_inc_cov"
  jsonvalue["pose_inc_cov"] = vectorMatrix_to_json(pose_inc_cov);  // t * ndim(3)*ndim(3)

  // "pose_inc"
  jsonvalue["pose_ekf"] = vectorVector_to_json(pose_ekf);  // t * ndim(3)

  //"pose_ekf_cov"
  jsonvalue["pose_ekf_cov"] = vectorMatrix_to_json(pose_inc_cov);  // t * ndim(3)*ndim(3)

  // "shape_input"
  jsonvalue["shape_input"] = vectorVector_to_json(shape_input);  // nM * nMdim

  // "pose_input"
  jsonvalue["pose_input"] = vectorVector_to_json(pose_input);  // t * ndim(3)

  // "pusher"
  jsonvalue["pusher"] = jsonArray(2);
  jsonvalue["contact_normal"] = jsonArray(2);
  jsonvalue["contact_point"] = jsonArray(2);
  jsonvalue["has_contact"] = jsonArray(2);

  for(int side=0; side<2; side++) {
    jsonvalue["pusher"][side] = vectorVector_to_json(pusher2d__world[side]);
    jsonvalue["contact_normal"][side] = vectorVector_to_json(contact_normal__world[side]);
    jsonvalue["contact_point"][side] = vectorVector_to_json(contact_point__world[side]);
    jsonvalue["contact_force"][side] = vectorVector_to_json(contact_force__world[side]);
    jsonvalue["has_contact"][side] = vectorbool_to_json(has_contact[side]);
  }
  jsonvalue["has_apriltag"] = vectorbool_to_json(has_apriltag);

  jsonvalue["inc_time"] = vector_to_json(inc_time);
  jsonvalue["pose_true"] = vectorVector_to_json(pose_true);
  jsonvalue["probe_radius"] = isam_param->probe_radius;
  jsonvalue["label"] = isam_param->label;
  jsonvalue["startdate"] = isam_param->startdate;
  jsonvalue["shape_id"] = isam_param->shape_id;
  jsonvalue["offset"] = VectorXd_to_json(isam_param->offset);

  ofstream fout(isam_param->outfilename);
  fout << jsonvalue;
}