//
// Created by mcube on 4/23/17.
//

#include "Feeder.h"

bool Feeder::getValues(unsigned int t, VectorXd& object_apriltag__world,
               vecVec& pusher2d__world,
               vecVec& contact_force__world, vecVec& contact_point__world,
               vecVec& contact_normal__world,
               VectorXd& Vector_object_vicon__world,
               bool& updated_apriltag, bool updated_contact_point[], bool updated_ft[]){
  if(t >= root["has_apriltag"].size())
    return false;

  object_apriltag__world = parse_jsonval_into_vector(root["pose_input"][t]);
  Vector_object_vicon__world = parse_jsonval_into_vector(root["pose_true"][t]);;

  for(int side=0; side<2; side++) {
    pusher2d__world[side] = parse_jsonval_into_vector(root["pusher"][side][t]);
    contact_force__world[side] = parse_jsonval_into_vector(root["contact_force"][side][t]);
    contact_point__world[side] = parse_jsonval_into_vector(root["contact_point"][side][t]);
    contact_normal__world[side] = parse_jsonval_into_vector(root["contact_normal"][side][t]);
    updated_contact_point[side] = root["contact_point"][side][t].asBool();
    updated_ft[side] = root["has_contact"][side][t].asBool();
    updated_contact_point[side] = root["has_contact"][side][t].asBool();
  }

  updated_apriltag = root["has_apriltag"][t].asBool();;
  return true;
}