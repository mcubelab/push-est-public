//
// Created by mcube on 4/23/17.
//

#ifndef ISAM_POSE_FEEDER_H
#define ISAM_POSE_FEEDER_H

#include <sp_util.h>

class Feeder {
public:
    Json::Value root;
    Feeder(const string jsonfilepath){
        ifstream fin(jsonfilepath);
        fin >> root;
    }

    bool getValues(unsigned int t, VectorXd& object_apriltag__world,
                   vecVec& pusher2d__world,
                   vecVec& contact_force__world, vecVec& contact_point__world,
                   vecVec& contact_normal__world,
                   VectorXd& Vector_object_vicon__world,
                   bool& updated_apriltag, bool updated_contact_point[], bool updated_ft[]);
};

#endif //ISAM_POSE_FEEDER_H
