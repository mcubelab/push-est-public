//
// Created by mcube on 4/3/17.
//

#ifndef ISAM_POSE_ROSMSGBUFFER_H
#define ISAM_POSE_ROSMSGBUFFER_H

#include "ros/ros.h"
#include <cstdlib>
using namespace std;

template<typename T>
class RosMsgBuffer{
private:
    T msg_;
    bool updated_;
    ros::Subscriber sub_;
public:
    RosMsgBuffer(std::string topic_name, ros::NodeHandle& node): updated_(false){
      sub_ = node.subscribe<T>(topic_name, 1, &RosMsgBuffer<T>::callback, this);
    }
    void callback(T msg){
      // need to use mutex to protect msg_
      msg_ = msg;
      updated_ = true;
    }
    bool getmsg(T* msg);
};

template<typename T>
bool RosMsgBuffer<T>::getmsg(T* msg){
  // need to use mutex to protect msg_
  if(updated_) {
    *msg = msg_;
    updated_ = false;
    return true;
  }
  return false;
}
#endif //ISAM_POSE_ROSMSGBUFFER_H