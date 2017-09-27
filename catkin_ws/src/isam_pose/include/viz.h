//
// Created by mcube on 1/6/17.
//

#ifndef ISAMSHAPEPOSE_VIZ_H
#define ISAMSHAPEPOSE_VIZ_H

#include "bot_lcmgl_client/lcmgl.h"
#include "contact_measurement.h"


class LcmGL_Visualizer{
    lcm_t *_lcm;
    bot_lcmgl_t* _lcmgl;
    bool _toviz;
    bool _topause;
    unsigned int _sleep_microseconds;
    vector<Pose2d_Node*>& _pose_nodes;
    vector<Point2d_Node*>& _shape_nodes;
    vector<Point2d*>& _shape_data;
    vector<ContactMeasurement*>& _contact_data;
public:
    LcmGL_Visualizer(vector<Point2d_Node*>& shape_nodes,
                     vector<Pose2d_Node*>& pose_nodes,
                     vector<Point2d*>& shape_data,
                     vector<ContactMeasurement*>& contact_data,
                     bool toviz,
                     bool topause,
                     unsigned int sleep_microseconds):
            _toviz(toviz),
            _topause(topause),
            _sleep_microseconds(sleep_microseconds),
            _shape_nodes(shape_nodes),
            _pose_nodes(pose_nodes),
            _shape_data(shape_data),
            _contact_data(contact_data),
            _lcm(lcm_create(NULL)),
            _lcmgl(bot_lcmgl_init(_lcm, "isam"))
    {}

    ~LcmGL_Visualizer() {
      lcm_destroy(_lcm);
    }
    void visualize();
};

class Screen_Displayer{
    vector<Point2d_Node*>& _shape_nodes;
    vector<Pose2d_Node*>& _pose_nodes;
    bool _toprint;
public:
    Screen_Displayer(vector<Point2d_Node*>& shape_nodes,
                     vector<Pose2d_Node*>& pose_nodes,
                     bool toprint):
      _shape_nodes(shape_nodes),
      _pose_nodes(pose_nodes),
      _toprint(toprint){}

    void print_before(int t){
      if (!_toprint) return;
      cout << "Before slam update, t=" << t << endl;
      write(t);
    }
    void print_after(int t){
      if (!_toprint) return;
      cout << "After slam update, t=" << t << endl;
      write(t);
    }
    void write(int t){
      cout << endl;
      for (int i = 0; i < _shape_nodes.size(); i++)
        cout << "Shape[" << i << "]: " << _shape_nodes[i]->value() << endl;
      for (int i = 0; i < t + 1; i++)
        cout << "Pose[" << i << "]: " << _pose_nodes[i]->value() << endl;

      cout << endl;
    }
};

#endif //ISAMSHAPEPOSE_VIZ_H
