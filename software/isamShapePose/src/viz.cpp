//
// Created by mcube on 1/6/17.
//

#include "viz.h"


void LcmGL_Visualizer::visualize(){

  if(!_toviz) return;

  bot_lcmgl_draw_axes(_lcmgl);
  
  for(int i=_contact_data.size()-1; i<_contact_data.size(); i++){
    Point2d tmp1 = _contact_data[i]->_contact_point;
    double xyz1[] = {tmp1.x(), tmp1.y(), 0};
    
    bot_lcmgl_color3f(_lcmgl, 0.0, 0.5, 0.5); // cyan
    bot_lcmgl_sphere(_lcmgl, xyz1, 0.001, 36, 36);
    
    Point2d tmp2 = _contact_data[i]->_probe_center;
    double xyz2[] = {tmp2.x(), tmp2.y(), 0};

    bot_lcmgl_color3f(_lcmgl, 0.0, 0.0, 1.0); // blue
    bot_lcmgl_cylinder(_lcmgl, xyz2, _contact_data[i]->_probe_radius, _contact_data[i]->_probe_radius, 0.1, 36, 36);
  }
  
  for(int i=_pose_nodes.size()-1; i<_pose_nodes.size(); i++){
    Pose2d p = _pose_nodes[i]->value();
    for(int j=0; j<_shape_nodes.size(); j++){
      int j_next = (j+1)%_shape_nodes.size();
      Point2d tmp1 = p.transform_from(_shape_nodes[j]->value());
      double xyz1[] = {tmp1.x(), tmp1.y(), 0};
      Point2d tmp2 = p.transform_from(_shape_nodes[j_next]->value());
      double xyz2[] = {tmp2.x(), tmp2.y(), 0};
      
      bot_lcmgl_color3f(_lcmgl, 1.0, 0.0, 0.0); // red
      bot_lcmgl_sphere(_lcmgl, xyz1, 0.001, 36, 36);
      //bot_lcmgl_sphere(_lcmgl, xyz2, 0.001, 36, 36);
      char buff[10];
      sprintf(buff, "%d", j);
      bot_lcmgl_text(_lcmgl, xyz1, buff);
      // connect the dots
      
      bot_lcmgl_begin(_lcmgl, LCMGL_LINES);
      bot_lcmgl_line_width(_lcmgl, 100.0);
      bot_lcmgl_color3f(_lcmgl, 1, 0.5, 0.5);
      bot_lcmgl_vertex3f(_lcmgl, xyz1[0], xyz1[1], xyz1[2]);
      bot_lcmgl_vertex3f(_lcmgl, xyz2[0], xyz2[1], xyz2[2]);
      bot_lcmgl_end(_lcmgl);
      //bot_lcmgl_line(_lcmgl, xyz1[0], xyz1[1], xyz2[0], xyz2[1]);
      /*
      Point2d tmp3 = p.transform_from(*(shape_data[j]));
      double xyz3[] = {tmp3.x(), tmp3.y(), 0};
      Point2d tmp4 = p.transform_from(*(shape_data[j_next]));
      double xyz4[] = {tmp4.x(), tmp4.y(), 0};
      bot_lcmgl_color3f(_lcmgl, 0.0, 1.0, 0.0); // green
      bot_lcmgl_sphere(_lcmgl, xyz3, 0.001, 36, 36);
      bot_lcmgl_sphere(_lcmgl, xyz4, 0.001, 36, 36);
      */
      // connect the dots
      // bot_lcmgl_line_width(_lcmgl, 1.0);
      //
      // bot_lcmgl_color3f(_lcmgl, 1, 0.5, 0.5);
      // bot_lcmgl_begin(_lcmgl, LCMGL_LINES);
      // bot_lcmgl_vertex2f(_lcmgl, xyz1[0], xyz1[1]);
      // bot_lcmgl_vertex2f(_lcmgl, xyz2[0], xyz2[1]);
    }
  }
  bot_lcmgl_switch_buffer(_lcmgl);
  //usleep(_sleep_microseconds);
  if(_topause/* && _pose_nodes.size() == 80*/) pk();
}