//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_COMMON_H
#define POINTCLOUDAPPLICATION_COMMON_H

#include <osg/Vec3d>

const osg::Vec3d ground_center_location = osg::Vec3d(23.0304, 113.783, 0.0);

const char root_node_name[] = "root_node";
const char earth_node_name[] = "earth_node";
const char vmap_node_name[] = "vmap_node";
const char text_node_name[] = "text_node";
const char temp_node_name[] = "temp_node";

const char point_cloud_node_name[] = "point_cloud_node";
const char vector_item_node_name[] = "vector_item_node";
const char trace_item_node_name[] = "trace_item_node";

const char point_text_node_name[] = "point_text_node";
const char line_text_node_name[] = "line_text_node";
const char area_text_node_name[] = "area_text_node";
const char node_text_node_name[] = "node_text_node";
const char lane_text_node_name[] = "lane_text_node";
const char dtlane_text_node_name[] = "dtlane_text_node";

//used for SelectEditor, do not change them
const char vector_item_name[] = "item_vector_";
const char trace_item_name[] = "item_trace_";

#endif //POINTCLOUDAPPLICATION_COMMON_H
