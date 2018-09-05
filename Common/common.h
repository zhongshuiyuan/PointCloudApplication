//
// Created by WuKun on 9/5/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_COMMON_H
#define POINTCLOUDAPPLICATION_COMMON_H

#include <vector>
#include <string>

const std::vector<std::string> type_name_list = { "Uncertain", "RoadEdge", "CrossWalk", "StopLine", "Lane" };

const std::vector<std::string> Uncertain_fields = { "field1", "field2", "field3" };
const std::vector<std::string> RoadEdge_fields = { "field1", "field2", "field3" };
const std::vector<std::string> CrossWalk_fields = { "Type", "BdID", "field3" };
const std::vector<std::string> StopLine_fields = { "TLID", "SignID", "field3" };
const std::vector<std::string> Lane_fields = { "LCnt", "Lno", "field3" };

const std::vector<std::vector<std::string>> fields_vec = { Uncertain_fields, RoadEdge_fields, CrossWalk_fields, StopLine_fields, Lane_fields };

#endif //POINTCLOUDAPPLICATION_COMMON_H
