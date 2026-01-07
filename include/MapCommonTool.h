#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include <list>
// #include <nlohmann/json.hpp>

#include "common_data_memory.h"
#include "common/framework.h"
#include "store/yaml.h"

namespace CommuteMode {
namespace MapFusion {
// using json = nlohmann::json;
// typedef message::map_link_data::s_LinkData_t LinkData;
// typedef message::map_link_data::s_Intersection_t Intersection;
// typedef message::map_link_data::s_Lane_t Lane;
// typedef message::map_link_data::s_LaneConnect_t LaneConnect;
// typedef message::map_hdnv_info::map_hdnv_info MapHdnvInfo;
// typedef message::map2slam::s_Localization_t PositionInfo;
class MapCommonTool {
   public:
    static MapCommonTool* GetInstance();

   HQRawDataMap hq_raw_data_map_;
   std::vector<Intersection> intersection_infos_{};
   std::vector<PositionInfo> position_infos_{};
   std::vector<LinkMemory> link_data_infos_{};
   std::vector<MapHdnvInfo> map_hdnv_infos_{};
   std::vector<LaneMappingMsg> lane_mapping_infos_{};
   std::vector<uint32_t> link_list_{};
   std::vector<LaneMappingMemory> lane_mapping_memories_{};

   public:
   //在HQ路口数据中，找到具体road_id的intersection
   //输入： 1-road_id，intersection的InRoadsId;2-intersection_InnerRoadsIds_map，本地保存的map数据；3- intersection_infos,hq原始路口数据；
   //输出： InRoadsIds里包含road_id的所有intersection
   bool GetIntersectionFromDataMap(uint32_t road_id, std::vector<Intersection>& intersections_res);

   //在HQ linkdata数据中，找到具体road_id的信息
   //输入： 1-road_id，linkdata的road id; 2-link_data_RoadId_map，本地保存的map数据；3-link_infos,hq原始link数据；
   //输出： road_id 的linkData
   bool GetLinkDataFromDataMap(uint32_t road_id, LinkMemory& link_data_res);

   //在lane mapping msg数据中，找到具体link_id的信息
   //输入： 1-目标link id; 2-lane_mapping_msg_link_map，本地保存的map数据；3-link_infos,hq原始link数据；
   //输出： link_id的lane_mapping
   bool GetLaneMappingFromDataMap(uint32_t link_id, LaneMappingMsg& lane_mapping_res);

   bool GetLinkPositionFromPositionInfos(uint32_t link_id, std::vector<PositionInfo>& position_infos);
   
    /*
   1. 在生成的link memory数据中，找到具体link key的信息
   2. 输入： 1-link_key，link memory的road_id及另外一个元素组成的key; 2-link_memory_map，本地保存的map数据；
   3. 输出： link_key的link memory, index
   */
   bool GetLinkMemoryFromDataMap(const uint64_t& link_key, LinkMemory& link_memory_res, uint16_t& index);
   //在lane_mapping_memories_数据中，找到具体link_id的信息
   //输入： 1-目标link id; 
   //输出： link_id的lane_mapping_memory
   bool GetLaneMappingMemoryFromDataMap(uint32_t link_id, LaneMappingMemory& lane_mapping_memory_res);

   //在lane_mapping_memories_数据中，找到具体link_id的信息
   //输入： 1-目标position_id; 
   //输出： position_infos里的position_info
   bool GetPositonFromDataMap(uint32_t position_id, PositionInfo& position_info);

   //获取数据
   bool GetIntersectionInfos(std::vector<Intersection>& intersection_infos);
   bool GetPositionInfos(std::vector<PositionInfo>& position_infos);
   bool GetLinkDataInfos(std::vector<LinkMemory>& link_data_infos);
   bool GetMapHdnvInfos(std::vector<MapHdnvInfo>& map_hdnv_infos);
   bool GetLaneMappingInfos(std::vector<LaneMappingMsg>& lane_mapping_infos);
   bool GetLinkList(std::vector<uint32_t>& link_list);
   bool GetLaneMappingMemories(std::vector<LaneMappingMemory>& lane_mapping_memories);

};

}  // namespace MapFusion
}  // namespace CommuteMode
