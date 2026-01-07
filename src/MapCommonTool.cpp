#include "MapCommonTool.h"

namespace CommuteMode {
namespace MapFusion {
MapCommonTool* MapCommonTool::GetInstance() {
    static MapCommonTool instance;
    return &instance;
}

bool MapCommonTool::GetIntersectionFromDataMap(uint32_t road_id, std::vector<Intersection>& intersections_res){
    intersections_res.clear();
    if(hq_raw_data_map_.intersection_InnerRoadsIds_map.find(road_id)==hq_raw_data_map_.intersection_InnerRoadsIds_map.end()){
        return false;
    }
    std::vector<int> index_vec = hq_raw_data_map_.intersection_InnerRoadsIds_map.at(road_id);
    for(auto index:index_vec){
        if(intersection_infos_.size()>index && index>=0){
            intersections_res.push_back(intersection_infos_[index]);
        }
    }

    if(intersections_res.empty()){
        return false;
    }
    return true;
}

bool MapCommonTool::GetLinkDataFromDataMap(uint32_t road_id, LinkMemory& link_data_res){
    if(hq_raw_data_map_.link_data_RoadId_map.find(road_id)==hq_raw_data_map_.link_data_RoadId_map.end()){
        return false;
    }else{
        int index = hq_raw_data_map_.link_data_RoadId_map.at(road_id);
        link_data_res = link_data_infos_.at(index);
    }
    return true;
}

bool MapCommonTool::GetLinkMemoryFromDataMap(const uint64_t& link_key, LinkMemory& link_memory_res, uint16_t& index){
    if(hq_raw_data_map_.link_memory_map.find(link_key)==hq_raw_data_map_.link_memory_map.end()){
        return false;
    }else{
        index = hq_raw_data_map_.link_memory_map.at(link_key);
        link_memory_res = link_data_infos_.at(index);
    }
    return true;
}

bool MapCommonTool::GetLaneMappingFromDataMap(uint32_t link_id, LaneMappingMsg& lane_mapping_res){
    if(hq_raw_data_map_.lane_mapping_msg_link_map.find(link_id)==hq_raw_data_map_.lane_mapping_msg_link_map.end()){
        return false;
    }else{
        int index = hq_raw_data_map_.lane_mapping_msg_link_map.at(link_id);
        lane_mapping_res = lane_mapping_infos_.at(index);
    }
    return true;
}

bool MapCommonTool::GetLinkPositionFromPositionInfos(uint32_t link_id, std::vector<PositionInfo>& position_infos){
    for(const auto& position_info:position_infos_){
        if(position_info.link_id == link_id){
            position_infos.push_back(position_info);
        }
    }
    return true;
}

bool MapCommonTool::GetLaneMappingMemoryFromDataMap(uint32_t link_id, LaneMappingMemory& lane_mapping_memory_res){
    if(hq_raw_data_map_.lane_mapping_memory_link_map.find(link_id)==hq_raw_data_map_.lane_mapping_memory_link_map.end()){
        return false;
    }else{
        int index = hq_raw_data_map_.lane_mapping_memory_link_map.at(link_id);
        lane_mapping_memory_res = lane_mapping_memories_.at(index);
    }   
    return true;
}

bool MapCommonTool::GetIntersectionInfos(std::vector<Intersection>& intersection_infos){
    intersection_infos.clear();
    intersection_infos = intersection_infos_;
    if (intersection_infos_.empty()){
        return false;
    }
    return true;
}

bool MapCommonTool::GetPositionInfos(std::vector<PositionInfo>& position_infos){
    position_infos.clear();
    position_infos = position_infos_;
    if (position_infos_.empty()){
        return false;
    }
    return true;
}
bool MapCommonTool::GetLinkDataInfos(std::vector<LinkMemory>& link_data_infos){
    link_data_infos.clear();
    link_data_infos = link_data_infos_;
    if (link_data_infos_.empty()){
        return false;
    }
    return true;
}
bool MapCommonTool::GetMapHdnvInfos(std::vector<MapHdnvInfo>& map_hdnv_infos){
    map_hdnv_infos.clear();
    map_hdnv_infos = map_hdnv_infos_;
    if (map_hdnv_infos_.empty()){
        return false;
    }
    return true;
}
bool MapCommonTool::GetLaneMappingInfos(std::vector<LaneMappingMsg>& lane_mapping_infos){
    lane_mapping_infos.clear();
    lane_mapping_infos = lane_mapping_infos_;
    if (lane_mapping_infos_.empty()){
        return false;
    }
    return true;
}
bool MapCommonTool::GetLinkList(std::vector<uint32_t>& link_list){
    link_list.clear();
    link_list = link_list_;
    if (link_list_.empty()){
        return false;
    }
    return true;
}

bool MapCommonTool::GetLaneMappingMemories(std::vector<LaneMappingMemory>& lane_mapping_memories){
    lane_mapping_memories.clear();
    lane_mapping_memories = lane_mapping_memories_;
    if (lane_mapping_memories_.empty()){
        return false;
    }
    return true;
}

}  // namespace MapFusion
}  // namespace CommuteMode
