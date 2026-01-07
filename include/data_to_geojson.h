#ifndef	_DATA_TO_GEOJSON_H_
#define	_DATA_TO_GEOJSON_H_

#include "hdm_dbg_log.h"
#include "common_data_memory.h"

namespace CommuteMode {
namespace MapFusion {
using namespace earth::shell::framework;

class CDataToJson
{
public:
    CDataToJson();
	virtual ~CDataToJson();
 
	bool GenerateData(const TopicTrait::MapMapMsg& map_msg, const TopicTrait::MapPositionMsg& position_msg,
                      const TopicTrait::MapSwitchInfoMsg& switch_info_msg, const TopicTrait::MapRouteListMsg& map_route_list,
                      const TopicTrait::MapGlobalDataMsg& global_data_msg, const TopicTrait::MapDynamicMsg& map_dynamic_msg,
                      uint64_t efm_counter);

    bool MakeMemoryLinkLane(std::vector<MemoryLink> memory_links_data);
    bool MakeMemoryLinkPolygen(std::vector<LinkPolygon> links_polygon, const uint8_t& route_id);
    bool MakeMemoryBoundaryLine(std::vector<int64_t> link_list, std::vector<LaneMappingMsg> lane_mapping_infos);
    bool MakeCrossLaneInfos(std::vector<CrossLaneInfo> memory_cross_lane_infos, const uint8_t& route_id);
    bool MakeMemoryCenterLine(std::vector<LinkDataCenterLine> link_data_center_line);
    bool MakeMemoryPosition(const std::vector<PositionPointInfo>& position_infos, const uint8_t& route_id);
    bool MakeMemoryPositionCsv(const std::vector<PositionInfo>& position_infos);
    bool MakeLaneMappingPosition(const std::vector<LaneMappingMsg>& lane_mapping_infos, const uint8_t& route_id);
    bool MakeLaneMappingLine(const std::vector<LaneMappingMsg>& lane_mapping_infos, const uint8_t& route_id);
    bool MakeMemoryAllLines(const std::vector<Line>& memory_all_lines, const uint8_t& route_id);

    static bool DownTrajHistoryToGeoJson(const LaneMappingMsg& lane_mapping_msg,std::string file_name, int& write_times);
    static bool DownLinesToGeoJson(const LaneMappingMsg& lane_mapping_msg, std::string file_name, int& write_times);
    static bool DownLinesToGeoJson(const LaneMappingMemory& lane_mapping_msg, std::string file_name, int& write_times);
    static bool DownTrajToGeoJson(const LaneMappingMemory& lane_mapping_msg, std::string file_name, int& write_times);
private:

    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_idkerb_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_connect_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_cur_map_;
    std::unordered_map<uint32_t, uint8_t>  lineatobject_id_cur_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_cur_geofence_;
    std::shared_ptr<const TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const TopicTrait::MapPositionMsg> position_msg_;
    std::shared_ptr<const TopicTrait::MapSwitchInfoMsg> switch_info_msg_;
    std::shared_ptr<const TopicTrait::MapRouteListMsg> map_route_list_;
    std::shared_ptr<const TopicTrait::MapGlobalDataMsg> global_data_msg_;
    std::shared_ptr<const TopicTrait::MapDynamicMsg> map_dynamic_msg_;

    uint32_t write_times_;
    uint32_t write_times_kerb_;
    uint32_t write_times_connect_;
    uint32_t write_times_global_;
    uint64_t efm_counter_;
    uint64_t position_counter_;
    const char* file_name_char_section_c_;
    const char* file_name_char_section_l_;
    const char* file_name_char_section_kerb_;
    const char* file_name_char_position_;
    const char* file_name_char_connect_;
    const char* file_name_char_curvature_;
    const char* file_name_char_switch_;
    const char* file_name_char_switch_global_;
    const char* file_name_char_geofence_;


};

}
}

#endif	//	_DATA_TO_GEOJSON_H_
