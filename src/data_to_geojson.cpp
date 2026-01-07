#include "data_to_geojson.h"
#include <iostream>
#include <string>
#include <typeinfo>

namespace CommuteMode {
namespace MapFusion {
using namespace earth::shell::framework;

CDataToJson::CDataToJson() : write_times_(0), write_times_kerb_(0), write_times_connect_(0), write_times_global_(0) {
}

CDataToJson::~CDataToJson() {}


bool CDataToJson::MakeMemoryLinkPolygen(std::vector<LinkPolygon> links_polygon,
                                        const uint8_t& route_id) {
    // std::string geojson_file_lane_mapping_position;
        // 根据运行环境配置默认日志路径
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_links_polygen = folder_path + "/position_polygon_map.geojson";


    const char* linklane_file = geojson_file_links_polygen.c_str();

    // 每次调用时清除原来文本的内容
    if (std::remove(linklane_file) != 0) {
        // std::cout << "Error deleting file: " << linklane_file << std::endl;
    } else {
        std::cout << "File " << linklane_file << " successfully deleted." << std::endl;
    }

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        linklane_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_polygon\",\n\"features\":[\n");

    if (links_polygon.empty()) {
        return true;
    }

    for (const auto& link_polygen : links_polygon) {
        hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, false,
        "{\"type\":\"Feature\", \"properties\":{\"id\":%lu,\"start_point_id\":%lu,\"end_point_id\":%lu},",
        link_polygen.link_id, link_polygen.start_point_id, link_polygen.end_point_id);

        hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, false,
        "\"geometry\":{\"type\":\"MultiPolygon\",\"coordinates\":[[[");
        for (size_t i = 0; i < link_polygen.line_coordinates.size(); i++) {
            if (i == 0) {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, false,
                "[%lf, %lf]", link_polygen.line_coordinates[i].x, link_polygen.line_coordinates[i].y);
            } else {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, false,
                ",[%lf, %lf]", link_polygen.line_coordinates[i].x, link_polygen.line_coordinates[i].y);
            }

        }
        // for(const auto& points : link_polygen.line_coordinates){
        //     hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, false,
        //     "[%lf, %lf],",points.lon, points.lat);
        // }
        hdm_utility::CHdmDbgLog::GetInstance()->Write(linklane_file, true,
        "]]]}},\n");

    }

    return true;
}

bool CDataToJson::MakeCrossLaneInfos(std::vector<CrossLaneInfo> memory_cross_lane_infos,
                                    const uint8_t& route_id) {
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_links_cross_lane = folder_path + "/cross_lane_map.geojson";


    const char* cross_lane_file = geojson_file_links_cross_lane.c_str();

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
    cross_lane_file, true,
    "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_line2\",\n\"features\":[\n");

    if (memory_cross_lane_infos.empty()) {
        return true;
    }

    size_t cross_lane_cnt = 1;
    size_t cross_lane_size = memory_cross_lane_infos.size();
    for (const auto& cross_lane_info : memory_cross_lane_infos) {
        hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, false,
        "{\"type\":\"Feature\", \"properties\":{\"crossid\":%u, \"lane_idx\":%u, \"id\":%u, \"type\":%u, "
        "\"length\":%u, \"preid1\":%u, \"preid2\":%u, "
        "\"preid3\":%u, \"nextid1\":%u, \"nextid2\":%u, \"nextid3\":%u},",
        cross_lane_info.cross_id, cross_lane_info.lane_idx, cross_lane_info.id, cross_lane_info.type,
        cross_lane_info.length, cross_lane_info.pre_id_1,
        cross_lane_info.pre_id_2,
        cross_lane_info.pre_id_3, cross_lane_info.next_id_1, cross_lane_info.next_id_2, cross_lane_info.next_id_3);

        hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, false,
        "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (size_t i = 0; i < cross_lane_info.geometry.size(); i++) {
            if (i == 0) {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, false,
                "[%lf, %lf]", cross_lane_info.geometry[i].x, cross_lane_info.geometry[i].y);
            } else {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, false,
                ",[%lf, %lf]", cross_lane_info.geometry[i].x, cross_lane_info.geometry[i].y);
            }
        }

        if (cross_lane_cnt == cross_lane_size) {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, true,
            "]}} \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, true,
            "] \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, true,
            "} \n");
        } else {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(cross_lane_file, true,
            "]}},\n");
        }
        cross_lane_cnt++;
    }
    return true;

}

bool CDataToJson::MakeMemoryAllLines(const std::vector<Line>& memory_all_lines,
                                    const uint8_t& route_id) {
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_lane_mapping_all_lines = folder_path + "/lane_mapping_all_lines.geojson";


    const char* lane_mapping_all_lines = geojson_file_lane_mapping_all_lines.c_str();

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
    lane_mapping_all_lines, true,
    "{\n\"type\":\"FeatureCollection\",\n\"name\":\"lane_mapping_all_lines\",\n\"features\":[\n");

    if (memory_all_lines.empty()) {
        return true;
    }

    size_t cnt = 1;
    size_t size = memory_all_lines.size();
    for (const auto& line_info : memory_all_lines) {

        std::string traj_points_id_;
        for (const auto& pair : line_info.traj_line_ids) {
            traj_points_id_ += std::to_string(pair.first) + ":" + std::to_string(pair.second) + ",";
        }

        hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, false,
        "{\"type\":\"Feature\", \"properties\":{\"categroy\":%u, \"line_id\":%u, \"traj_points_id_\":\"%s\"},",
        line_info.categroy, line_info.id, traj_points_id_.c_str());

        hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, false,
        "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (size_t i = 0; i < line_info.points.size(); i++) {
            if (i == 0) {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, false,
                "[%lf, %lf]", line_info.points[i].x, line_info.points[i].y);
            } else {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, false,
                ",[%lf, %lf]", line_info.points[i].x, line_info.points[i].y);
            }
        }

        if (cnt == size) {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, true,
            "]}} \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, true,
            "] \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, true,
            "} \n");
        } else {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(lane_mapping_all_lines, true,
            "]}},\n");
        }
        cnt++;
    }
    return true;

}

bool CDataToJson::MakeMemoryCenterLine(std::vector<LinkDataCenterLine> link_data_center_line){
    uint8_t route_id = 1;
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_center_line = folder_path + "/memory_center_line.geojson";

    const char* center_line_file = geojson_file_center_line.c_str();
    std::cout << __FILE__ << __LINE__ << "center_line_file start: "  << std::endl;

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        center_line_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_center_line\",\n\"features\":[\n");
    
    if (link_data_center_line.empty()){
        return true;
    }

    size_t cnt = 1;
    size_t center_line_size = link_data_center_line.size();

    for(const auto& center_line : link_data_center_line){
        hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
        "{\"type\":\"Feature\", \"properties\":{\"id\":%u,\"linkid\":%u,\"maxx\":%u,\"minx\":%u,\"maxy\":%u,\"miny\":%u,\"LaneId\":%u},",
        center_line.link_data_info.RoadId, center_line.link_data_info.RoadKind,0,0,0,0,center_line.lane_connect_infos.LaneId);
        
        hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
        "\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[[");
        for(size_t i = 0; i < center_line.center_line_infos.size(); i++){
            if (i == 0){
                hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
                "[%lf, %lf]",center_line.center_line_infos[i].longitude, center_line.center_line_infos[i].latitude);
            }else{
                hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
                ",[%lf, %lf]",center_line.center_line_infos[i].longitude, center_line.center_line_infos[i].latitude);
            }

        }
        if (cnt == center_line_size) {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, true,
            "]]}} \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, true,
            "] \n");
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, true,
            "} \n");
        } else {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, true,
            "]]}},\n");
        }
        cnt++;

    }




    return true;
}

bool CDataToJson::MakeMemoryPosition(const std::vector<PositionPointInfo>& position_infos,
                                    const uint8_t& route_id) {
    // std::string geojson_file_lane_mapping_position;
        // 根据运行环境配置默认日志路径
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cout << "Error creating directory: " << folder_path << std::endl;
        LOGE("Error creating directory: %s", folder_path.c_str());
        return false;
    }

    // 构建文件路径
    std::string geojson_file_memory_position = folder_path + "/position_point_map.geojson";



    const char* position_file = geojson_file_memory_position.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        position_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_linklane\",\n\"features\":[\n");

    if (position_infos.empty()) {
        return true;
    }

    // size_t cnt = 1;
    // size_t position_size = position_infos.size();
    for (const auto& position_info : position_infos) {
        hdm_utility::CHdmDbgLog::GetInstance()->Write(position_file, true,
        "{\"type\":\"Feature\", \"properties\":{\"id\":%llu, \"path_offset\":%lu, \"changepoint_offset\":%lu, "
        "\"changepoint_type\":%u, \"changepoint_dir\":%u,  \"lane_id\":%u, \"speed\":%u, \"heading\":%f, "
        "\"cross_point\":%u, \"left_lane_type\":%u, \"right_lane_type\":%u, \"to_cross_point_dir\":%u, "
        "\"to_cross_point_times\":%u}, "
        "\"geometry\":{\"type\":\"Point\", \"coordinates\":[%lf,  %lf]}}, \n",
        position_info.id, position_info.path_offset / 100, position_info.changepoint_offset,
        position_info.changepoint_type, position_info.changepoint_dir, position_info.line_id,
        position_info.speed, position_info.heading,
        position_info.cross_point, position_info.left_lane_type, position_info.right_lane_type,
        position_info.to_cross_point_dir, position_info.to_cross_point_times,
        position_info.position.x, position_info.position.y);
        // cnt ++;

    }

    return true;

}

bool CDataToJson::MakeMemoryPositionCsv(const std::vector<PositionInfo>& position_infos){
    uint8_t route_id = 1;
    // std::string geojson_file_lane_mapping_position;
        // 根据运行环境配置默认日志路径
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_memory_position = folder_path + "/memory_position_csv.geojson";

    const char* position_file = geojson_file_memory_position.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        position_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_linklane\",\n\"features\":[\n");

    if (position_infos.empty()) {
        return true;
    }

    size_t cnt = 1;
    size_t position_size = position_infos.size();
    for(const auto& position_info : position_infos){
        hdm_utility::CHdmDbgLog::GetInstance()->Write(
        position_file, true,
        "{\"type\":\"Feature\", \"properties\":{\
\"id\":\"%llu\",\"path_offset\":\"%lu\",\"changepoint_offset\":\"%lu\",\"changepoint_type\":\"%u\",\"changepoint_dir\":\"%u\",\
\"lane_id\":\"%u\",\"speed\":\"%u\",\"heading\":\"%f\" \
 },\"geometry\":{\"type\":\"Point\",\"coordinates\":[%lf,  %lf]}},\n",
        0, position_info.path_offset, 0,
        0, 0,
        position_info.lane_id,0,
        position_info.heading,position_info.longitude, position_info.latitude);
        cnt ++;

    }

    return true;

}

bool CDataToJson::MakeLaneMappingPosition(const std::vector<LaneMappingMsg>& lane_mapping_infos,
                                        const uint8_t& route_id){
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_lane_mapping_position = folder_path + "/lane_mapping_position.geojson";

    const char* position_file = geojson_file_lane_mapping_position.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        position_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_linklane\",\n\"features\":[\n");

    if (lane_mapping_infos.empty()){
        return true;
    }

    size_t cnt = 1;
    size_t lane_mapping_infos_size = lane_mapping_infos.size();
    for(const auto& lane_mapping_info : lane_mapping_infos){
        for(const auto& point_info :lane_mapping_info.TrajHistory){
            std::string lane_type_char;
            size_t lane_type_list_size = point_info.LaneType.size();
            for (size_t i = 0; i <lane_type_list_size; i++) {
                uint8_t lnen_type = point_info.LaneType[i];
                lane_type_char += std::to_string(lnen_type);
                if (i < (lane_type_list_size - 1)) {
                    lane_type_char += ",";
                }
            }

            std::string relate_line_id_char;
            size_t relate_line_id_list_size = point_info.RelateLineID.size();
            for (size_t i = 0; i <relate_line_id_list_size; i++) {
                uint8_t relate_line_id = point_info.RelateLineID[i];
                relate_line_id_char += std::to_string(relate_line_id);
                if (i < (relate_line_id_list_size - 1)) {
                    relate_line_id_char += ",";
                }
            }

            hdm_utility::CHdmDbgLog::GetInstance()->Write(
            position_file, true,
            "{\"type\":\"Feature\", \"properties\":{\
\"LaneNumber\":\"%u\",\"EgoInLane\":\"%u\",\"RoadType\":\"%u\",\"SpeedLimitMax\":\"%u\",\"PosTime\":\"%llu\",\"PositionID\":\"%llu\",\
\"StopLineOffset\":\"%lf\",\"Heading\":\"%f\",\"lane_type_char\":\"%s\",\"relate_line_id_char\":\"%s\" \
},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.8lf,  %.8lf]}},\n",
            point_info.LaneNumber, point_info.EgoInLane,point_info.RoadType,point_info.SpeedLimitMax,point_info.PosTime ,point_info.PositionID,
            point_info.StopLineOffset,point_info.Heading,lane_type_char.c_str(),relate_line_id_char.c_str(),
            point_info.PosLgt, point_info.PosLat);
            cnt ++;

        }
    }

    return true;
}

bool CDataToJson::MakeLaneMappingLine(const std::vector<LaneMappingMsg>& lane_mapping_infos,
                                    const uint8_t& route_id) {
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_mapping_line = folder_path + "/lane_mapping_line.geojson";


    const char* center_line_file = geojson_file_mapping_line.c_str();
    // std::cout << __FILE__ << __LINE__ << "center_line_file start: "  << std::endl;

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        center_line_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_center_line\",\n\"features\":[\n");
    
    if (lane_mapping_infos.empty()){
        return true;
    }

    size_t cnt = 1;
    // size_t center_line_size = lane_mapping_infos.size();

    for(const auto& lane_mapping_info : lane_mapping_infos){
        for(const auto& lane_data : lane_mapping_info.LaneData){
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
            "{\"type\":\"Feature\", \"properties\":{\"RoadId\":%lu,\"LaneId\":%u,\"maxx\":%u,\"minx\":%u,\"maxy\":%u,\"miny\":%u,\"LaneId1\":%u},",
            lane_mapping_info.RoadId, lane_data.first,0,0,0,0,0);
            
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
            "\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[[");
            size_t line_points_size = lane_data.second.size();
            for(size_t i = 0; i < line_points_size; i++){
                if (i == 0){
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
                    "[%lf, %lf]",lane_data.second[i].X, lane_data.second[i].Y);
                }else{
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, false,
                    ",[%lf, %lf]",lane_data.second[i].X, lane_data.second[i].Y);
                }

            }
            hdm_utility::CHdmDbgLog::GetInstance()->Write(center_line_file, true,
            "]]}},\n");
            }
    }


    return true;
}


bool CDataToJson::MakeMemoryLinkLane(std::vector<MemoryLink> memory_links_data) {
    uint8_t route_id = 1;
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_switch_global = folder_path + "/memory_linklane.geojson";

    const char* linklane_file = geojson_file_switch_global.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        linklane_file, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memoy_linklane\",\n\"features\":[\n");

    if (memory_links_data.empty()){
        return true;
    }

    std::unordered_map<uint32_t, std::vector<uint32_t>> link_id_index_map; 
    for (uint32_t i = 0; i < memory_links_data.size(); i++){
        if (link_id_index_map.find(memory_links_data[i].link_id) == link_id_index_map.end()){
            link_id_index_map[memory_links_data[i].link_id] = {i};
        }else{
            link_id_index_map[memory_links_data[i].link_id].push_back(i);
        }
    }

    uint32_t linklane_cnt = 1;
    for (auto map_iter : link_id_index_map){
        auto link_id = map_iter.first;
        auto idxs    = map_iter.second;
        MemoryLink memory_link;

        uint32_t next_link_id = 0;
        std::vector<uint32_t> next_idxs = {};
        std::unordered_map<uint16_t, uint16_t> lane_id_maping;
        MemoryLink next_memory_link;
        //get next link
        for (auto idx :idxs){
            if (1 == memory_links_data[idx].is_memory){
                memory_link = memory_links_data[idx];
                next_link_id = memory_link.link_lane_info1.link_id;
                if (next_link_id > 0){
                    next_idxs = link_id_index_map[next_link_id];
                }
            }
            lane_id_maping[memory_links_data[idx].s_laneid] = memory_links_data[idx].link_lane_info1.lane_num;
        }

        for (auto next_idx :next_idxs){
            if (1 == memory_links_data[next_idx].is_memory){
                next_memory_link = memory_links_data[next_idx];
            }
        }

        //
        auto lane_number = memory_links_data[idxs[0]].lane_num;

        if (0 == next_link_id){
            for (uint8_t lane_id = 1; lane_id < lane_number + 1; lane_id++){
                hdm_utility::CHdmDbgLog::GetInstance()->Write(
                    linklane_file, true,
                    "{\"type\":\"Feature\", \"properties\":{\
\"id\":%u,\"linkid\":%u,\"laneid\":%u,\"rlength\":%u,\"nlane\":%u,\
\"nlink\":%u },\"geometry\": null},\n",
                    linklane_cnt, link_id, lane_id, 100000, 0, 0);
                linklane_cnt ++;
            }
            continue; 
        }

        auto next_lane_number = memory_links_data[next_idxs[0]].lane_num;

        for (uint8_t lane_id = 1; lane_id < lane_number + 1; lane_id++){
            uint32_t temp_next_link_id = 0;
            uint16_t temp_next_lane_id = 0;
            if (lane_id_maping.find(lane_id) != lane_id_maping.end()){
                temp_next_lane_id = lane_id_maping[lane_id];
                if (temp_next_lane_id > 0){
                    temp_next_link_id = next_link_id;
                }
            }else{
                int32_t temp_setp = static_cast<int32_t>(lane_id) - static_cast<int32_t>(memory_link.s_laneid);
                if (memory_link.split_merge == 5 && temp_setp > 0){
                    temp_setp -= 1;
                }else if (memory_link.split_merge == 6 && temp_setp < 0){
                    temp_setp += 1;
                }else if (next_memory_link.split_merge == 1 && temp_setp < 0){
                    temp_setp -= 1;
                }else if (next_memory_link.split_merge == 2 && temp_setp > 0){
                    temp_setp += 1;
                }

                int32_t next_lane_id = temp_setp + static_cast<int32_t>(lane_id_maping[memory_link.s_laneid]);
                if (next_lane_id >=1 && next_lane_id <= next_lane_number){
                    temp_next_link_id = next_link_id;
                    temp_next_lane_id = next_lane_id;
                }
            }
            
            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                linklane_file, true,
                "{\"type\":\"Feature\", \"properties\":{\
\"id\":%u,\"linkid\":%u,\"laneid\":%u,\"rlength\":%u,\"nlane\":%u,\
\"nlink\":%u },\"geometry\": null},\n",
                linklane_cnt, link_id, lane_id, 100000, temp_next_lane_id, temp_next_link_id);
            linklane_cnt ++;
        }
    }


    return true;
}

bool CDataToJson::DownTrajHistoryToGeoJson(const LaneMappingMsg& lane_mapping_msg, std::string file_name, int& write_times) {
        std::cout << __FILE__ << "," << __LINE__ << ","
              << " DownTrajHistoryToGeoJson"  << std::endl;
    const char* file_name_char_traj_hist;
    std::string geojson_file_traj_hist;
    geojson_file_traj_hist = std::to_string(0) + file_name;
    file_name_char_traj_hist = geojson_file_traj_hist.c_str();
    if(write_times == 0){
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_traj_hist, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"traj_hist_json\",\n\"features\":[\n");

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_traj_hist, true,
        "{\"type\":\"Feature\", \"properties\":{\
\"link_id\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",lane_mapping_msg.RoadId);

    for (size_t point_idx = 0; point_idx < lane_mapping_msg.TrajHistory.size(); point_idx++) {
        if (0 == point_idx) {
            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                file_name_char_traj_hist, false, " [ %lf,  %lf]",
                                    lane_mapping_msg.TrajHistory[point_idx].PosLgt,
                                    lane_mapping_msg.TrajHistory[point_idx].PosLat);  
        }else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_traj_hist, false, " ,[ %lf,  %lf]",
                                     lane_mapping_msg.TrajHistory[point_idx].PosLgt,
                              lane_mapping_msg.TrajHistory[point_idx].PosLat); 
                        }
    }
    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_traj_hist, true, "]}},\n");  
    }

    write_times++;
    return true;
}

bool CDataToJson::DownLinesToGeoJson(const LaneMappingMsg& lane_mapping_msg, std::string file_name, int& write_times) {
        std::cout << __FILE__ << "," << __LINE__ << ","
              << " DownLinesToGeoJson"  << std::endl;
    const char* file_name_char_line;
    std::string geojson_file_line;
    geojson_file_line = std::to_string(0) + file_name;
    file_name_char_line = geojson_file_line.c_str();
    if(write_times == 0){
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_line, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"traj_hist_json\",\n\"features\":[\n");
    }
       
    for(int lane_index = 0;lane_index<lane_mapping_msg.LaneData.size();lane_index++){
            hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_line, true,
        "{\"type\":\"Feature\", \"properties\":{\
\"link_id\":\"%u\",\"line_id\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",lane_mapping_msg.RoadId,lane_index);
        for (size_t point_idx = 0; point_idx < lane_mapping_msg.LaneData.at(lane_index).size(); point_idx++) {
            if (0 == point_idx) {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(
                     file_name_char_line, false, " [ %lf,  %lf]",
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).X,
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).Y);  
            }else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_line, false, " ,[ %lf,  %lf]",
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).X,
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).Y); 
            }
        }
    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_line, true, "]}},\n");          
    


    }

    write_times++;
    return true;
}

bool CDataToJson::DownTrajToGeoJson(const LaneMappingMemory& lane_mapping_msg, std::string file_name, int& write_times) {
        std::cout << __FILE__ << "," << __LINE__ << ","
              << " DownLinesToGeoJson"  << std::endl;
    const char* file_name_char_line;
    std::string geojson_file_line;
    geojson_file_line = std::to_string(0) + file_name;
    file_name_char_line = geojson_file_line.c_str();
    if(write_times == 0){
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_line, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"traj_hist_json\",\n\"features\":[\n");
    }
       
    for (const auto& position_info : lane_mapping_msg.TrajHistory) {
        hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_line, true,
        "{\"type\":\"Feature\", \"properties\":{\"LaneNumber\":%u, \"EgoInLane\":%u, \"RoadType\":%u, "
        "\"SpeedLimitMax\":%u, \"PositionID\":%llu, \"Heading\":%f, \"is_split\":%u}, "
        "\"geometry\":{\"type\":\"Point\", \"coordinates\":[%lf,  %lf]}}, \n",
        position_info.LaneNumber, position_info.EgoInLane , position_info.RoadType,
        position_info.SpeedLimitMax, position_info.PositionID, position_info.Heading,
        position_info.is_split, position_info.PosLgt, position_info.PosLat);
        // cnt ++;

    }        
    

    write_times++;
    return true;
}

bool CDataToJson::DownLinesToGeoJson(const LaneMappingMemory& lane_mapping_msg, std::string file_name, int& write_times) {
        std::cout << __FILE__ << "," << __LINE__ << ","
              << " DownLinesToGeoJson"  << std::endl;
    const char* file_name_char_line;
    std::string geojson_file_line;
    geojson_file_line = std::to_string(0) + file_name;
    file_name_char_line = geojson_file_line.c_str();
    if(write_times == 0){
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_line, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"traj_hist_json\",\n\"features\":[\n");
    }
    for(int lane_index = 0;lane_index<lane_mapping_msg.LaneData.size();lane_index++){
            hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_line, true,
        "{\"type\":\"Feature\", \"properties\":{\
\"link_id\":\"%u\",\"line_id\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",lane_mapping_msg.LinkId,lane_index);
        for (size_t point_idx = 0; point_idx < lane_mapping_msg.LaneData.at(lane_index).size(); point_idx++) {
            if (0 == point_idx) {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(
                     file_name_char_line, false, " [ %lf,  %lf]",
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).X,
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).Y);  
            }else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_line, false, " ,[ %lf,  %lf]",
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).X,
                                    lane_mapping_msg.LaneData.at(lane_index).at(point_idx).Y); 
            }
        }
    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_line, true, "]}},\n");          
    


    }

    write_times++;
    return true;
}

bool CDataToJson::MakeMemoryBoundaryLine(std::vector<int64_t> link_list, std::vector<LaneMappingMsg> lane_mapping_infos){
    
    uint8_t route_id = 1;
    #ifdef __QNX__
        std::string default_log_path = "/userdata/tmp/";
    #else
        std::string default_log_path = "./";
    #endif

    // 构建文件夹路径
    std::string folder_path = default_log_path + "memory" + std::to_string(route_id);
    if (mkdir(folder_path.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Error creating directory: " << folder_path << std::endl;
        return false;
    }

    // 构建文件路径
    std::string geojson_file_memory_boundary_line = folder_path + "/memory_boundary_line.geojson";

    // hdm_utility::CHdmDbgLog::GetInstance()->Write(
    //     memory_boundary_line, true,
    //     "{\n\"type\":\"FeatureCollection\",\n\"name\":\"memory_boundary_line\",\n\"features\":[\n");

    // if (link_list.empty() || lane_mapping_infos.empty()){
    //     return true;
    // }

    // // lane_mapping_infos_[0].LaneDatas[0].LineDatas
    
    // for(const auto& lane_mapping_info : lane_mapping_infos){
    //     std::cout << __FILE__ << __LINE__ << "lane_mapping_info.LinkId: " << lane_mapping_info.LinkId << std::endl;
    //     if(std::find(link_list.begin(), link_list.end(), lane_mapping_info.LinkId) != link_list.end()){
    //         for(size_t i = 0; i < lane_mapping_info.LaneDatas.size(); i++){
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true,
    //             "{\"type\":\"Feature\", \"properties\":{\"linkid\":%u,\"ismatch\":%u,\"lineidx\":%u,\"minx\":%u,\"maxy\":%u,\"miny\":%u,\"heading\":%f},",
    //             lane_mapping_info.LinkId,1, i,0,0,0,0.0);

    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true,
    //             "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    //             for (size_t point_idx = 0; point_idx < lane_mapping_info.LaneDatas[i].LineDatas.size(); point_idx++) {
    //                     if (0 == point_idx) {
    //                         hdm_utility::CHdmDbgLog::GetInstance()->Write(
    //                             memory_boundary_line, true, " [ %lf,  %lf]",lane_mapping_info.LaneDatas[i].LineDatas[point_idx].X,
    //                             lane_mapping_info.LaneDatas[i].LineDatas[point_idx].Y);
    //                     } else {
    //                         hdm_utility::CHdmDbgLog::GetInstance()->Write(
    //                             memory_boundary_line, true, " ,[ %lf,  %lf]",lane_mapping_info.LaneDatas[i].LineDatas[point_idx].X,
    //                             lane_mapping_info.LaneDatas[i].LineDatas[point_idx].Y);
    //                     }
    //             }
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true, "]}}");
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true, ",\n");
    //         }
    //     }else{
    //         for(size_t i = 0; i < lane_mapping_info.LaneDatas.size(); i++){
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true,
    //             "{\"type\":\"Feature\", \"properties\":{\"linkid\":%u,\"ismatch\":%u,\"lineidx\":%u,\"minx\":%u,\"maxy\":%u,\"miny\":%u,\"heading\":%f},",
    //             lane_mapping_info.LinkId,0, i,0,0,0,0.0);

    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true,
    //             "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
    //             for (size_t point_idx = 0; point_idx < lane_mapping_info.LaneDatas[i].LineDatas.size(); point_idx++) {
    //                     if (0 == point_idx) {
    //                         hdm_utility::CHdmDbgLog::GetInstance()->Write(
    //                             memory_boundary_line, false, " [ %lf,  %lf]",lane_mapping_info.LaneDatas[i].LineDatas[point_idx].X,
    //                             lane_mapping_info.LaneDatas[i].LineDatas[point_idx].Y);
    //                     } else {
    //                         hdm_utility::CHdmDbgLog::GetInstance()->Write(
    //                             memory_boundary_line, false, " ,[ %lf,  %lf]",lane_mapping_info.LaneDatas[i].LineDatas[point_idx].X,
    //                             lane_mapping_info.LaneDatas[i].LineDatas[point_idx].Y);
    //                     }
    //             }
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true, "]}}");
    //             hdm_utility::CHdmDbgLog::GetInstance()->Write(memory_boundary_line, true, ",\n");
    //         }  
    //     }
    // }


    return true;
}




}  
}  
