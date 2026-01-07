#include "MakeInputData.h"
#include <string>

// #define PRO_DEAL

std::string DATA_FILE = "/home/ecarx/zbag/logs/";

namespace CommuteMode {
namespace MapFusion {
MakeInputStasticVar MakeInputData::make_input_stastic_var_;

bool MakeInputData::Execute(const std::vector<LaneMappingMsg>& lane_mapping_infos,
                    const std::vector<Line>& line_out, const uint8_t& route_id) {
    //  init variable
    //   ReadConfig();
    //  判断数据是否开始进行后处理
    lane_mapping_infos_ = lane_mapping_infos;
    line_out_ = line_out;

// 记录所有轨迹点的原始positionid及对应的位置信息
    for (const auto& lane_mapping_info : lane_mapping_infos) {
        if (false == lane_mapping_info.TrajHistory.empty()) {
            for (const auto& position : lane_mapping_info.TrajHistory) {
                DoublePosePoint pose_point{};
                pose_point.x = position.PosLgt;
                pose_point.y = position.PosLat;
                pose_point.yaw = position.Heading;
                traj_points_position_map_.emplace(position.PositionID, pose_point);
          }
        }
    }
    // static int lane_mapping_write_times = 0;
    // if (lane_mapping_write_times == 0) {
        std::cout << "******** raw data load  start ***********" << std::endl;
        CDataToJson data_to_json;
        data_to_json.MakeMemoryAllLines(line_out, route_id);
        data_to_json.MakeLaneMappingPosition(lane_mapping_infos_, route_id);
        data_to_json.MakeLaneMappingLine(lane_mapping_infos_, route_id);
        // lane_mapping_write_times++;
        std::cout << "******** raw data load end ***********" << std::endl;
    // }

    // if (false == MakeLinkList()){
    //     // return false;
    // }

    if (false == map_hdnv_infos_.empty() && make_input_stastic_var_.json_write_times == 0) {
        CDataToJson data_to_json;
        data_to_json.MakeMemoryBoundaryLine(map_hdnv_infos_[0].linkid_list, lane_mapping_infos_);
        make_input_stastic_var_.json_write_times++;
    }

    // process
    if (false == ModityOrgData()) {
        return false;
    }

    if (false == ModitySdData()){
        return false;
    }

    if (false == CombineData()){
        return false;
    }

    // if (false == MakeIndex(intersection_infos_, link_data_infos_, lane_mapping_infos_)) {
    //     return false;
    // }

    return true;
}

bool MakeInputData::MakeLinkList() {
    link_list_.clear();
    if (map_hdnv_infos_.empty()){
        return false;
    }

    uint32_t rout_id = 0;
    for (auto& hdnv : map_hdnv_infos_){
        if (0 == hdnv.route_id){
            //invalid
            continue;
        }

        if (0 == rout_id || rout_id != hdnv.route_id){
            //clear 
            link_list_.clear();
        }
        int start_idx = 0;
        if (hdnv.linkid_list.size() > 0){
            rout_id = hdnv.route_id;
            std::vector<uint32_t>::iterator iter = std::find(link_list_.begin(), link_list_.end(), hdnv.linkid_list[start_idx]);
            while (iter != link_list_.end() && start_idx < hdnv.linkid_list.size()){
                if (*iter != hdnv.linkid_list[start_idx]){
                    start_idx = 0;
                    break;
                }
                iter += 1;
                start_idx += 1;
            }
        }
        link_list_.insert(link_list_.end(), hdnv.linkid_list.begin() + start_idx, hdnv.linkid_list.end());
    }

  // //get MAX link_id
  // max_link_id_ = *(std::max_element(link_list_.begin(), link_list_.end()));
  // uint32_t temp_link_id = *(std::max_element(link_data_infos_.begin(), link_data_infos_.end(), cmp));
  // max_link_id_ = std::max(max_link_id_, temp_link_id);

  // if (fasle == MakeNewLinkList()){
  //   return false;
  // }

  return true;
}

bool MakeInputData::SaveData() {
    std::string position_str = DATA_FILE + "position.csv";
    // ReadPositionCsv(position_str.c_str(), position_infos_);

    // std::cout << "position_infos.size(): " << position_infos_.size() << std::endl;
    //   std::cout << "position_infos[0].longitude:{ " ;
    //   for(const auto& position_info : position_infos_){
    //       std::cout << position_info.longitude << ", ";
    //   }
    //   std::cout << "}" << std::endl;

    std::string intersection_str = DATA_FILE + "intersection.csv";
    // ReadIntersectionCsv(intersection_str.c_str(), intersection_infos_);

    // std::cout << __FILE__ << __LINE__ << " " << "intersection_infos.size(): " << intersection_infos_.size() << std::endl;
    // if(intersection_infos_.size() > 1){
    //     std::cout << __FILE__ << __LINE__ << "intersection_infos[0].InnerRoadsIds{";
    //     for(const auto& id : intersection_infos_[0].InnerRoadsIds){
    //         std::cout << id << ", ";
    //     }
    //     std::cout << "}" << std::endl;
    // }

    std::string link_data_str = DATA_FILE + "link_data.csv";
    // ReadLinkDataCsv(link_data_str.c_str(), center_line_data_infos_);
    // if (false == center_line_data_infos_.empty() &&
    //     make_input_stastic_var_.json_write_memory_center_line_times == 0) {
    //     CDataToJson data_to_json;
    //     data_to_json.MakeMemoryCenterLine(center_line_data_infos_);
    //     make_input_stastic_var_.json_write_memory_center_line_times++;
    // }


    // std::cout << __FILE__ << __LINE__ << " " << "link_data_infos.size(): " << link_data_infos_.size() << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_data_infos[0].PrevIds{ ";
    // for(const auto& id : link_data_infos_[0].PrevIds){
    //     std::cout << id << ", ";
    // }
    // std::cout << "}" << std::endl;

    // std::cout << __FILE__ << __LINE__ << "link_data_infos[27].PrevIds{ ";
    // for(const auto& id : link_data_infos_[1].PrevIds){
    //     std::cout << id << ", ";
    // }
    // std::cout << "}" << std::endl;

    // std::cout << __FILE__ << __LINE__ << "link_data_infos[0].lanes{ ";
    // // for(const auto& lane : link_data_infos_[0].Lanes){
    // auto link_data_info = link_data_infos_[0];
    // std::cout << "road_id: " << link_data_info.RoadId << ", lane_id: " << link_data_info.Lanes[0].LaneId  << ", lanenext_RoadId_id: " << link_data_info.Lanes[0].Nexts[0].RoadId 
    //     << ", lanenext_LaneNum: " << int(link_data_info.Lanes[0].Nexts[0].LaneNum) << ", lane_pre_size(): " << link_data_info.Lanes[0].Pres.size() ;  
    // // } 
    // std::cout << "}" << std::endl;


    // std::cout << __FILE__ << __LINE__ << "link_data_infos[0].LaneConnects.size():  " << link_data_infos_[0].LaneConnects.size() << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_data_infos[0].RoadId" << link_data_infos_[0].RoadId << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_data_infos[1].RoadId" << link_data_infos_[1].RoadId << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_data_infos[0].LaneConnects{ " << std::endl;
    // for(const auto& LaneConnectsInfo : link_data_infos_[0].LaneConnects){
    //     std::cout << "LaneId: " << LaneConnectsInfo.LaneId << ", RoadId: " << LaneConnectsInfo.RoadId << ", PassRoadIds.size(): " << LaneConnectsInfo.PassRoadIds.size() << ", ArrowInfo: " << int(LaneConnectsInfo.ArrowInfo.data);
    //     std::cout << " ,LaneInfo: " << LaneConnectsInfo.LaneInfo << ", BusLane: " << LaneConnectsInfo.BusLane << ", LaneNum: " << LaneConnectsInfo.LaneNum << ", LaneLeft: " << LaneConnectsInfo.LaneLeft << std::endl;
    // }
    // std::cout << "}" << std::endl;

    // std::cout << __FILE__ << __LINE__ << "link_data_infos[1].LaneConnects{ " << std::endl;
    // for(const auto& LaneConnectsInfo : link_data_infos_[1].LaneConnects){
    //     std::cout << "LaneId: " << LaneConnectsInfo.LaneId << ", RoadId: " << LaneConnectsInfo.RoadId << ", PassRoadIds.size(): " << LaneConnectsInfo.PassRoadIds.size() << ", ArrowInfo: " << int(LaneConnectsInfo.ArrowInfo.data);
    //     std::cout << " ,LaneInfo: " << LaneConnectsInfo.LaneInfo << ", BusLane: " << LaneConnectsInfo.BusLane << ", LaneNum: " << LaneConnectsInfo.LaneNum << ", LaneLeft: " << LaneConnectsInfo.LaneLeft << std::endl;
    // }
    // std::cout << "}" << std::endl;


    std::string hdnv_str = DATA_FILE + "hdnv.csv";
    // ReadMapHdnvInfoCsv(hdnv_str.c_str(), map_hdnv_infos_);

    // std::cout << __FILE__ << __LINE__ << " " << "map_hdnv_infos.size(): " << map_hdnv_infos_.size() << std::endl;
    // std::cout << __FILE__  << __LINE__ << "hdnv_infos[0].linkid_list{ "; 
    // for(const auto& linkid : map_hdnv_infos_[0].linkid_list){
    //     std::cout << linkid << ", ";
    // }
    // std::cout << "}" << std::endl;

    return true;
}

bool MakeInputData::LocalDataOK() {
    std::string map_str = DATA_FILE + "map.json";
    // ReadLaneMappingInfoJson(map_str.c_str(), lane_mapping_infos_);

    //   std::cout << "lane_mapping_infos.size(): " << lane_mapping_infos_.size() << std::endl;
    //   if(lane_mapping_infos_.size() > 1){
    //       std::cout << __FILE__ << __LINE__ << "lane_mapping_infos[0].PosTime{" ;
    //       for(const auto& pos_time : lane_mapping_infos_[0].PosTime){
    //           std::cout << pos_time << ", ";
    //       }
    //       std::cout << "}" << std::endl;

    // std::cout << __FILE__ << __LINE__ << "lane_mapping_infos[0].lane_datas.line_points{" ;
    // std::cout << std::fixed << std::setprecision(14); 
    // for(const auto& point : lane_mapping_infos_[0].LaneDatas[0].LineDatas){
    //     std::cout << "(" << point.X << ", " << point.Y << "), ";
    // }
    // std::cout << "}" << std::endl;

    // std::cout << __FILE__ << __LINE__ << "points:{ " << std::endl;
    // for(const auto& link_lane : lane_mapping_infos_){
    //     if(link_lane.LinkId == 107941049){
    //        for(int i = 0 ; i < link_lane.LaneDatas.size(); i++){
    //         if(i == 9){
    //             for(const auto& point : link_lane.LaneDatas[i].LineDatas){
    //                std::cout << "(" << point.X << ", " << point.Y << "), ";
    //            }
    //            std::cout << "}" << std::endl;

    //         }
    //        }
    //     }
    // }

    // std::cout << __FILE__ << __LINE__ << "points:{ " << std::endl;
    // for(const auto& link_lane : lane_mapping_infos_){
    //     if(link_lane.LinkId == 107941049){
    //        for(int i = 0 ; i < link_lane.LaneDatas.size(); i++){
    //         if(i == 12){
    //             for(const auto& point : link_lane.LaneDatas[i].LineDatas){
    //                std::cout << "(" << point.X << ", " << point.Y << "), ";
    //            }
    //            std::cout << "}" << std::endl;

    //         }
    //        }
    //     }
    // }

    //       std::cout << __FILE__ << __LINE__ << "lane_mapping_infos[0].traj_points{" ;
    //       for(const auto& point : lane_mapping_infos_[0].TrajHistory.Positions){
    //           std::cout << "(" << point.PosLgt << ", " << point.PosLat << "), ";
    //       }
    //       std::cout << "}" << std::endl;
    //   }
    return true;
}

bool MakeInputData::ModityOrgData() { 
  //step 1: 将lane排序，从右到左
  //step 2: 给每个link首尾都沿最短平切
  //step 4:第一个link尾部和第二个link的首部都延伸，求交点
  //step 5: 交点就是最终切好的点
  ////0218,修改代码，1. 只有贴近路口的线才切； 2. 最多只保留4车道线；3.临临车道线存在豁口的，需要连起来

// #define PRO_DEAL
#define PRO_DEAL_DUMP
  for(int i =0; i<lane_mapping_infos_.size();i++){
    LaneMappingMemory laneInfo_tmp{};
    laneInfo_tmp.LinkId = lane_mapping_infos_[i].RoadId;
    for(auto lane:lane_mapping_infos_[i].LaneData){
        laneInfo_tmp.LaneData.push_back(lane.second);
    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << ":" << __LINE__ << ": " << "sort lane: " <<i<< std::endl;
    std::cout << __FILE__ << ":" << __LINE__ << ": " << "laneInfo_tmp.LaneData.size(): " <<laneInfo_tmp.LaneData.size()<< std::endl;
#endif
    laneInfo_tmp.TrajHistory = lane_mapping_infos_[i].TrajHistory;
    if(laneInfo_tmp.LaneData.size()>0){
        //    lane排序，从右到左排
        SortLane(laneInfo_tmp);
    }
    //没线的直接填充轨迹线
    lane_mapping_memories_.push_back(laneInfo_tmp);
    hq_raw_data_map_.lane_mapping_memory_link_map.emplace(laneInfo_tmp.LinkId, i);
#ifdef PRO_DEAL
    std::cout << __FILE__ << ":" << __LINE__ << ": " << "sort lane: " <<i<<" ,end"<< std::endl;
#endif
  }
#ifdef PRO_DEAL_DUMP
  static int write_times5= 0;
  for(auto lane :lane_mapping_memories_){
        CDataToJson::DownLinesToGeoJson(lane,"_sort_line1.geojson",write_times5);
  }
#endif
  //只取自车的3车道4条线
  //step 2: 给每个link首尾都沿最短平切
  int section_num = 0;
  for(int i = 0;i<lane_mapping_memories_.size();i++){
#ifdef PRO_DEAL
    std::cout << __FILE__ << ":" << __LINE__ << ": " << "cut lane: " <<i<<" ,start"<< std::endl;
#endif
    if(lane_mapping_memories_[i].LaneData.size()>0){
      std::vector<std::vector<Point2Dd>> cut_lane_lines_xy{};
      DoublePosePoint ori_point;
      int base_lane_index = 0;
      PickLane(lane_mapping_memories_[i],section_num);
      CutLane(lane_mapping_memories_[i],section_num,cut_lane_lines_xy,ori_point,base_lane_index);
      FixLane(cut_lane_lines_xy,ori_point,base_lane_index,lane_mapping_memories_[i]);        
      section_num++;
    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << ":" << __LINE__ << ": " << "cut lane: " <<i<<" ,end"<< std::endl;
#endif
  }
#ifdef PRO_DEAL_DUMP
  static int write_times6= 0;
  for(auto lane :lane_mapping_memories_){
      CDataToJson::DownLinesToGeoJson(lane,"_cut_line1.geojson",write_times6);
  }
//   static int write_times7= 0;
//   CDataToJson::DownLinesToGeoJson(lane_mapping_memories_[1],"_cut_line2.geojson",write_times7);
#endif

  std::map<uint64_t, std::vector<uint64_t>> wide_lane_position_id_map{};
  WideLaneJudge(line_out_, lane_mapping_memories_, wide_lane_position_id_map);
  OutputWideLane(wide_lane_position_id_map, lane_mapping_memories_);
#ifdef PRO_DEAL
  static int write_times7= 0;
  for(auto lane :lane_mapping_memories_){
      CDataToJson::DownTrajToGeoJson(lane,"traj1.geojson",write_times7);
  }
//   static int write_times7= 0;
//   CDataToJson::DownLinesToGeoJson(lane_mapping_memories_[1],"_cut_line2.geojson",write_times7);
#endif
  return true;
}

bool MakeInputData::ModitySdData() {
  return true;
}

bool MakeInputData::CombineData() {
  return true;
}

bool MakeInputData::MakeIndex(const std::vector<Intersection>& intersection_infos,const std::vector<LinkMemory>& link_data_infos,const std::vector<LaneMappingMsg>& lane_mapping_msg) {
    //输入hq的intersection数据，构造map数据
    GenerateIntersectionDataMap(intersection_infos);

    //输入hq的linkdata数据，构map数据
    GenerateLinkDataMap(link_data_infos);

    //输入local map的LaneMappingMsg数据，构map数据
    GenerateLaneMappingMsgDataMap(lane_mapping_msg);    
  return true;
}

bool MakeInputData::ReadPositionCsv(const std::string& file_path, std::vector<PositionInfo>& position_infos){
    position_infos.clear();
    std::ifstream file(file_path);
    if(false == file.is_open()){
        // std::cout << __FILE__ << ":" << __LINE__ << ": " << "open file failed" << std::endl;
        return false;
    }

    std::string line;
    /*
    1：忽视第一行表头
    2：一个函数内std::getline每运行一次就读取一行csv
    */
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        PositionInfo info;

        /*
        1：忽视第一列
        2：std::getline(ss, token, ',')每运行一次就读取一列csv
        */
        std::getline(ss, token, ',');

        // Read the rest of the columns
        std::getline(ss, token, ','); info.longitude = std::stod(token);
        std::getline(ss, token, ','); info.latitude = std::stod(token);
        std::getline(ss, token, ','); info.heading = std::stof(token);
        std::getline(ss, token, ','); info.speed = std::stof(token);
        std::getline(ss, token, ','); info.xAcc = std::stof(token);
        std::getline(ss, token, ','); info.yAcc = std::stof(token);
        std::getline(ss, token, ','); info.zAcc = std::stof(token);
        std::getline(ss, token, ','); info.angular_velocity_x = std::stof(token);
        std::getline(ss, token, ','); info.angular_velocity_y = std::stof(token);
        std::getline(ss, token, ','); info.angular_velocity_z = std::stof(token);
        std::getline(ss, token, ','); info.fail_safe_loc_status = static_cast<uint8_t>(std::stoi(token));
        std::getline(ss, token, ','); info.geo_fence_judge_status = static_cast<uint8_t>(std::stoi(token));
        std::getline(ss, token, ','); info.geo_fence_judge_type = static_cast<uint8_t>(std::stoi(token));
        std::getline(ss, token, ','); info.path_id = std::stoul(token);
        std::getline(ss, token, ','); info.link_id = std::stoul(token);
        std::getline(ss, token, ','); info.lane_id = static_cast<uint8_t>(std::stoi(token));
        std::getline(ss, token, ','); info.path_offset = std::stoul(token);
        std::getline(ss, token, ','); info.timestamp = std::stoull(token);
        std::getline(ss, token, ','); info.position_age = std::stoull(token);

        position_infos.emplace_back(info);
    }
    file.close(); 
    return true;
}

bool MakeInputData::ReadMapHdnvInfoCsv(const std::string& file_path, std::vector<MapHdnvInfo>& map_hdnv_infos) {
    map_hdnv_infos.clear();

    std::ifstream file(file_path);
    if (!file.is_open()) {
        // std::cout << "Failed to open file: " << file_path << std::endl;
        return false;
    }

    std::string line;
    // 忽视第一行表头
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        MapHdnvInfo info;

        // 忽视第一列
        std::getline(ss, token, ',');

        // 解析 route_id
        std::getline(ss, token, ',');
        info.route_id = std::stoll(token);

        // 解析 linkid_list
        std::getline(ss, token, ',');
        std::stringstream linkid_ss(token);
        std::string linkid;
        while (std::getline(linkid_ss, linkid, '/')) {
            if (!linkid.empty()) {
                info.linkid_list.push_back(std::stoll(linkid));
            }
        }

        // 解析 nav_status
        std::getline(ss, token, ',');
        info.nav_status.data = static_cast<uint8_t>(std::stoi(token));

        // 解析 remain_distance
        std::getline(ss, token, ',');
        info.remain_distance = std::stoll(token);

        // 解析 UpdateType
        std::getline(ss, token, ',');
        info.UpdateType = static_cast<uint8_t>(std::stoi(token));

        map_hdnv_infos.emplace_back(info);
    }

    file.close();
    return true;
}

bool MakeInputData::ReadIntersectionCsv(const std::string& file_path, std::vector<Intersection>& intersection_infos){
    intersection_infos.clear();
    std::ifstream file(file_path);
    if(false == file.is_open()){
        // std::cout << __FILE__ << ":" << __LINE__ << ": " << "open file failed" << std::endl;
        return false;
    }

    std::string line;
    /*
    1：忽视第一行表头
    2：一个函数内std::getline每运行一次就读取一行csv
    */
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        Intersection info;

        std::getline(ss, token, ','); info.PathId = std::stoul(token);
        std::getline(ss, token, ','); info.StartOffset = std::stoul(token);
        std::getline(ss, token, ','); info.EndOffset = std::stoul(token);
        std::getline(ss, token, ','); info.IntersectionId = std::stoul(token);
        std::getline(ss, token, ','); info.Type.data = static_cast<uint8_t>(std::stoul(token));

        std::getline(ss, token, ',');
        std::stringstream inner_ss(token);
        std::string inner_token;
        while (std::getline(inner_ss, inner_token, '/')) {
            if (false == inner_token.empty()) {
                info.InnerRoadsIds.emplace_back(std::stoul(inner_token));
            }
        }

        std::getline(ss, token, ',');
        std::stringstream in_road(token);
        std::string in_road_token;
        while (std::getline(in_road, in_road_token, '/')) {
            if (false == in_road_token.empty()) {
                info.InRoadsIds.push_back(std::stoul(in_road_token));
            }
        }

        std::getline(ss, token, ',');
        std::stringstream out_ss(token);
        std::string out_token;
        while (std::getline(out_ss, out_token, '/')) {
            if (false == out_token.empty()) {
                info.OutRoadsIds.push_back(std::stoul(out_token));
            }
        }

        std::getline(ss, token, ',');
        std::stringstream points_ss(token);
        std::string point_token;
        while (std::getline(points_ss, point_token, ',')) {
            if (!point_token.empty()) {
                std::stringstream point_ss(point_token);
                double x, y;
                message::map_link_data::s_Point_t point;
                point_ss >> x >> y;
                point.Longitude = x;
                point.Latitude = y;
                info.Points.emplace_back(point);
            }
        }
        intersection_infos.push_back(info);
    }

    file.close();
    return true;
}

bool MakeInputData::ReadLinkDataCsv(const std::string& file_path, std::vector<LinkDataCenterLine>& link_data_infos){
    std::ifstream file(file_path);
    if(false == file.is_open()){
        // std::cout << __FILE__ << ":" << __LINE__ << ": " << "open file failed" << std::endl;
        return false;
    }
    link_data_infos.clear();
    // 使用 std::unordered_map 来存储 link_data_map，其中 key 是 uint64_t，值是 s_LinkData_t
    // std::unordered_map<uint64_t, LinkDataCenterLine> link_data_map{};
    // std::vector<uint64_t> link_data_keys{};
    std::vector<uint32_t> lane_laneid_list{};
    std::string line;

    // 跳过标题行
    std::getline(file, line);

    int link_data_count = 0;
    ////std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
    while (std::getline(file, line)) {
        link_data_count++;
        std::stringstream ss(line);
        std::string token;

        // std::cout << "ss: " << ss.str() << std::endl;

        LinkData link_data;
        LaneConnectInfo lane;
        CenterLinePointInfo lane_connect;
        uint32_t lane_laneid = 0;

        // 解析 RoadId
        std::getline(ss, token, ',');
        uint32_t road_id = std::stoul(token);
        link_data.RoadId = road_id;

        // 解析 RoadKind
        std::getline(ss, token, ',');
        uint8_t road_kind = static_cast<uint8_t>(std::stoi(token));
        link_data.RoadKind.data = road_kind;

        // 解析 RoadDirection
        std::getline(ss, token, ',');
        link_data.RoadDirection.data = static_cast<uint8_t>(std::stoi(token));

        // 解析 RoadForm
        std::getline(ss, token, ',');
        link_data.RoadForm.data = static_cast<uint8_t>(std::stoi(token));

        // 解析 LaneCount
        std::getline(ss, token, ',');
        link_data.LaneCount = static_cast<uint8_t>(std::stoi(token));

        // 解析 Width
        std::getline(ss, token, ',');
        link_data.Width = std::stof(token);

        // 解析 SpeedLimitMin
        std::getline(ss, token, ',');
        link_data.SpeedLimitMin = std::stoll(token);

        // 解析 SpeedLimitMax
        std::getline(ss, token, ',');
        link_data.SpeedLimitMax = std::stoll(token);

        // 解析 RoadStatus
        std::getline(ss, token, ',');
        link_data.RoadStatus.data = static_cast<uint8_t>(std::stoi(token));

        // 解析 NextIds
        std::getline(ss, token, ',');
        std::stringstream next_ids_ss(token);
        std::string next_id;
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        while (std::getline(next_ids_ss, next_id, '/')) {
            // std::cout << __FILE__ << __LINE__ << " " << "next_id: " << next_id << std::endl; // 添加调试输出
            try {
                if (!next_id.empty()) {
                    link_data.NextIds.push_back(std::stoul(next_id));
                }
            } catch (const std::exception& e) {
                std::cout << " while parsing next_id error" << std::endl;
            }
        }
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;


        // 解析 PrevIds
        std::getline(ss, token, ',');
        std::stringstream prev_ids_ss(token);
        std::string prev_id;
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        while (std::getline(prev_ids_ss, prev_id, '/')) {
            if (!prev_id.empty()) {
                link_data.PrevIds.push_back(std::stoul(prev_id));
            }
        }
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;


        // 解析 lane_roadid
        std::getline(ss, token, ',');
        lane.RoadId = std::stoul(token);

        // 解析 lane_lanenum
        std::getline(ss, token, ',');
        lane.LaneNum = static_cast<uint8_t>(std::stoi(token));

        // 解析 lane_laneid
        std::getline(ss, token, ',');
        lane.LaneId = std::stoul(token);
        // lane_laneid_list.emplace_back(lane.LaneId);
        lane_laneid = lane.LaneId;

        // 解析 lane_nexts
        std::getline(ss, token, ',');
        std::stringstream lane_nexts_ss(token);
        std::string lane_next;
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        while (std::getline(lane_nexts_ss, lane_next, ')')) {
            if (!lane_next.empty()) {
                size_t pos = lane_next.find('(');
                if (pos != std::string::npos) {
                    lane_next = lane_next.substr(pos + 1);
                    size_t colon_pos = lane_next.find(':');
                    if (colon_pos != std::string::npos) {
                        LaneTopo lane_topo{};
                        // lane_topo.road_id = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // colon_pos = lane_next.find(':');
                        // lane_topo.lane_num = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // colon_pos = lane_next.find(':');
                        // lane_topo.lane_id = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // lane_topo.topo_dir = std::stoul(lane_next);
                        lane.Nexts.push_back(lane_topo);
                    }
                }
            }
        }
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;


        // 解析 lane_pres
        std::getline(ss, token, ',');
        std::stringstream lane_pres_ss(token);
        std::string lane_pre;
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        while (std::getline(lane_pres_ss, lane_pre, ')')) {
            if (!lane_pre.empty()) {
                size_t pos = lane_pre.find('(');
                if (pos != std::string::npos) {
                    lane_pre = lane_pre.substr(pos + 1);
                    size_t colon_pos = lane_pre.find(':');
                    if (colon_pos != std::string::npos) {
                        LaneTopo lane_topo{};
                        // lane_topo.road_id = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // colon_pos = lane_next.find(':');
                        // lane_topo.lane_num = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // colon_pos = lane_next.find(':');
                        // lane_topo.lane_id = std::stoul(lane_next.substr(0, colon_pos));
                        // lane_next = lane_next.substr(colon_pos + 1);
                        // lane_topo.topo_dir = std::stoul(lane_next);
                        lane.Nexts.push_back(lane_topo);
                    }
                }
            }
        }
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;

        // 解析 lane_CategoryVehicle
        std::getline(ss, token, ',');
        lane.CategoryVehicle = std::stoll(token);

        // 解析 lane_CategorySpecial
        std::getline(ss, token, ',');
        lane.CategorySpecial = std::stoll(token);

        // 解析 lane_CategoryLocation
        std::getline(ss, token, ',');
        lane.CategoryLocation = std::stoll(token);

        // 解析 lane_Transition
        std::getline(ss, token, ',');
        lane.Transition = static_cast<uint8_t>(std::stoi(token));

        // 解析 lane_isVirtual
        std::getline(ss, token, ',');
        lane.isVirtual = static_cast<int8_t>(std::stoi(token));

        // 解析 laneArrowType
        std::getline(ss, token, ',');
        lane.lane_arrow_type = static_cast<uint8_t>(std::stoi(token));

        // 解析 LaneConnects
        std::getline(ss, token, ',');
        std::stringstream lane_connects_ss(token);
        // if(lane.RoadId == 107726338){
        //     std::cout << __FILE__ << __LINE__ << " " << lane_connects_ss.str() << std::endl;
        // }
        std::string lane_connect_str;
        //  std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        std::vector<CenterLinePointInfo> tmp_lane_connects{};
        while (std::getline(lane_connects_ss, lane_connect_str, ')')) {
            // if(lane.RoadId == 107726338){
            //     std::cout << __FILE__ << __LINE__ << " " << lane_connect_str << std::endl;
            // }
            if (!lane_connect_str.empty()) {
                size_t pos = lane_connect_str.find('(');
                if (pos != std::string::npos) {
                    lane_connect_str = lane_connect_str.substr(pos + 1);
                    std::stringstream lane_connect_ss(lane_connect_str);
                    // if(lane.RoadId == 107726338){
                    //     std::cout << __FILE__ << __LINE__ << " " << lane_connect_ss.str() << std::endl;
                    // }
                    std::string token;
                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.longitude = std::stod(token);

                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.latitude = std::stod(token);

                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.curvature = std::stod(token);

                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.sloope = std::stod(token);

                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.crossslope = std::stod(token);

                    std::getline(lane_connect_ss, token, ':');
                    lane_connect.offset = std::stod(token);

                    tmp_lane_connects.emplace_back(lane_connect);
                    // link_data.LaneConnects.push_back(lane_connect);
                }
            }
        }
        // std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;

        // if (link_data_map.find(road_id_kind_key) == link_data_map.end()) {
        //     link_data_map[road_id_kind_key] = link_data;
        //     link_data_keys.push_back(road_id_kind_key);
        // }

        // link_data_map[road_id_kind_key].link_data_info.Lanes.push_back(lane);
        // for(auto& tmp_lane_connect : tmp_lane_connects){
        //     link_data_map[road_id_kind_key].link_data_info.LaneConnects.push_back(tmp_lane_connect);

        // }
        //  std::cout << __FILE__ << __LINE__ << " " << "link_data_count: " << link_data_count << std::endl;
        LinkDataCenterLine link_data_ceter_line{};
        link_data_ceter_line.link_data_info = link_data;
        link_data_ceter_line.lane_connect_infos = lane;
        link_data_ceter_line.center_line_infos = tmp_lane_connects;
        if (std::find(lane_laneid_list.begin(), lane_laneid_list.end(), lane_laneid) == lane_laneid_list.end()) {
            link_data_infos.push_back(link_data_ceter_line);
            lane_laneid_list.emplace_back(lane.LaneId);
        }
    }

    // // std::vector<LinkData> link_datas;
    // for (const auto& key : link_data_keys) {
    //     link_data_infos.push_back(link_data_map[key]);
    // }
    return true;
}

bool MakeInputData::ReadLaneMappingInfoJson(const std::string &file_path, std::vector<LaneMappingMsg>& lane_mapping_infos) {
    lane_mapping_infos.clear();

    // std::ifstream file(file_path);
    // if (!file.is_open()) {
    //     // std::cout << "Failed to open file: " << file_path << std::endl;
    //     return false;
    // }

    // // 将文件读取到 buffer 中，并将 buffer 转换为 string 赋值给 content
    // std::stringstream buffer;
    // buffer << file.rdbuf();
    // std::string content = buffer.str();

    // // 解析 JSON 数据
    // Json::Value root;
    // Json::CharReaderBuilder readerBuilder;
    // std::string errs;
    // std::istringstream s(content);
    // if (!Json::parseFromStream(readerBuilder, s, &root, &errs)) {
    //     std::cerr << "JSON parse error: " << errs << std::endl;
    //     return false;
    // }

    // // 遍历 JSON 数据并存储在 lane_mapping_infos 中
    // for (const auto& item : root) {
    //     LaneMappingMsg info;

    //     info.RoadId = item["RoadId"].asUInt();

    //     lane_mapping_infos.push_back(info);
    // }

    // file.close();
    return true;
}

bool MakeInputData::GenerateIntersectionDataMap(const std::vector<Intersection>& intersection_infos){
    std::unordered_map<uint32_t, std::vector<int>> &intersection_InnerRoadsIds_map = hq_raw_data_map_.intersection_InnerRoadsIds_map;
    intersection_InnerRoadsIds_map.clear();
    for (int intersection_index = 0; intersection_index < intersection_infos.size(); intersection_index++) {
        Intersection search_intersection = intersection_infos[intersection_index];
        for(int in_road_index = 0;in_road_index<search_intersection.InRoadsIds.size();in_road_index++){
            uint32_t search_road_id = search_intersection.InRoadsIds[in_road_index];
            if (intersection_InnerRoadsIds_map.find(search_road_id) ==
                intersection_InnerRoadsIds_map.end()) {
                // new link id, add index vector to store
                std::vector<int> index_vec{};
                index_vec.emplace_back(intersection_index);
                intersection_InnerRoadsIds_map.emplace(search_road_id, index_vec);
            } else {
                intersection_InnerRoadsIds_map[search_road_id].emplace_back(intersection_index);
            }            
        }
    }    
    return true;
}

bool MakeInputData::GenerateLinkDataMap(const std::vector<LinkMemory>& link_data_infos){
    std::unordered_map<uint32_t,int> &link_data_RoadId_map = hq_raw_data_map_.link_data_RoadId_map;
    link_data_RoadId_map.clear();
    for (int raw_data_index = 0; raw_data_index < link_data_infos.size(); raw_data_index++) {
        link_data_RoadId_map.emplace(link_data_infos[raw_data_index].LinkData_info.RoadId, raw_data_index);
    }
    return true;
}

bool MakeInputData::GenerateLaneMappingMsgDataMap(const std::vector<LaneMappingMsg>& lane_mapping_msg){
    std::unordered_map<uint32_t, int> &lane_mapping_msg_link_map = hq_raw_data_map_.lane_mapping_msg_link_map;
    lane_mapping_msg_link_map.clear();
    for (int raw_data_index = 0; raw_data_index < lane_mapping_msg.size(); raw_data_index++) {
        lane_mapping_msg_link_map.emplace(lane_mapping_msg[raw_data_index].RoadId, raw_data_index);
    }
    return true;
}

// bool MakeInputData::GeneratePositionInfosDataMap(const std::vector<PositionInfo> position_infos){
//     std::unordered_map<uint32_t, int> &position_info_id_map = hq_raw_data_map_.position_info_id_map;
//     position_info_id_map.clear();
//     for (int raw_data_index = 0; raw_data_index < position_infos.size(); raw_data_index++) {
//         position_info_id_map.emplace(position_infos[raw_data_index].PositionID, raw_data_index);
//     }
//     return true;

// }

bool MakeInputData::SortLane(LaneMappingMemory & lane_mapping){
    if(lane_mapping.LaneData.size()<=0 && lane_mapping.TrajHistory.size()<=0){
        return false;
    }
    //判断是出还是入路口
    uint8_t road_type = 0;//0-normal;1-路口前；2-路口中；3-路口后
    for(auto traj:lane_mapping.TrajHistory){
        if(traj.RoadType==3 || traj.RoadType == 1){
            road_type =traj.RoadType;
            break;
        }
    }
    //找坐标转换的原点和heading
    std::vector<Point2Dd> point_line_wgs84{};
    // //找一条有两个点的线，取前两个点
    for(int lane_index = 0;lane_index < lane_mapping.LaneData.size(); lane_index++){
        //这里的lane即line,一个lane一条line
        if(lane_mapping.LaneData[lane_index].size()>1){
            Point2Dd line_point0;
            Point2Dd line_point1;
            line_point0.x = lane_mapping.LaneData[lane_index][0].X;
            line_point0.y = lane_mapping.LaneData[lane_index][0].Y;
            line_point1.x = lane_mapping.LaneData[lane_index][1].X;
            line_point1.y = lane_mapping.LaneData[lane_index][1].Y;
            point_line_wgs84.push_back(line_point0);
            point_line_wgs84.push_back(line_point1);
            break;
        }
    }
    if(point_line_wgs84.size()<2){
        return false;
    }
    // 将取到的两个点，转到正北的方向上，并平移到[1]点上，point_line_wgs84[1]为坐标原点，headings[1]为方向
    std::vector<Point2Dd> point_line_north{};
    for(auto point:point_line_wgs84){
        DoublePosePoint ego_point;
        ego_point.x = point_line_wgs84[1].x;
        ego_point.y = point_line_wgs84[1].y;
        ego_point.yaw = 0;
        Point2Dd body_point;
        WGS84ToBody(point, ego_point, body_point);
        point_line_north.push_back(body_point);      
    }
    //求线段相对正北的heading
    std::vector<double> headings{};
    ComputePathHeading(point_line_north,headings);
    lane_mapping.link_heading = -headings[1];//atan2 转成顺时针为正

    //直接用position的heading和第一个点
    DoublePosePoint ego_point;
    ego_point.x = lane_mapping.TrajHistory.front().PosLgt;
    ego_point.y = lane_mapping.TrajHistory.front().PosLat;;
    // ego_point.yaw = lane_mapping.TrajHistory.front().Heading;    
    ego_point.yaw = lane_mapping.link_heading;  
    // std::cout << std::cout.precision(12) << " ego_point.yaw " << ego_point.yaw << std::endl;
    //将每条线的第一个点转到heading上
    //顺便将点数不够的线去掉
    std::vector<std::tuple<Point2Dd, int, double>> point_body_start_vec{};//点坐标，线索引lane_mapping.LaneData，线长度
    for(int lane_index = 0;lane_index < lane_mapping.LaneData.size(); lane_index++){
        if(lane_mapping.LaneData[lane_index].size()>1){
            Point2Dd line_point0;
            Point2Dd body_point0;
            line_point0.x = lane_mapping.LaneData[lane_index][0].X;
            line_point0.y = lane_mapping.LaneData[lane_index][0].Y;
            WGS84ToBody(line_point0, ego_point, body_point0);
            double line_dist = 0;
            auto line = lane_mapping.LaneData[lane_index];
            for(int i=1;i<line.size();i++){
                line_dist += calculateDistanceGC02(line[i-1].X, line[i-1].Y, 0.0, line[i].X,line[i].Y, 0.0);
            } 
            point_body_start_vec.push_back(std::make_tuple(body_point0,lane_index,line_dist));
        }
    }

    //求线的end点的对于原点的转换后的坐标
    std::vector<std::pair<Point2Dd, int>> point_body_end_vec{};
    for(int lane_index = 0;lane_index < lane_mapping.LaneData.size(); lane_index++){
        if(lane_mapping.LaneData[lane_index].size()>1){
            Point2Dd line_point0;
            Point2Dd body_point0;
            line_point0.x = lane_mapping.LaneData[lane_index].back().X;
            line_point0.y = lane_mapping.LaneData[lane_index].back().Y;
            WGS84ToBody(line_point0, ego_point, body_point0);
            point_body_end_vec.push_back(std::make_pair(body_point0,lane_index));
        }
    }
    std::vector<std::tuple<Point2Dd, int, double>> point_body_vec= point_body_start_vec;
    //如果首部的x距离最大和最小差值很大，需要到最大的x附近的点处算y
    {
        double min_x = 1000; 
        double max_x = 0;
        for(auto iter:point_body_vec){
            double x = std::get<0>(iter).x;
           if(x> max_x){
              max_x = x;
           }
           if(x <= min_x){
              min_x = x;
           }
        }
        if(std::abs(max_x - min_x)>20){
            //加权结果
            for(int i=0;i< point_body_vec.size();i++){
                std::get<0>(point_body_vec[i]).y = std::get<0>(point_body_start_vec[i]).y + point_body_end_vec[i].first.y;
            }
            
        }   
    }        

    // bubble sort, y min->max
    for (int i = 0; i < point_body_vec.size(); i++) {
        for (int j = 0; j < point_body_vec.size() - 1 - i; j++) {
            if (std::get<0>(point_body_vec[j]).y > std::get<0>(point_body_vec[j + 1]).y) {
                std::tuple<Point2Dd,int,double> tmp{};
                tmp = point_body_vec[j];
                point_body_vec[j] = point_body_vec[j + 1];
                point_body_vec[j + 1] = tmp;
            }
        }
    }

    //0307new,出路口的中间的短线，去掉
    std::vector<std::tuple<Point2Dd, int, double>> point_body_vec_tmp{};
    if(road_type ==3){
        for(int i=0; i<point_body_vec.size();i++){           
            if(std::get<0>(point_body_vec[i]).x > 50 && std::get<2>(point_body_vec[i])<20 && i!=0 && i!=point_body_vec.size()-1){
                //
            }else{
                point_body_vec_tmp.push_back(point_body_vec[i]);
            }
        }
        point_body_vec = point_body_vec_tmp;
    }else{
        //do nothing
    }
    //0219new, 还需要判断个线的长短， 以及对于短线，是否有另一条短线y和这条短线很近，如果有则首尾相连
    //只针对最边缘的线,找到距离边缘近的短线，max_y一组， min_y一组, 只处理这两组即可
    double max_y = std::get<0>(point_body_vec.back()).y;
    double min_y = std::get<0>(point_body_vec.front()).y;
    std::vector<std::tuple<Point2Dd, int, int>> max_y_group{};//点坐标，lane_mapping.LaneData线索引，以及point_body_vec的索引
    std::vector<std::tuple<Point2Dd, int, int>> min_y_group{};//点坐标，线索引，以及point_body_vec的索引
    for(int i =0;i< point_body_vec.size();i++){
        if(std::get<2>(point_body_vec[i])<20 ){//线长度短
            if(std::abs(std::get<0>(point_body_vec[i]).y - max_y)<0.3){
                //靠近max侧, 同时按照x排序min->max,如果新来的比原来的小，insert
                bool insert_f =false;
                for(int j = 0;j< max_y_group.size(); j++){
                    if(std::get<0>(point_body_vec[i]).x< std::get<0>(max_y_group[j]).x){
                        std::tuple<Point2Dd, int, int> tmp=std::make_tuple(std::get<0>(point_body_vec[i]),std::get<1>(point_body_vec[i]),i);
                        max_y_group.insert(max_y_group.begin()+j, tmp);
                        insert_f = true;
                        break;
                    }
                }
                if(insert_f == false){
                    std::tuple<Point2Dd, int, int> tmp=std::make_tuple(std::get<0>(point_body_vec[i]),std::get<1>(point_body_vec[i]),i);
                    max_y_group.push_back(tmp);
                }
                
            }else if(std::abs(std::get<0>(point_body_vec[i]).y - min_y)<0.3){
                //靠近min侧, 同时按照x排序min->max,如果新来的比原来的小，insert
                bool insert_f =false;
                for(int j = 0;j< min_y_group.size(); j++){
                    if(std::get<0>(point_body_vec[i]).x< std::get<0>(min_y_group[j]).x){
                        std::tuple<Point2Dd, int, int> tmp=std::make_tuple(std::get<0>(point_body_vec[i]),std::get<1>(point_body_vec[i]),i);
                        min_y_group.insert(min_y_group.begin()+j, tmp);
                        insert_f = true;
                        break;
                    }
                }
                if(insert_f == false){
                    std::tuple<Point2Dd, int, int> tmp=std::make_tuple(std::get<0>(point_body_vec[i]),std::get<1>(point_body_vec[i]),i);
                    min_y_group.push_back(tmp);
                }
            }
        }
    }
    //只有一根的去掉， 多根的要首尾相连
    std::vector<LPoint> min_y_connect_lane{};
    std::vector<LPoint> max_y_connect_lane{};
    uint8_t max_y_fill_type =0; //1-delete, 2- connect
    uint8_t min_y_fill_type =0; //1-delete, 2- connect
    if(min_y_group.size()>1){
        min_y_fill_type = 2;
        min_y_connect_lane = lane_mapping.LaneData[std::get<1>(min_y_group[0])];
        for(int i =1; i<min_y_group.size();i++){
            min_y_connect_lane.insert(min_y_connect_lane.end(),lane_mapping.LaneData[std::get<1>(min_y_group[i])].begin(), lane_mapping.LaneData[std::get<1>(min_y_group[i])].end());
        }
    }else if(min_y_group.size()==1){
        min_y_fill_type = 1;
    }

    if(max_y_group.size()>1){
        max_y_fill_type =2;
        max_y_connect_lane = lane_mapping.LaneData[std::get<1>(max_y_group[0])];
        for(int i =1; i<max_y_group.size();i++){
            max_y_connect_lane.insert(max_y_connect_lane.end(),lane_mapping.LaneData[std::get<1>(max_y_group[i])].begin(), lane_mapping.LaneData[std::get<1>(max_y_group[i])].end());
        }
    }else if(max_y_group.size()==1){
        max_y_fill_type = 1;
    }

    //填充
    std::vector<std::vector<LPoint>> LaneDatas_tmp{};
    // for(auto body:point_body_vec){
    //     if(std::get<1>(body)<lane_mapping.LaneData.size()){
    //         LaneDatas_tmp.push_back(lane_mapping.LaneData[std::get<1>(body)]);
    //     }
    // }  
    bool min_y_connect_done =false;
    bool max_y_connect_done =false;
    for(auto body:point_body_vec){
        if(std::get<1>(body)<lane_mapping.LaneData.size()){
            bool need_push = true;
            if(min_y_fill_type ==1 && !min_y_group.empty() && std::get<1>(min_y_group[0]) == std::get<1>(body)){//找到需要删除的线
                //do not push
                need_push =false;
            }else if(min_y_fill_type==2 ){//找到合并的线
                for(auto iter:min_y_group){
                    if(std::get<1>(iter) == std::get<1>(body)){
                        need_push =false;
                        if(min_y_connect_done == false){//还没加入的
                            LaneDatas_tmp.push_back(min_y_connect_lane);
                            min_y_connect_done =true;
                        }
                    }
                }
            }
            
            if(max_y_fill_type ==1 && !max_y_group.empty() && std::get<1>(max_y_group[0]) == std::get<1>(body)){
                //do not push
                need_push =false;
            }else if(max_y_fill_type==2){//找到合并的线
                for(auto iter:min_y_group){
                    if(std::get<1>(iter) == std::get<1>(body)){
                        need_push =false;
                        if(max_y_connect_done == false){//还没加入的
                            LaneDatas_tmp.push_back(max_y_connect_lane);
                            max_y_connect_done =true;
                        }
                    }
                }
            }
            if(need_push == true){
                LaneDatas_tmp.push_back(lane_mapping.LaneData[std::get<1>(body)]);
            }
            
        }
    }
    lane_mapping.LaneData = LaneDatas_tmp;

#ifdef PRO_DEAL
    {
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:point_body_start_vec){
        ss2 << " " << std::get<0>(p).x;
        ss << " " << std::get<0>(p).y;
        }
    std::cout << std::cout.precision(12) << " point_body_start_vecx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " point_body_start_vecy=[ " << ss.str() << " ]" << std::endl;
    }
    {
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:point_body_end_vec){
        ss2 << " " << p.first.x;
        ss << " " << p.first.y;
        }
    std::cout << std::cout.precision(12) << " point_body_end_vecx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " point_body_end_vecy=[ " << ss.str() << " ]" << std::endl;
    }
    {
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:point_body_vec){
        ss2 << " " << std::get<0>(p).x;
        ss << " " << std::get<0>(p).y;
        }
    std::cout << std::cout.precision(12) << " point_body_vecx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " point_body_vecy=[ " << ss.str() << " ]" << std::endl;
    }
#endif

    std::vector<std::pair<Point2Dd, int>> point_body_start_vec_final{};
    for(int lane_index = 0;lane_index < lane_mapping.LaneData.size(); lane_index++){
        if(lane_mapping.LaneData[lane_index].size()>1){
            Point2Dd line_point0;
            Point2Dd body_point0;
            line_point0.x = lane_mapping.LaneData[lane_index][0].X;
            line_point0.y = lane_mapping.LaneData[lane_index][0].Y;
            WGS84ToBody(line_point0, ego_point, body_point0);
            point_body_start_vec_final.push_back(std::make_pair(body_point0,lane_index));
        }
    }
    std::vector<std::pair<Point2Dd, int>> point_body_end_vec_final{};
    for(int lane_index = 0;lane_index < lane_mapping.LaneData.size(); lane_index++){
        if(lane_mapping.LaneData[lane_index].size()>1){
            Point2Dd line_point0;
            Point2Dd body_point0;
            line_point0.x = lane_mapping.LaneData[lane_index].back().X;
            line_point0.y = lane_mapping.LaneData[lane_index].back().Y;
            WGS84ToBody(line_point0, ego_point, body_point0);
            point_body_end_vec_final.push_back(std::make_pair(body_point0,lane_index));
        }
    }
#ifdef PRO_DEAL
    {
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:point_body_start_vec_final){
        ss2 << " " << p.first.x;
        ss << " " << p.first.y;
        }
    std::cout << std::cout.precision(12) << " point_body_start_vec_finalx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " point_body_start_vec_finaly=[ " << ss.str() << " ]" << std::endl;
    }
    {
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:point_body_end_vec_final){
        ss2 << " " << p.first.x;
        ss << " " << p.first.y;
        }
    std::cout << std::cout.precision(12) << " point_body_end_vec_finalx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " point_body_end_vec_finaly=[ " << ss.str() << " ]" << std::endl;
    }
#endif
    //求自车轨迹的第一个点和最后一个点的，转坐标，和线的转换统一，point_line_wgs84[1]为坐标原点，headings[1]为方向
    if(lane_mapping.TrajHistory.size()<1){
        lane_mapping.enter_ego_lane_id = 0;
        lane_mapping.exit_ego_lane_id = 0;
    }else{
        // 进入的pos就是现在转化的原点，所以是0，0
        Point2Dd ego_point_exit, ego_point_middle;
        Point2Dd body_point_enter, body_point_exit, body_point_middle;
        body_point_enter = {0.0,0.0};
        ego_point_exit.x = lane_mapping.TrajHistory.back().PosLgt;
        ego_point_exit.y = lane_mapping.TrajHistory.back().PosLat;
        WGS84ToBody(ego_point_exit, ego_point, body_point_exit);
        //判断左右，决定ego id
        for(int i = 0;i<point_body_start_vec_final.size();i++){
            if(body_point_enter.y>std::get<0>(point_body_start_vec_final[i]).y){
                lane_mapping.enter_ego_lane_id = static_cast<uint8_t>(i)+1;
            }
        }
        //判断左右，决定ego id
        for(int i = 0;i<point_body_end_vec_final.size();i++){
            if(body_point_exit.y>std::get<0>(point_body_end_vec_final[i]).y){
                lane_mapping.exit_ego_lane_id = static_cast<uint8_t>(i)+1;
            }
        }

        //0218new ,道路中间区域的ego lane id

    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << __LINE__ << " " << "lane_mapping.exit_ego_lane_id: " << (int)lane_mapping.exit_ego_lane_id << std::endl; 
    std::cout << __FILE__ << __LINE__ << " " << "lane_mapping.enter_ego_lane_id: " << (int)lane_mapping.enter_ego_lane_id << std::endl; 
#endif   
    return true;

}

bool MakeInputData::PickLane(LaneMappingMemory & lane_mapping, int section_num){
    //偶数的section_num 代表是进入， 奇数代表出
    uint8_t ego_id = 0;
    if(section_num%2 != 0){
        ego_id = lane_mapping.enter_ego_lane_id;
    }else{
        ego_id = lane_mapping.exit_ego_lane_id;
    }

    if(ego_id == 0){
        return false;
    }
    int lane_index = ego_id - 1;
    std::vector<std::vector<LPoint>> LaneData{};
    for(int i=0; i<lane_mapping.LaneData.size();i++){
        if(i>= lane_index-1 && i<=lane_index+2){
            LaneData.push_back(lane_mapping.LaneData[i]);
        }
    }
    lane_mapping.LaneData = LaneData;
    //因为pick之后，只会选出自车周围的3车道的4条线，所以如果ego_id >2的要修成2
    int id_gap = ego_id -2;
    if(id_gap > 0){
        lane_mapping.enter_ego_lane_id = std::max(lane_mapping.enter_ego_lane_id-id_gap, 1);
        lane_mapping.exit_ego_lane_id = std::max(lane_mapping.exit_ego_lane_id-id_gap, 1);
    }
    return true;
}

bool MakeInputData::CutLane(const LaneMappingMemory & lane_mapping , int section_num,
                            std::vector<std::vector<Point2Dd>>& cut_lane_lines_xy,DoublePosePoint& ori_point, int& base_lane_index){
    cut_lane_lines_xy.clear();
    base_lane_index = 0;
    //输入的是排好顺序的lane，且每个lane的点数是大于1的，先简单取中间的lane的法向量，会比较符合道路走向；
    //沿最短的道路切
    // std::cout << __FILE__ << "," << __LINE__ << ","<< " CutLane  in "  << std::endl;
    //  取离自车轨迹一开始最近的一条车道
    // 奇数的路口，即路口后，
    // 偶数的路口，即路口前
    if (lane_mapping.TrajHistory.size()<= 1) {
        return false;
    }
    bool is_cut_front = false;
    uint8_t road_type = 0;//0-normal;1-路口前；2-路口中；3-路口后
    for(auto traj:lane_mapping.TrajHistory){
        if(traj.RoadType==3 || traj.RoadType == 1){
            road_type =traj.RoadType;
            if(road_type == 3 ){
                is_cut_front = true;
            }
            break;
        }
    }
    // if(section_num%2 != 0){
    //     //奇数
    //     is_cut_front = true;
    // }
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< " is_cut_front "  <<(int)is_cut_front<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< " section_num "  <<section_num<< std::endl;
#endif
    //  将车道的线转到正北方向的平面坐标上，以轨迹首点为原点
    std::vector<std::vector<Point2Dd>> lane_lines_xy{};
    DoublePosePoint ego_point;
    ego_point.x = lane_mapping.TrajHistory[0].PosLgt;
    ego_point.y = lane_mapping.TrajHistory[0].PosLat;
    ego_point.yaw = lane_mapping.TrajHistory[0].Heading;
    int max_length_index = -1;
    double max_length = 0;
    for(int lane_index = 0;lane_index<lane_mapping.LaneData.size(); lane_index++){
        auto lane_data = lane_mapping.LaneData[lane_index];
        Point2Dd line_point;
        Point2Dd body_point;
        std::vector<Point2Dd> one_line{};
        for(int point_index=0; point_index<lane_data.size();point_index++){
            auto point = lane_data[point_index];         
            line_point.x = point.X;
            line_point.y = point.Y;
            WGS84ToBody(line_point, ego_point, body_point);
            one_line.push_back(body_point);
        } 
        lane_lines_xy.push_back(one_line);
    }
#ifdef PRO_DEAL
    {
    for(int i =0; i<lane_lines_xy.size();i++){
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:lane_lines_xy[i]){
        ss2 << " " << p.x;
        ss << " " << p.y;            
        }
    std::cout << std::cout.precision(12) << " body_line"<<i<<"x=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " body_line"<<i<<"y=[ " << ss.str() << " ]" << std::endl;
    }
    }
#endif
    //首尾部各取距离自车行点最近的线，做法线
    Point2Dd ego_traj_back_point_wgs;
    ego_traj_back_point_wgs.x = lane_mapping.TrajHistory.back().PosLgt;
    ego_traj_back_point_wgs.y = lane_mapping.TrajHistory.back().PosLat;
    Point2Dd ego_traj_back_point_xy;
    WGS84ToBody(ego_traj_back_point_wgs, ego_point, ego_traj_back_point_xy);
    double min_back_dist = 255;
    double min_front_dist = 255;
    int min_back_index = 0;
    int min_front_index = 0;
    for(int i=0;i<lane_lines_xy.size();i++){
        if(lane_lines_xy[i].size()>0){
            double dist_front_tmp = Distance(0.0, 0.0, lane_lines_xy[i].front().x, lane_lines_xy[i].front().y);
            if(min_front_dist> dist_front_tmp){
                min_front_dist = dist_front_tmp;
                min_front_index = i;
            }
            double dist_back_tmp = Distance(ego_traj_back_point_xy.x, ego_traj_back_point_xy.y, lane_lines_xy[i].back().x, lane_lines_xy[i].back().y);
            if(min_back_dist> dist_back_tmp){
                min_back_dist = dist_back_tmp;
                min_back_index = i;
            }
        }
    }
#ifdef PRO_DEAL
    std::cout << std::cout.precision(12) << " min_back_index " << min_back_index<< std::endl;
    std::cout << std::cout.precision(12) << " min_front_index " << min_front_index << std::endl; 
#endif
    // int min_back_index = (lane_mapping.exit_ego_lane_id >0) ? (lane_mapping.exit_ego_lane_id -1) : 0;
    // int min_front_index = (lane_mapping.enter_ego_lane_id >0) ? (lane_mapping.enter_ego_lane_id -1) : 0;
    //求最长线的首尾的方向向量，和求法向量， 在平面坐标下求
    Point2Dd back_lane_dir_vec(0.0,0.0);
    Point2Dd back_lane_n_vec(0.0,0.0);
    Point2Dd front_lane_dir_vec(0.0,0.0);
    Point2Dd front_lane_n_vec(0.0,0.0);
    Point2Dd back_ori_point(0.0, 0.0);
    Point2Dd front_ori_point(0.0, 0.0);
    double front_lane_width = 0;
    double back_lane_width = 0;
    //std::cout << __FILE__ << ":" << __LINE__ << ": " << "lane_lines_xy.size(): " <<lane_lines_xy.size()<< std::endl;
    //std::cout << __FILE__ << ":" << __LINE__ << ": " << "min_back_index: " <<min_back_index<< std::endl;
    //std::cout << __FILE__ << ":" << __LINE__ << ": " << "lane_mapping.exit_ego_lane_id: " <<(int)lane_mapping.exit_ego_lane_id<< std::endl;
    if(min_back_index<lane_lines_xy.size()){
        if(lane_lines_xy[min_back_index].size()>1){
            //尾部
            Point2Dd back_first_point(lane_lines_xy[min_back_index].back().x, lane_lines_xy[min_back_index].back().y);
            Point2Dd back_second_point((*(lane_lines_xy[min_back_index].rbegin()+1)).x, (*(lane_lines_xy[min_back_index].rbegin()+1)).y);
            back_lane_dir_vec = DirectionVector(back_second_point, back_first_point);
            back_lane_n_vec.x =back_lane_dir_vec.y;
            back_lane_n_vec.y = -back_lane_dir_vec.x;
            back_ori_point = back_first_point;
            //求自车所在车道的车道宽
            if(lane_mapping.exit_ego_lane_id >0 && lane_mapping.exit_ego_lane_id< lane_lines_xy.size()){
                PointSLd sl_tmp;
                CalPointSLBodyCoordinate(lane_lines_xy[lane_mapping.exit_ego_lane_id -1], lane_lines_xy[lane_mapping.exit_ego_lane_id].back(), sl_tmp);
                back_lane_width = sl_tmp.l;
            }
        }
    }else{
        return false;
    }
    if(min_front_index<lane_lines_xy.size()&& lane_lines_xy[min_front_index].size()>1){
        //首部
        Point2Dd front_first_point(lane_lines_xy[min_front_index].front().x, lane_lines_xy[min_front_index].front().y);
        Point2Dd front_second_point((*(lane_lines_xy[min_front_index].begin()+1)).x, (*(lane_lines_xy[min_front_index].begin()+1)).y); 
        front_lane_dir_vec = DirectionVector(front_first_point, front_second_point);       
        front_lane_n_vec.x =front_lane_dir_vec.y;
        front_lane_n_vec.y = -front_lane_dir_vec.x;
        front_ori_point = front_first_point;
        //求自车所在车道的车道宽
        if(lane_mapping.enter_ego_lane_id >0 && lane_mapping.enter_ego_lane_id< lane_lines_xy.size()){
            PointSLd sl_tmp;
            CalPointSLBodyCoordinate(lane_lines_xy[lane_mapping.enter_ego_lane_id -1], lane_lines_xy[lane_mapping.enter_ego_lane_id].front(), sl_tmp);
            front_lane_width = sl_tmp.l;
        }
    }else{
        return false;
    }
#ifdef PRO_DEAL
    std::cout << std::cout.precision(12) << " back_lane_width" << back_lane_width<< std::endl;
    std::cout << std::cout.precision(12) << " front_lane_width " << front_lane_width << std::endl; 
#endif   
    //法向量和方向向量都是以0，0为原点，需要法向量移到线段的端点,构造法线，
    //lane_n_vec现在方向是lane_dir_vec的右边
    Point2Dd fornt_right_point,back_right_point;
    fornt_right_point = ExtendPoint(front_ori_point,front_lane_n_vec,1);
    back_right_point = ExtendPoint(back_ori_point,back_lane_n_vec,1);
#ifdef PRO_DEAL
                  std::cout << __FILE__ << "," << __LINE__ << ","
              << " back_lane_n_vec.x: "  <<back_lane_n_vec.x<< " ,back_lane_n_vec.y: "  <<back_lane_n_vec.y<<std::endl; 
              std::cout << __FILE__ << "," << __LINE__ << ","
              << " front_ori_point.x: "  <<front_ori_point.x<< " ,front_ori_point.y: "  <<front_ori_point.y<<std::endl; 
                            std::cout << __FILE__ << "," << __LINE__ << ","
              << " back_ori_point.x: "  <<back_ori_point.x<< " ,back_ori_point.y: "  <<back_ori_point.y<<std::endl; 
#endif
    //对所有道路切割
    std::array<Point2Dd,2> front_n_line = {front_ori_point, fornt_right_point};
    std::array<Point2Dd,2> back_n_line = {back_ori_point, back_right_point};
    for(int line_index = 0; line_index<lane_lines_xy.size() && line_index<lane_mapping.LaneData.size();line_index++){
        if(lane_lines_xy[line_index].size()>1){
                // std::cout << __FILE__ << "," << __LINE__ << "," << " line_index: "  <<line_index<<std::endl; 
                // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_lines_xy[line_index].size(): "  <<lane_lines_xy[line_index].size()<<std::endl; 
            //首部
            Point2Dd front_first_point(lane_lines_xy[line_index].front().x, lane_lines_xy[line_index].front().y);
            Point2Dd front_second_point((*(lane_lines_xy[line_index].begin()+1)).x, (*(lane_lines_xy[line_index].begin()+1)).y);
            Point2Dd front_cross_point;
            ComputeCrossPoint(front_n_line[0], front_n_line[1], front_first_point,front_second_point, front_cross_point);
            //判断插入的位置
            int insert_front_index = -1;
            if(line_index != min_front_index){//基准线不切
                for(int point_index = 0;point_index<lane_lines_xy[line_index].size();point_index++){
                    if(front_cross_point.x<lane_lines_xy[line_index].at(point_index).x){
                        insert_front_index = point_index;
                        break;
                    }
                }                
            }
            Point2Dd front_extend_point;
            if(insert_front_index == 0){//属于延长，那么用车道宽延长,法向量指向车道heading的右边
                double extend_l = front_lane_width*(min_front_index - line_index);
                front_extend_point =ExtendPoint(front_ori_point,front_lane_n_vec,extend_l);
            }
            //尾部
            Point2Dd back_first_point(lane_lines_xy[line_index].back().x, lane_lines_xy[line_index].back().y);
            Point2Dd back_second_point((*(lane_lines_xy[line_index].rbegin()+1)).x, (*(lane_lines_xy[line_index].rbegin()+1)).y);
            Point2Dd back_cross_point;
            ComputeCrossPoint(back_n_line[0], back_n_line[1], back_first_point,back_second_point, back_cross_point);
            //判断插入的位置
            int insert_back_index = -1;
            if(line_index != min_back_index){//基准线不切
                for(int point_index = lane_lines_xy[line_index].size()-1;point_index>=0;point_index--){
                    if(back_cross_point.x>lane_lines_xy[line_index].at(point_index).x){
                        insert_back_index = point_index+1;
                        break;
                    }
                }
            }
            Point2Dd back_extend_point;
            if(insert_back_index == lane_lines_xy[line_index].size()){//属于延长，那么用车道宽延长
                double extend_l_b = back_lane_width*(min_back_index - line_index);
                back_extend_point =ExtendPoint(back_ori_point,back_lane_n_vec,extend_l_b);
            }
#ifdef PRO_DEAL
              std::cout << __FILE__ << "," << __LINE__ << ","
              << " back_cross_point.x: "  <<back_cross_point.x<< " ,back_cross_point.y: "  <<back_cross_point.y<<std::endl; 
              std::cout << __FILE__ << "," << __LINE__ << ","
              << " front_cross_point.x: "  <<front_cross_point.x<< " ,front_cross_point.y: "  <<front_cross_point.y<<std::endl; 
                  std::cout << __FILE__ << "," << __LINE__ << ","
              << " insert_back_index: "  <<insert_back_index<< " ,insert_front_index: "  <<insert_front_index<<std::endl; 
              std::cout << __FILE__ << "," << __LINE__ << ","
              << " front_extend_point.x: "  <<front_extend_point.x<< " ,front_extend_point.y: "  <<front_extend_point.y<<std::endl; 
              std::cout << __FILE__ << "," << __LINE__ << ","
              << " back_extend_point.x: "  <<back_extend_point.x<< " ,back_extend_point.y: "  <<back_extend_point.y<<std::endl; 
#endif
            //将交点转回wgs84，插入原来的lane里
            Point2Dd back_point_wgs84, front_point_wgs84;
            Point2Dd back_point_xy,front_point_xy;
            if(insert_back_index == lane_lines_xy[line_index].size()){//延长
                // BodyToWGS84(back_extend_point, ego_point, back_point_wgs84);
                back_point_xy = back_extend_point;
            }else{
                // BodyToWGS84(back_cross_point, ego_point, back_point_wgs84);
                back_point_xy = back_cross_point;
            }
            if(insert_front_index == 0){//延长
                // BodyToWGS84(front_extend_point, ego_point, front_point_wgs84);
                front_point_xy = front_extend_point;
            }else{
                // BodyToWGS84(front_cross_point, ego_point, front_point_wgs84);
                front_point_xy = front_cross_point;
            }
            // auto &line_data = lane_mapping.LaneData[line_index];
            auto &line_data = lane_lines_xy[line_index];
            int assign_front_index = (insert_front_index >=0) ? insert_front_index : 0;
            int assign_back_index = (insert_back_index >=0) ? insert_back_index : lane_lines_xy[line_index].size();
            // line_data.assign(line_data.begin()+assign_front_index, line_data.begin()+assign_back_index);

            // LPoint inert_ponit;
            // inert_ponit.X = front_point_wgs84.x;
            // inert_ponit.Y = front_point_wgs84.y;
            // inert_ponit.LineType = line_data.front().LineType;
            // inert_ponit.Color = line_data.front().Color;
            // inert_ponit.PositionID = line_data.front().PositionID;
            // line_data.insert(line_data.begin(),inert_ponit);
            // LPoint inert_ponit2;
            // inert_ponit2.X = back_point_wgs84.x;
            // inert_ponit2.Y = back_point_wgs84.y;
            // inert_ponit2.LineType = line_data.back().LineType;
            // inert_ponit2.Color = line_data.back().Color;
            // inert_ponit2.PositionID = line_data.back().PositionID;
            // line_data.push_back(inert_ponit2);
            
            //处理直角坐标下的点，为之后的等距点做准备
            if(is_cut_front== true){
                //只切front
                line_data.assign(line_data.begin()+assign_front_index, line_data.end());
                line_data.insert(line_data.begin(),front_point_xy);
            }else{
                line_data.assign(line_data.begin(), line_data.end());
                line_data.push_back(back_point_xy);
            }
            // line_data.assign(line_data.begin()+assign_front_index, line_data.begin()+assign_back_index);
            // line_data.insert(line_data.begin(),front_point_xy);
            // line_data.push_back(back_point_xy);
        }
    }
    cut_lane_lines_xy = lane_lines_xy;
    ori_point = ego_point;
    base_lane_index = min_back_index;
    // std::cout << __FILE__ << "," << __LINE__ << ","<< " CutLane end: "  << std::endl;

    return true;
}

bool MakeInputData::FixLane(const std::vector<std::vector<Point2Dd>>& cut_lane_lines_xy,const DoublePosePoint& ori_point,int base_lane_index,LaneMappingMemory & lane_mapping){
#ifdef PRO_DEAL
    {
    for(int i =0; i<cut_lane_lines_xy.size();i++){
    std::stringstream ss2;
    std::stringstream ss;        
        for(auto p:cut_lane_lines_xy[i]){
        ss2 << " " << p.x;
        ss << " " << p.y;            
        }
    std::cout << std::cout.precision(12) << "cut body_line"<<i<<"x=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << "cut body_line"<<i<<"y=[ " << ss.str() << " ]" << std::endl;
    }
    }
#endif
    if(lane_mapping.LaneData.size()!= cut_lane_lines_xy.size() && base_lane_index>= cut_lane_lines_xy.size()){
        return false;
    }
    //对base lane，求点数
    int sample_point_num = cut_lane_lines_xy[base_lane_index].size();
    if(sample_point_num<2){
        return false;
    }
    std::vector<std::vector<LPoint>> res_lane_data{};
    for(int lane_index =0; lane_index<cut_lane_lines_xy.size();lane_index++){
        std::vector<Point2Dd> lane = cut_lane_lines_xy[lane_index];
        if(lane.size()<2 || lane_mapping.LaneData[lane_index].size()<2){
            continue;
        }
        
        double dist = 0;
        for(int point_index = 1;point_index<lane.size();point_index++){
            dist += Distance(lane[point_index].x, lane[point_index].y, lane[point_index-1].x, lane[point_index-1].y);
        }
        double sample_dist = 2.5;
        std::vector<Point2Dd> fix_lane{};
        FixPathDensity(lane, sample_dist,fix_lane);
        //转到wgs84
        std::vector<LPoint> one_wgs_line{};
        for(auto body_point:fix_lane){
            Point2Dd point_wgs84;
            BodyToWGS84(body_point, ori_point, point_wgs84);
            LPoint res_ponit;
            res_ponit.X = point_wgs84.x;
            res_ponit.Y = point_wgs84.y;
            res_ponit.LineType = lane_mapping.LaneData[lane_index].front().LineType;
            res_ponit.Color = lane_mapping.LaneData[lane_index].front().Color;
            one_wgs_line.push_back(res_ponit);
        }
        res_lane_data.push_back(one_wgs_line);
    }
    lane_mapping.LaneData.clear();
    lane_mapping.LaneData = res_lane_data;
    return true;
}

bool MakeInputData::GenerateLinkMemoryDataMap(const std::vector<LinkMemory>& link_memorys){
    std::unordered_map<uint64_t, int> &link_memory_map = hq_raw_data_map_.link_memory_map;
    link_memory_map.clear();
    for (int raw_data_index = 0; raw_data_index < link_memorys.size(); raw_data_index++) {
        uint32_t road_id = link_memorys[raw_data_index].LinkData_info.RoadId;
        uint32_t other_flag = 0; //另一个标志位
        uint64_t link_key = (static_cast<uint64_t>(other_flag) << 32) | road_id;
        link_memory_map.emplace(link_key, raw_data_index);
    }
    return true;
}

// bool MakeInputData::SaveLaneMappingTrajPoints(const std::vector<LaneMappingMsg>& lane_mapping_infos){
//     if(true == lane_mapping_infos.empty()){
//         return false;
//     }
//     for(const auto& lane_mapping_info : lane_mapping_infos){
//         if(true == lane_mapping_info.TrajHistory.empty()){
//             return false;
//         }
//         for(const auto& traj_point : lane_mapping_info.TrajHistory){
//             PositionInfo position_info{};
//         }
//     }
//     return true;
// }
bool MakeInputData::WideLaneJudge(const std::vector<Line>& line_out,  const std::vector<LaneMappingMemory>& lane_mapping_memories, std::map<uint64_t, std::vector<uint64_t>>& wide_lane_position_id_map){
    //step 1 , 用lane_mapping_memories的，即源于lane_mapping_infos_的切好的信息来绑定line_out, 一段路包含出路口和进路口， 第一段只有进路口，之后都是 出+进；
    wide_lane_position_id_map.clear();//Key是第几段bunch ,value 是positionID, 保存的是超宽车道的positionID
    std::unordered_map<uint32_t,BunchLines> bunch_map{};
    GetBunchLines(line_out,lane_mapping_memories,bunch_map);
    uint64_t bunch_num = 0;
    for(auto bunch : bunch_map){
        //选出每个bunch最长的线作为基准线
        int base_line_index = BunchProcessStep1(line_out, bunch.second);
        // std::cout << __FILE__ << "," << __LINE__ << ","<< "section: "<<bunch.first<< ", base_line_index: "  << base_line_index<<std::endl;
        //取基准线上的点对应的轨迹点，计算所在位置的 超宽车道 
        std::vector<uint64_t> wide_lane_position_id_vec{};
        BunchProcessStep2(line_out, base_line_index, bunch.second, wide_lane_position_id_vec);
        wide_lane_position_id_map.emplace(std::make_pair(bunch_num, wide_lane_position_id_vec));
        bunch_num ++;
    }
    return true;
}

int MakeInputData::BunchProcessStep1(const std::vector<Line>& line_out, const BunchLines& bunch){
    //计算长度，选出最长
    // std::cout << __FILE__ << "," << __LINE__ << ","<< "BunchProcessStep1: "  << std::endl;
    double max_length = 0;
    int max_leangth_index = 0;
    for(auto index:bunch.lint_out_indexes){
        if(index>=0 && index<line_out.size()){
            std::vector<Point2d> points = line_out[index].points;
            double line_len = 0;
            for(int p_i = 1;p_i<points.size();p_i++){
                line_len += calculateDistanceGC02(points[p_i-1].x, points[p_i-1].y, 0.0, points[p_i].x,points[p_i].y, 0.0);
            }
            if(line_len>max_length){
                max_length = line_len;
                max_leangth_index = index;
            }
        }
    }
    return max_leangth_index;
}

bool MakeInputData::BunchProcessStep2(const std::vector<Line>& line_out, int base_line_index, const BunchLines& bunch, std::vector<uint64_t>& wide_lane_position_id_vec){
    //  std::cout << __FILE__ << "," << __LINE__ << ","<< "BunchProcessStep2: "  << std::endl;
    //把线都转到直角坐标系下
    std::map<int, std::vector<Point2Dd>> line_cartesian_map{};//key 是line_out的索引，value是线的直角坐标
    auto index_vec = bunch.lint_out_indexes;
    for(auto index:index_vec){
        if(index>=0 && index<line_out.size()){
            std::vector<Point2Dd> line{};
            DoublePosePoint ori_point_wgs84(bunch.ori_point_wgs84.x,bunch.ori_point_wgs84.y, 0.0, bunch.ori_point_wgs84.heading, 0.0,0.0);
            // LineWGS84ToBody(line_out[index].points, ori_point_wgs84, line);
            for (const auto& point : line_out[index].points) {
                 Point2Dd point_body ,point_wgs84;
                 point_wgs84.x = point.x;
                 point_wgs84.y = point.y;
                 WGS84ToBody(point_wgs84, ori_point_wgs84, point_body);
                 line.emplace_back(point_body);
            }
            line_cartesian_map.emplace(std::make_pair(index,line));
        }
    }

    // 建线的 起始 终止pos id
    std::map<int, std::pair<uint64_t,uint64_t>> line_pos_map{};//key 是line_out的索引，value是线的起始和终止的position id
    for(auto iter: line_cartesian_map){
        if(iter.first<line_out.size() && line_out[iter.first].traj_line_ids.size()>0){
            uint64_t start_id = line_out[iter.first].traj_line_ids.begin()->first;
            uint64_t end_id = line_out[iter.first].traj_line_ids.rbegin()->first;
            line_pos_map.emplace(std::make_pair(iter.first,std::make_pair(start_id, end_id)));
        }
    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< " line_pos_map.size(): " <<line_pos_map.size() << std::endl;
    for(auto iter:line_pos_map){
        std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out index: " <<iter.first<<" , start_id: " <<iter.second.first<<" , end_id: " <<iter.second.second<< std::endl;
    }
#endif     
    
    double min_negative_l = -255;
    double min_positive_l = 255;
    wide_lane_position_id_vec.clear();//key 是positionID, 保存的是超宽车道的positionID
    if(base_line_index<line_out.size()){
        // 对基准线上的positionID， 看是否落在每条线的id范围内
        int start_index = 4;
        int end_index = line_out[base_line_index].traj_line_ids.size()-4;
        int count = 0;
        for(auto pos: line_out[base_line_index].traj_line_ids){
            if(count<start_index || count> end_index){
                count++;
                //为了去除首尾的区间
                continue;
            }
            min_negative_l = -255;
            min_positive_l = 255; 
            bool find_left_line = false;
            bool find_right_line = false;    
            int min_left_lane_id = -1;
            int min_right_lane_id = -1;
#ifdef PRO_DEAL   
            std::cout << __FILE__ << "," << __LINE__ << ","<< " position ID : " <<pos.first<<std::endl;   
#endif   
            for(auto line_pos:line_pos_map){//对于bunch的所有线
                //pos.first是线上点对应的轨迹id
                if(pos.first>= line_pos.second.first && pos.first<= line_pos.second.second){
                    //在范围内，计算轨迹到线的l
                    if(traj_points_position_map_.find(pos.first) != traj_points_position_map_.end()){
                        Point2Dd traj_point_wgs84(traj_points_position_map_[pos.first].x, traj_points_position_map_[pos.first].y); 
                        Point2Dd traj_body_point_xy;
                        DoublePosePoint ori_point_wgs84(bunch.ori_point_wgs84.x,bunch.ori_point_wgs84.y, 0.0, bunch.ori_point_wgs84.heading, 0.0,0.0);
                        WGS84ToBody(traj_point_wgs84, ori_point_wgs84, traj_body_point_xy);  //和line_cartesian_map 都是基于 bunch.ori_point                   
                        //直角坐标的线中找，计算l
                        if(line_cartesian_map.find(line_pos.first) != line_cartesian_map.end()){
                            PointSLd sl_temp;
                            int sign = 1;
                            CalPointSLBodyCoordinate(line_cartesian_map[line_pos.first], traj_body_point_xy, sl_temp);
#ifdef PRO_DEAL
                            std::cout << __FILE__ << "," << __LINE__ << ","<< " sl_temp: " <<sl_temp.l<<" , line_id: "<<line_out[line_pos.first].id<<std::endl;
#endif                            
                            if(sl_temp.l<= 0 && sl_temp.l>min_negative_l){
                                min_negative_l = sl_temp.l;
                                find_left_line = true;
                                min_left_lane_id = line_out[line_pos.first].id;
                            }
                            if(sl_temp.l> 0 && sl_temp.l<min_positive_l){
                                min_positive_l = sl_temp.l;
                                find_right_line = true;
                                min_right_lane_id = line_out[line_pos.first].id;
                            }
                        }
                    }             
                }
            }
#ifdef PRO_DEAL            
            std::cout << __FILE__ << "," << __LINE__ << ","<< " min_positive_l : "<<min_positive_l <<" ,min_negative_l: "<<min_negative_l  << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << ","<< " min_left_lane_id : "<<min_left_lane_id <<" ,min_right_lane_id: "<<min_right_lane_id  << std::endl;
#endif
            if(find_left_line && find_right_line){
                //左右车道线都找到了，超过4m,认为所超宽

                if((std::abs(min_positive_l)+std::abs(min_negative_l)) >= 4.2){
                    wide_lane_position_id_vec.push_back(pos.first);
                }
            } 
            count++;           
        }
    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< " wide_lane_position_id_vec.size() : "<<wide_lane_position_id_vec.size()  << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< " wide_lane_position_id_vec. : ";
    for(auto id:wide_lane_position_id_vec){
        std::cout<< " ,"<<id;
    }
    std::cout<< std::endl;
#endif
    
    return true;
}

bool MakeInputData::OutputWideLane(const std::map<uint64_t, std::vector<uint64_t>>& wide_lane_position_id_map, std::vector<LaneMappingMemory>& lane_mapping_memories){
    //填充输出接口
    for(auto wide_lane_vec: wide_lane_position_id_map){
        int start_pos_id = 0;
        int end_pos_id = 0;
        if(wide_lane_vec.second.size()==1){
            //zhi有一个点，那么前后填充一点
            start_pos_id = std::max(static_cast<int>(wide_lane_vec.second.front())-5, 0);
            end_pos_id = std::max(static_cast<int>(wide_lane_vec.second.front())+5, 0);
        }else if(wide_lane_vec.second.size()>1){
            start_pos_id = wide_lane_vec.second.front();
            end_pos_id = *(wide_lane_vec.second.rbegin());
        }else{
            continue;
        }
        for(auto& lane:lane_mapping_memories){
            int seg_start = lane.TrajHistory.front().PositionID;
            int seg_end = lane.TrajHistory.rbegin()->PositionID;
            if(end_pos_id <= seg_end && end_pos_id>= seg_start && start_pos_id>= seg_start && start_pos_id <= seg_end){
                //都落在一段里
                for(auto& point: lane.TrajHistory){
                    if(point.PositionID<=end_pos_id && point.PositionID>=start_pos_id){
                        point.is_split = true;
                    }
                }                    
            }else if(end_pos_id <= seg_end && end_pos_id >= seg_start){
                //后半段在区间里
                for(auto& point: lane.TrajHistory){
                    if(point.PositionID<=end_pos_id){
                        point.is_split = true;
                    }
                }
            }else if(start_pos_id <= seg_end && start_pos_id >= seg_start){
                //前半段落在区间里
                for(auto& point: lane.TrajHistory){
                    if(point.PositionID>=start_pos_id){
                        point.is_split = true;
                    }
                 }                    
            }
        }
    }

    return true;

}

bool MakeInputData::GetBunchLines(const std::vector<Line>& line_out,  const std::vector<LaneMappingMemory>& lane_mapping_memories, std::unordered_map<uint32_t,BunchLines>& bunch_map){
    //  std::cout << __FILE__ << "," << __LINE__ << ","<< "GetBunchLines: "  << std::endl;
    //std::unordered_map<uint32_t,std::vector<uint32_t>> bunch_map ; key-第几个bunch ；value-line_out的index
    //step1 构建 start，end position id
    bunch_map.clear();
    uint32_t section_num = 0;
    bool find_enter_f= false;
    bool find_exit_f = false;
    BunchLines bunch_temp{};
    bunch_temp.lint_out_indexes.clear();
    uint64_t offset = 30;
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< "lane_mapping_memories.size(): " <<lane_mapping_memories.size() << std::endl;
#endif
    for(int i =0; i<lane_mapping_memories.size(); i++){
        auto lanes = lane_mapping_memories[i];
#ifdef PRO_DEAL
        std::cout << __FILE__ << "," << __LINE__ << ","<< "lane_mapping_memories[ "<<i<<" ].TrajHistory.size() " <<lane_mapping_memories[i].TrajHistory.size() << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "lane_mapping_memories[ "<<i<<" ].TrajHistory.front().positionID: " <<lane_mapping_memories[i].TrajHistory.front().PositionID << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "lane_mapping_memories[ "<<i<<" ].TrajHistory.back().positionID: " <<lane_mapping_memories[i].TrajHistory.back().PositionID << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "lanes.TrajHistory.front().RoadType " <<(int)lanes.TrajHistory.front().RoadType <<",lanes.TrajHistory.back().RoadType " <<(int)lanes.TrajHistory.back().RoadType << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "section_num start  " <<section_num  << std::endl;
#endif
        if(section_num ==0){
            //对于第一个路段，只有进路口
            if(lanes.TrajHistory.size()>0 && lanes.TrajHistory.back().RoadType == 1){              
                bunch_temp.start_position_id = 0;
                bunch_temp.end_position_id = lanes.TrajHistory.back().PositionID+offset;
                bunch_temp.ori_point_wgs84.x = lanes.TrajHistory.front().PosLgt;
                bunch_temp.ori_point_wgs84.y = lanes.TrajHistory.front().PosLat;
                bunch_temp.ori_point_wgs84.heading = lanes.link_heading;
                bunch_map.emplace(std::make_pair(section_num,bunch_temp));
                section_num++;
            }
        }else{
            //对于中间的路段，出路口+进路口
            if(lanes.TrajHistory.size()>0 && find_exit_f == false && lanes.TrajHistory.front().RoadType == 3){
                find_exit_f =true;
                bunch_temp.start_position_id = std::max(lanes.TrajHistory.front().PositionID-offset,static_cast<uint64_t>(0));
                bunch_temp.ori_point_wgs84.x = lanes.TrajHistory.front().PosLgt;
                bunch_temp.ori_point_wgs84.y = lanes.TrajHistory.front().PosLat;
                bunch_temp.ori_point_wgs84.heading = lanes.link_heading;
            }else if(find_exit_f == true  && lanes.TrajHistory.back().RoadType == 1){
                bunch_temp.end_position_id = lanes.TrajHistory.back().PositionID+offset;
                bunch_map.emplace(std::make_pair(section_num,bunch_temp));
                section_num++;
                find_exit_f = false;
            }

            //最后一段路
            if(i==lane_mapping_memories.size()-1 && find_exit_f == true){                  
                bunch_temp.end_position_id = lanes.TrajHistory.back().PositionID+offset;
                bunch_map.emplace(std::make_pair(section_num,bunch_temp));
            }
        }
        
        // std::cout << __FILE__ << "," << __LINE__ << ","<< "section_num loop end " <<section_num  << std::endl;
    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< " bunch_map.size(): " <<bunch_map.size() << std::endl;
    for(auto iter:bunch_map){
        std::cout << __FILE__ << "," << __LINE__ << ","<< " section: " <<iter.first<<" , start_id: " <<iter.second.start_position_id<<" , end_id: " <<iter.second.end_position_id<< std::endl;
    }

    std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out.size() "<<line_out.size() << std::endl;
#endif    
    //step2 填充bunch_map的lint_out_indexes， 遍历line_out， 将每条线的首尾position Id落在 bunch_map的start_position_id-end_position_id之间的，认为是同一段
    for(int i =0; i<line_out.size(); i++){
#ifdef PRO_DEAL
        std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out[ "<<i<<" ].id " <<(int)line_out[i].id << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out.points.size() "<<line_out[i].points.size() << std::endl;
#endif
        int line_start_pos_id = -1;
        int line_end_pos_id = -1;
        if(line_out[i].traj_line_ids.size()>1){
            line_start_pos_id = line_out[i].traj_line_ids.begin()->first;
            line_end_pos_id = line_out[i].traj_line_ids.rbegin()->first;
        } 
#ifdef PRO_DEAL
        std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out[ "<<i<<" ] line_start_pos_id " <<line_start_pos_id << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< "line_out[ "<<i<<" ] line_end_pos_id " <<line_end_pos_id << std::endl;
#endif
        if(line_start_pos_id>=0 && line_end_pos_id>= 0){
          for(auto& bunch:bunch_map){
            if(line_start_pos_id >= bunch.second.start_position_id && line_end_pos_id <= bunch.second.end_position_id){
                 bunch.second.lint_out_indexes.push_back(i);
                 break;
            }
          }
        }

    }
#ifdef PRO_DEAL
    std::cout << __FILE__ << "," << __LINE__ << ","<< " bunch_map.size(): " <<bunch_map.size() << std::endl;
    for(auto iter:bunch_map){
        std::cout << __FILE__ << "," << __LINE__ << ","<< " section: " <<iter.first<<" , start_id: " <<iter.second.start_position_id<<" , end_id: " <<iter.second.end_position_id<< std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<< " index: " ;
        for(auto subiter: iter.second.lint_out_indexes){
           std::cout<<", "<<subiter;
        }
        std::cout <<std::endl;
    }
#endif  
    return true;  
}
}  // namespace MapFusion
}  // namespace CommuteMode
