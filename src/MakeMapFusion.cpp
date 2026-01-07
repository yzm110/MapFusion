#include "MakeMapFusion.h"

namespace CommuteMode {
namespace MapFusion {
MemoryFileTimes MakeMapFusion::memory_file_times_;

bool MakeMapFusion::Execute(const std::shared_ptr<MakeInputData>& make_input_data,
                            const std::vector<Line>& line_out) {
  map_common_tool_ = MapCommonTool::GetInstance();
  map_common_tool_->GetIntersectionInfos(intersection_infos_);
  map_common_tool_->GetPositionInfos(position_infos_);
  if (false == map_common_tool_->GetLinkDataInfos(link_data_infos_)) {
    // return false;
  }
  map_common_tool_->GetMapHdnvInfos(map_hdnv_infos_);
  map_common_tool_->GetLinkList(link_list_);
  //  init variable
  max_link_id_ = 0;
  if (false == ChangeLinkList()) {
    // return false;
  }

  // process
  if (false == MakeCenter(make_input_data)) {
    // return false;
  }

  if (false == MakePosition(make_input_data, line_out)) {
    return false;
  }

  if (false == MakeConnect()) {
    return false;
  }

  if (false == MakeOtherField()) {
    return false;
  }

  return true;
}

bool MakeMapFusion::cmp(LinkMemory& data1, LinkMemory& data2) {
  if (data1.new_link_id < data2.new_link_id) {
    return true;
  } else {
    return false;
  }
}

bool MakeMapFusion::ChangeLinkList() {
  // if (map_hdnv_infos_.empty()){
  //   return false;
  // }

  // uint32_t rout_id = 0;
  // for (auto& hdnv : map_hdnv_infos_){
  //   if (0 == hdnv.route_id){
  //     //invalid
  //     continue;
  //   }

  //   if (0 == rout_id || rout_id != hdnv.route_id){
  //     //clear
  //     link_list_.clear();
  //   }

  //   if (hdnv.linkid_list.size() > 0){
  //     int start_idx = 0;
  //     rout_id = hdnv.route_id;
  //     std::vector<uint32_t>::iterator iter = std::find(link_list_.begin(), link_list_.end(),
  //                                                      hdnv.linkid_list[start_idx]);
  //     while (iter != link_list_.end() && start_idx < hdnv.linkid_list.size()){
  //       if (*iter != hdnv.linkid_list[start_idx]){
  //         start_idx = 0;
  //         break;
  //       }
  //       iter += 1;
  //       start_idx += 1;
  //     }
  //     link_list_.insert(link_list_.end(), hdnv.linkid_list.begin() + start_idx, hdnv.linkid_list.end());
  //   }
  // }

  // //get MAX link_id
  // max_link_id_ = *(std::max_element(link_list_.begin(), link_list_.end()));
  // uint32_t temp_link_id = *(std::max_element(link_data_infos_.begin(), link_data_infos_.end(), cmp));
  // max_link_id_ = std::max(max_link_id_, temp_link_id);

  // if (fasle == MakeNewLinkList()){
  //   return false;
  // }

  return true;
}

bool MakeMapFusion::MakeCenter(const std::shared_ptr<MakeInputData>& make_input_data) {
  memory_lane_mapping_infos_.clear();
  memory_lane_mapping_infos_ = make_input_data->GetInputLaneMappingMemories();
  std::cout << __FILE__ << __LINE__ << "memory_lane_mapping_infos_.size(): " << memory_lane_mapping_infos_.size() << std::endl;
  size_t memory_lane_mapping_infos_size = memory_lane_mapping_infos_.size();

  if (memory_lane_mapping_infos_size < 2) {
      std::cout << __FILE__ << __LINE__ << "memory lane mapping num not enough" << std::endl;
      return false;
  }

  std::unordered_map<uint32_t, uint64_t>road_id_line_id_list{};
  memory_cross_lane_infos_.clear();
  uint64_t line_id = 1;
  uint16_t intersection_id = 1;
  for (size_t roadidx = 0; roadidx < memory_lane_mapping_infos_size - 1; roadidx++) {
    uint8_t cur_lane_road_type = GetMostFrequentRoadType(memory_lane_mapping_infos_[roadidx]);
    uint8_t next_lane_road_type = 0;
    int next_roadidx = 0;
    // 找到一个路口的出口及进口，一一匹配
    for (size_t next_roadidx_tmp = roadidx + 1; next_roadidx_tmp < memory_lane_mapping_infos_size - 1;
        next_roadidx_tmp++) {
      next_lane_road_type = GetMostFrequentRoadType(memory_lane_mapping_infos_[next_roadidx_tmp]);
      if (next_lane_road_type == 3) {
        next_roadidx = next_roadidx_tmp;
        break;
      }
    }

    // 如果路口的出口和进口都没有轨迹点，直接跳过
    if (true == memory_lane_mapping_infos_[roadidx].TrajHistory.empty() ||
        true == memory_lane_mapping_infos_[next_roadidx].TrajHistory.empty()) {
      return false;
    }
    uint8_t enter_lane_id = memory_lane_mapping_infos_[roadidx].exit_ego_lane_id - 1 >= 0 ?
                          memory_lane_mapping_infos_[roadidx].exit_ego_lane_id - 1 : 0;
    uint8_t exit_lane_id = memory_lane_mapping_infos_[next_roadidx].enter_ego_lane_id - 1 >= 0 ?
            memory_lane_mapping_infos_[next_roadidx].enter_ego_lane_id - 1 : 0;
    // std::cout << "exit_ego_lane_id: " << int(memory_lane_mapping_infos_[roadidx].exit_ego_lane_id)
    //           << " enter_ego_lane_id: " << int(memory_lane_mapping_infos_[next_roadidx].enter_ego_lane_id) << std::endl;
    // std::cout << __FILE__ << __LINE__ << "roadidx: " << int(roadidx) << " next_roadidx: " << int(next_roadidx)
    //           << "; cur_lane_road_type: " << int(cur_lane_road_type)
    //           << " next_lane_road_type: " << int(next_lane_road_type) << std::endl;

    // 匹配好了出口和进口才能继续生成中心线
    if (cur_lane_road_type == 1 && next_lane_road_type == 3) {
      std::vector<Point2Dd> intersection_traj_points_wgs84{}, intersection_traj_points_body{};
      DoublePosePoint intersection_ego_pose_point{};
      std::vector<double> intersection_traj_points_curv{};
      double intersection_traj_points_radius_avg = 0.0, intersection_traj_points_radius_sum = 0.0;
      // 获取路口的历史轨迹，后续用来计算路口内轨迹点的平均曲率
      for (size_t idx = roadidx + 1; idx < next_roadidx; idx++) {
        std::vector<Position> traj_history = memory_lane_mapping_infos_[idx].TrajHistory;
        if (true == traj_history.empty()) {
          // return false;
        }
        for (const auto& point : traj_history) {
          intersection_traj_points_wgs84.emplace_back(Point2Dd{point.PosLgt, point.PosLat});
          // 获取一个基准点
          if (intersection_ego_pose_point.x < 10) {
            intersection_ego_pose_point.x = point.PosLgt;
            intersection_ego_pose_point.y = point.PosLat;
            intersection_ego_pose_point.yaw = point.Heading;
          }
        }
      }
      LineWGS84ToBody(intersection_traj_points_wgs84, intersection_ego_pose_point, intersection_traj_points_body);
      intersection_traj_points_curv = ComputeCurvatures(intersection_traj_points_body);

      int intersection_traj_points_body_size = intersection_traj_points_body.size();
      int curv_point_num = 0;
      for (int idx = 0; idx < intersection_traj_points_body_size; idx++) {
        double curv = 1 / std::abs(intersection_traj_points_curv[idx]);
        if (curv < 300) {
          curv_point_num++;
          intersection_traj_points_radius_sum += curv;
        }
      }
      intersection_traj_points_radius_avg = intersection_traj_points_radius_sum / curv_point_num;
      // std::cout << __FILE__ << __LINE__ << "curv_point_num: " << curv_point_num << std::endl;
      // std::cout << __FILE__ << __LINE__ << "intersection_traj_points_body_size: "
      //           << intersection_traj_points_body_size << std::endl;
      // std::cout << __FILE__ << __LINE__ << "intersection_traj_points_radius_avg: "
      // << intersection_traj_points_radius_avg << std::endl;

      // 当前是入口下一个是出口的场景，暂时没用
      uint8_t cur_lane_type = GetMostFrequentLaneType(memory_lane_mapping_infos_[roadidx],
                                                      cur_lane_road_type);  // 进入路口的link是直行还是左右转

      // 获取进入和退出路口的轨迹历史
      std::vector<Position>  enter_traj_history = memory_lane_mapping_infos_[roadidx].TrajHistory;
      DoublePosePoint enter_ego_pose_point{};
      enter_ego_pose_point.x = enter_traj_history.back().PosLgt;
      enter_ego_pose_point.y = enter_traj_history.back().PosLat;
      enter_ego_pose_point.yaw = enter_traj_history.back().Heading;

      // 获取进入和退出路口的边界线
      std::vector<std::vector<LPoint>> enter_boundry_lines_wgs84 = memory_lane_mapping_infos_[roadidx].LaneData;
      std::vector<std::vector<LPoint>> exit_boundry_lines_wgs84 = memory_lane_mapping_infos_[next_roadidx].LaneData;
      std::vector<CrossLaneInfo> enter_center_lines_body{}, exit_center_lines_body{}, enter_center_lines_wgs84{},
        mid_center_lines_wgs84{}, exit_center_lines_wgs84{};

      // 记忆路线中的第几个路口
      intersection_id++;
      // 生成中心线
      BoundrysGenerateCenterLines(enter_boundry_lines_wgs84, intersection_id, true,
                                enter_ego_pose_point, enter_center_lines_body);
      BoundrysGenerateCenterLines(exit_boundry_lines_wgs84, intersection_id, false,
                                enter_ego_pose_point, exit_center_lines_body);

      // std::cout << __FILE__ << __LINE__ << "cur_lane_type: " << int(cur_lane_type) << std::endl;
      // std::cout << __FILE__ << __LINE__ << "enter_center_lines_body: "
      //           << enter_center_lines_body.size() << std::endl;
      // std::cout << __FILE__ << __LINE__ << "exit_center_lines_body: " << exit_center_lines_body.size() << std::endl;

      // std::cout << __FILE__ << __LINE__ << "enter_center_lines_body:{ " << std::endl;
      // for (const auto& exit_center_line_points_body : enter_center_lines_body) {
      //   for (const auto& point : exit_center_line_points_body.geometry) {
      //     std::cout << "[" << point.x << ", " << point.y << "], ";
      //   }
      //   std::cout << "}" << std::endl;
      // }
      // std::cout << __FILE__ << __LINE__ << "exit_center_lines_body:{ " << std::endl;
      // for (const auto& exit_center_line_points_body : exit_center_lines_body) {
      //   for (const auto& point : exit_center_line_points_body.geometry) {
      //     std::cout << "[" << point.x << ", " << point.y << "], ";
      //   }
      //   std::cout << "}" << std::endl;
      // }

      // 生成路口中心线并补全进入和退出车道中心线的信息
      bool is_bigcurv_intersection = false;
      if (3 * curv_point_num > intersection_traj_points_body_size &&
        intersection_traj_points_radius_avg < 100) {
        is_bigcurv_intersection = true;
      }
      bool is_right_has_lane = false;
      // std::cout << __FILE__ << __LINE__ << "intersection_id: " << int(intersection_id) << std::endl;
      GenerateCenterLines(enter_lane_id, exit_lane_id, enter_ego_pose_point, cur_lane_type,
                          is_bigcurv_intersection, intersection_id,
                          enter_center_lines_body, exit_center_lines_body, mid_center_lines_wgs84,
                          is_right_has_lane);

      for (const auto& enter_center_line_body : enter_center_lines_body) {
        CrossLaneInfo enter_center_line_wgs84;
        enter_center_line_wgs84 = enter_center_line_body;
        std::vector<Point2Dd> enter_center_line_wgs84_points{}, enter_center_line_body_points{};
        enter_center_line_body_points = enter_center_line_body.geometry;
        LineBodyToWGS84(enter_center_line_body_points, enter_ego_pose_point, enter_center_line_wgs84_points);
        enter_center_line_wgs84.geometry = enter_center_line_wgs84_points;
        enter_center_lines_wgs84.emplace_back(enter_center_line_wgs84);
      }

      for (const auto& exit_center_line_body : exit_center_lines_body) {
        CrossLaneInfo exit_center_line_wgs84;
        exit_center_line_wgs84 = exit_center_line_body;
        std::vector<Point2Dd> exit_center_line_wgs84_points{}, exit_center_line_body_points{};
        exit_center_line_body_points = exit_center_line_body.geometry;
        LineBodyToWGS84(exit_center_line_body_points, enter_ego_pose_point, exit_center_line_wgs84_points);
        exit_center_line_wgs84.geometry = exit_center_line_wgs84_points;
        exit_center_lines_wgs84.emplace_back(exit_center_line_wgs84);
      }

      uint32_t link_id = 0;
      for (const auto& enter_crosslane_info : enter_center_lines_wgs84) {
        link_id = enter_crosslane_info.cross_id;
        memory_cross_lane_infos_.emplace_back(enter_crosslane_info);
      }
      for (const auto& mid_crosslane_info : mid_center_lines_wgs84) {
        memory_cross_lane_infos_.emplace_back(mid_crosslane_info);
      }
      for (const auto& exit_crosslane_info : exit_center_lines_wgs84) {
        memory_cross_lane_infos_.emplace_back(exit_crosslane_info);
      }

      // 路口附近轨迹点最近线的id生成
      std::vector<Point2Dd> enter_traj_points_wgs84{}, enter_traj_points_body{}, exit_traj_points_wgs84{},
                            exit_traj_points_body{};
      for (const auto& point : memory_lane_mapping_infos_[roadidx].TrajHistory) {
          enter_traj_points_wgs84.emplace_back(Point2Dd{point.PosLgt, point.PosLat});
      }
      for (const auto& point : memory_lane_mapping_infos_[next_roadidx].TrajHistory) {
          exit_traj_points_wgs84.emplace_back(Point2Dd{point.PosLgt, point.PosLat});
      }

      LineWGS84ToBody(enter_traj_points_wgs84, enter_ego_pose_point, enter_traj_points_body);
      LineWGS84ToBody(exit_traj_points_wgs84, enter_ego_pose_point, exit_traj_points_body);

      // 进入入口轨迹点最近的线的id生成
      uint64_t enter_start_nearest_line_id = 0, enter_end_nearest_line_id = 0;
      enter_start_nearest_line_id = FindNearestLineId(enter_traj_points_body.front(), enter_center_lines_body);
      enter_end_nearest_line_id = FindNearestLineId(enter_traj_points_body.back(), enter_center_lines_body);
      if (enter_start_nearest_line_id == enter_end_nearest_line_id) {
        for (size_t idx = 0; idx < memory_lane_mapping_infos_[roadidx].TrajHistory.size(); idx++) {
          memory_lane_mapping_infos_[roadidx].TrajHistory[idx].RelateLineID.emplace_back(enter_start_nearest_line_id);
        }
      } else {
        for (size_t idx = 0; idx < memory_lane_mapping_infos_[roadidx].TrajHistory.size(); idx++) {
          uint64_t nearest_line_id = 0;
          nearest_line_id = FindNearestLineId(enter_traj_points_body[idx], enter_center_lines_body);
          memory_lane_mapping_infos_[roadidx].TrajHistory[idx].RelateLineID.emplace_back(nearest_line_id);
        }
      }
      // 退出入口轨迹点最近的线的id生成
      uint64_t exit_start_nearest_line_id = 0, exit_end_nearest_line_id = 0;
      exit_start_nearest_line_id = FindNearestLineId(exit_traj_points_body.front(), exit_center_lines_body);
      exit_end_nearest_line_id = FindNearestLineId(exit_traj_points_body.back(), exit_center_lines_body);
      if (exit_start_nearest_line_id == exit_end_nearest_line_id) {
        for (size_t idx = 0; idx < memory_lane_mapping_infos_[next_roadidx].TrajHistory.size(); idx++) {
          memory_lane_mapping_infos_[next_roadidx].TrajHistory[idx].RelateLineID.
          emplace_back(exit_start_nearest_line_id);
        }
      } else {
        for (size_t idx = 0; idx < memory_lane_mapping_infos_[next_roadidx].TrajHistory.size(); idx++) {
          uint64_t nearest_line_id = 0;
          nearest_line_id = FindNearestLineId(exit_traj_points_body[idx], exit_center_lines_body);
          memory_lane_mapping_infos_[next_roadidx].TrajHistory[idx].RelateLineID.emplace_back(nearest_line_id);
        }
      }

      // 路口内轨迹点最近线的id生成
      if (true == is_right_has_lane && mid_center_lines_wgs84.size() > 1) {
        for (size_t idx = roadidx + 1; idx < next_roadidx; idx++) {
          for (size_t idx1 = 0; idx1 < memory_lane_mapping_infos_[idx].TrajHistory.size(); idx1++) {
            memory_lane_mapping_infos_[idx].TrajHistory[idx1].RelateLineID.emplace_back(
              mid_center_lines_wgs84[1].id);
          }
        }
      } else if (mid_center_lines_wgs84.size() > 0) {
        for (size_t idx = roadidx + 1; idx < next_roadidx; idx++) {
          for (size_t idx1 = 0; idx1 < memory_lane_mapping_infos_[idx].TrajHistory.size(); idx1++) {
            memory_lane_mapping_infos_[idx].TrajHistory[idx1].RelateLineID.emplace_back(
              mid_center_lines_wgs84[0].id);
          }
        }
      }

      // 生成了一个路口的中心线之后，跳过这个路口，继续下一个路口
      roadidx = next_roadidx;
    } else {
      continue;
    }
  }

  std::cout << __FILE__ << __LINE__ << "Make Center finish" << std::endl;


  if (false == MakeRoadSectionCenter()) {
    return false;
  }

  if (false == MakeSplitCenter()) {
    return false;
  }

  if (false == MakeMergeCenter()) {
    return false;
  }

  if (false == MakeIntersectionCenter()) {
    return false;
  }

  return true;
}

bool MakeMapFusion::MakeRoadSectionCenter() {
  return true;
}

bool MakeMapFusion::MakeSplitCenter() {
  return true;
}

bool MakeMapFusion::MakeMergeCenter() {
  return true;
}

bool MakeMapFusion::MakeIntersectionCenter() {
  return true;
}

bool MakeMapFusion::MakeConnect() {
  return true;
}

bool MakeMapFusion::MakeOtherField() {
  return true;
}

bool MakeMapFusion::MakePosition(const std::shared_ptr<MakeInputData>& make_input_data,
                                const std::vector<Line>& line_out) {
  lane_mapping_traj_infos_.clear();
  traj_points_position_map_.clear();
std::cout << __FILE__ << __LINE__ << "memory_lane_mapping_infos_.size(): " << memory_lane_mapping_infos_.size() << std::endl;
  // 记录所有轨迹点的原始positionid及对应的位置信息
  for (const auto& lane_mapping_info : memory_lane_mapping_infos_) {
    if (false == lane_mapping_info.TrajHistory.empty()) {
      TrajInfo traj_info;
      traj_info.road_id = lane_mapping_info.LinkId;
      // traj_info.nearest_line_id = 0;
      traj_info.positions = lane_mapping_info.TrajHistory;
      lane_mapping_traj_infos_.emplace_back(traj_info);
      for (const auto& position : lane_mapping_info.TrajHistory) {
        DoublePosePoint pose_point{};
        pose_point.x = position.PosLgt;
        pose_point.y = position.PosLat;
        pose_point.yaw = position.Heading;
        traj_points_position_map_.emplace(position.PositionID, pose_point);
      }
    }
  }


  std::vector<LaneChangePointInfo> lane_change_points{};
  FindLaneChangePoints(line_out, &lane_change_points);

  std::sort(lane_change_points.begin(), lane_change_points.end(),
            [](const LaneChangePointInfo& a, const LaneChangePointInfo& b) {
              return a.point_raw_id < b.point_raw_id;
            });

  std::cout << __FILE__ << __LINE__ << "lane_change_points size: " << lane_change_points.size();
  for (const auto& lane_change_point : lane_change_points) {
    std::cout << "lane_change_point.point_raw_id: " << lane_change_point.point_raw_id
              << " lane_change_point.is_change: " << lane_change_point.is_change
              << " lane_change_point.change_dir: " << int(lane_change_point.change_dir) << std::endl;
  }

  uint32_t raw_point_num = 0;
  for (const auto& lane_mapping_traj_info : lane_mapping_traj_infos_) {
    raw_point_num += lane_mapping_traj_info.positions.size();
  }
  // std::cout << __FILE__ << __LINE__ << "raw_point_num: " << raw_point_num << std::endl;

  if (false == lane_mapping_traj_infos_.empty() && memory_file_times_.memory_postion_times == 0) {
    std::cout << "******** position info process start ***********" << std::endl;
    uint32_t new_id = 1;
    double total_distance = 0;
    // TrajInfo current_traj_info;
    // current_traj_info.road_id = lane_mapping_traj_infos_[0].road_id;
    // current_traj_info.nearest_line_id = lane_mapping_traj_infos_[0].nearest_line_id;

    for (const auto& lane_mapping_tra_info : lane_mapping_traj_infos_) {
        for (size_t idx = 0; idx < lane_mapping_tra_info.positions.size() - 2; idx++) {
            Position position = lane_mapping_tra_info.positions[idx];
            if (position.PosLgt >= 70.0 && position.PosLgt <= 138.0 &&
                position.PosLat >= 15.0 && position.PosLat <= 57.0) {
                TrajPointInfo traj_point_info{};
                traj_point_info.road_id = lane_mapping_tra_info.road_id;
                if (false == position.RelateLineID.empty()) {
                    traj_point_info.nearest_line_id = position.RelateLineID[0];
                } else {
                    traj_point_info.nearest_line_id = 0;
                }
                traj_point_info.position.LaneNumber = position.LaneNumber;
                traj_point_info.position.EgoInLane = position.EgoInLane;
                traj_point_info.position.RoadType = position.RoadType;
                traj_point_info.position.SpeedLimitMax = position.SpeedLimitMax;
                traj_point_info.position.PosTime = position.PosTime;
                traj_point_info.position.PositionID = position.PositionID;
                if (2 == position.RoadType) {
                  traj_point_info.position.StopLineOffset = 2000.0 * 100;
                } else {
                  traj_point_info.position.StopLineOffset = position.StopLineOffset;
                }
                traj_point_info.position.PosLat = position.PosLat;
                traj_point_info.position.PosLgt = position.PosLgt;
                traj_point_info.position.Heading = position.Heading;
                traj_point_info.position.LaneType = position.LaneType;
                traj_point_info.position.is_split = position.is_split;
                // 使用std::find_if查找id
                auto it = std::find_if(lane_change_points.begin(), lane_change_points.end(),
                                      [position_id = position.PositionID](const LaneChangePointInfo& point) {
                                          return point.point_raw_id == position_id;
                                      });
                if (it != lane_change_points.end() && 2 != position.RoadType) {
                  traj_point_info.lane_change_point_info = *it;
                  std::cout << __FILE__ << __LINE__ << "traj_point_info.lane_change_point_info.point_raw_id: "
                            << traj_point_info.lane_change_point_info.point_raw_id << std::endl;
                  // traj_point_info.lane_change_point_info.RoadType = position.RoadType;
                }
                if (false == position_semantic_infos_.empty()) {
                    Point2Dd  cur_point{}, cur_point_ego{};
                    DoublePosePoint pre_point{};
                    cur_point.x =  traj_point_info.position.PosLgt;
                    cur_point.y = traj_point_info.position.PosLat;
                    pre_point.x = position_semantic_infos_.back().position.x;
                    pre_point.y = position_semantic_infos_.back().position.y;
                    pre_point.yaw = position_semantic_infos_.back().heading;
                    WGS84ToBody(cur_point, pre_point, cur_point_ego);
                    double distance = Distance(0.0, 0.0, cur_point_ego.x, cur_point_ego.y) *100;
                    std::cout << __FILE__ << __LINE__ << "distance: " << distance << std::endl;

                    if (distance >= 100||
                        // (traj_point_info.position.StopLineOffset >
                        // lane_mapping_tra_info.positions[idx + 1].StopLineOffset &&
                        // lane_mapping_tra_info.positions[idx + 1].StopLineOffset < 1.0 &&
                        // traj_point_info.position.StopLineOffset > 1.0) ||
                        true == traj_point_info.lane_change_point_info.is_change) {
                    std::cout << __FILE__ << __LINE__ << "position.PositionID: " << position.PositionID << std::endl;
                      PositionPointInfo point_info{};
                      // 获取停止线前最后一个点的原始id及标志位
                      if (traj_point_info.position.StopLineOffset >
                        lane_mapping_tra_info.positions[idx + 1].StopLineOffset &&
                        lane_mapping_tra_info.positions[idx + 1].StopLineOffset < 1.0 &&
                        traj_point_info.position.StopLineOffset > 1.0) {
                          point_info.raw_position_point_id = lane_mapping_tra_info.positions[idx].PositionID;
                          point_info.is_stop_line_end = true;
                      }
                      // 将变道点信息赋值给当前点，后续保存到语义信息中
                      point_info.lane_change_point_info = traj_point_info.lane_change_point_info;

                      total_distance += distance;
                      point_info.id = new_id++;
                      point_info.path_offset = total_distance;
                      // 停止线距离最多只发2000米，没有停止线信息的默认为2000米
                      if (traj_point_info.position.StopLineOffset > 0.1 * 100
                          && traj_point_info.position.StopLineOffset < 2000.0 * 100
                          && false == traj_point_info.position.is_split) {
                        point_info.changepoint_offset =
                        static_cast<uint32_t>(traj_point_info.position.StopLineOffset / 100);
                        point_info.changepoint_type = 1;
                      } else if (true == traj_point_info.position.is_split) {
                        std::cout << __FILE__ << __LINE__ << "split_PositionID: " << traj_point_info.position.PositionID
                        << std::endl;
                        point_info.changepoint_type = 3;
                        point_info.changepoint_offset = 2000;
                      } else {
                        point_info.changepoint_offset = 2000;
                        point_info.changepoint_type = 0;
                      }

                      point_info.changepoint_dir = 0;
                      point_info.line_id = traj_point_info.nearest_line_id;
                      point_info.speed = traj_point_info.position.SpeedLimitMax;
                      point_info.heading = static_cast<float>(position.Heading);
                      point_info.position = {traj_point_info.position.PosLgt, traj_point_info.position.PosLat};
                      position_semantic_infos_.emplace_back(point_info);
                      traj_point_infos_.emplace_back(traj_point_info);
                    }
                } else {
                  std::cout << __FILE__ << __LINE__ << "position_semantic_infos_ is empty" << std::endl;
                    PositionPointInfo point_info{};
                    // total_distance += 0;
                    point_info.id = new_id++;
                    point_info.path_offset = 0;
                    if (traj_point_info.position.StopLineOffset > 0.1 * 100
                          && traj_point_info.position.StopLineOffset < 2000 * 100) {
                      point_info.changepoint_offset =
                      static_cast<uint32_t>(traj_point_info.position.StopLineOffset / 100);
                      point_info.changepoint_type = 1;
                    } else {
                      point_info.changepoint_offset = 2000;
                      point_info.changepoint_type = 0;
                    }
                    point_info.changepoint_dir = 0;
                    point_info.line_id = traj_point_info.nearest_line_id;
                    point_info.speed = traj_point_info.position.SpeedLimitMax;
                    point_info.heading = static_cast<float>(position.Heading);
                    point_info.position = {traj_point_info.position.PosLgt, traj_point_info.position.PosLat};
                    position_semantic_infos_.emplace_back(point_info);
                    traj_point_infos_.emplace_back(traj_point_info);
                }
            }
        }
    }
    size_t position_semantic_infos_size = position_semantic_infos_.size();
    for (size_t s_idx = 0; s_idx < position_semantic_infos_size - 1; s_idx++) {
      if (position_semantic_infos_[s_idx].changepoint_offset <
          position_semantic_infos_[s_idx + 1].changepoint_offset) {
            uint32_t marker_id = position_semantic_infos_[s_idx].id;
            for (int e_idx = s_idx; e_idx >= 0; --e_idx) {
                if (position_semantic_infos_[e_idx].cross_point != 0) {
                    break;
                }
                position_semantic_infos_[e_idx].cross_point = marker_id;
            }
        }
    }

    std::cout << __FILE__ << __LINE__ << "Start ProcessCrossLanePoints" << std::endl;
    ProcessCrossLanePoints(position_semantic_infos_);
    std::cout << __FILE__ << __LINE__ << "Start ProcessSplitLanePoints" << std::endl;
    ProcessSplitLanePoints(position_semantic_infos_);
    std::cout << __FILE__ << __LINE__ << "End ProcessMergeLanePoints" << std::endl;

    // postion_point_infos.shrink_to_fit();
    traj_point_infos_.shrink_to_fit();
    position_semantic_infos_.shrink_to_fit();
    memory_file_times_.memory_postion_times++;
    std::cout <<__FILE__ << __LINE__<< "position_semantic_infos_.size(): " << position_semantic_infos_.size()
              << std::endl;
    std::cout <<__FILE__ << __LINE__ << "******** position info process end ***********" << std::endl;
  }
  return true;
}

bool MakeMapFusion::GenerateCenterLine(const std::vector<Point2Dd> & enter_center_line_points_body,
                                      const std::vector<Point2Dd> & exit_center_line_points_body,
                                      const DoublePosePoint& ego_pose_point, const uint8_t& section_type,
                                      LineAttribute & intersection_center_line_points_wgs84) {
  std::vector<Point2Dd> intersection_center_line_body{};

  std::vector<float> enter_center_line_x{}, enter_center_line_y{}, exit_center_line_x{},
                    exit_center_line_y{}, intersection_center_line_x{}, intersection_center_line_y{};
  for (const auto& point : enter_center_line_points_body) {
    enter_center_line_x.emplace_back(static_cast<float>(point.x));
    enter_center_line_y.emplace_back(static_cast<float>(point.y));
  }
  for (const auto& point : exit_center_line_points_body) {
    exit_center_line_x.emplace_back(static_cast<float>(point.x));
    exit_center_line_y.emplace_back(static_cast<float>(point.y));
  }
  // if (section_type != 1 && section_type != 0) {
  if (0) {
    // std::cout << __FILE__ << __LINE__ << "section_type: " << int(section_type) << std::endl;
    CrossPlanning(enter_center_line_x, enter_center_line_y, exit_center_line_x,
              exit_center_line_y, intersection_center_line_x, intersection_center_line_y);

    for (int i = 0; i < intersection_center_line_x.size(); i++) {
      intersection_center_line_body.emplace_back(Point2Dd{static_cast<double>(intersection_center_line_x[i]),
                                                          static_cast<double>(intersection_center_line_y[i])});
    }
  } else if (enter_center_line_points_body.size() >= 4 &&
             exit_center_line_points_body.size() > 4) {
    // else if (section_type == 1 && enter_center_line_points_body.size() >= 2 &&
    //          exit_center_line_points_body.size() > 1) {
    // std::cout << __FILE__ << __LINE__ << "section_type: " << int(section_type) << std::endl;
    // std::cout << __FILE__ << __LINE__ << "enter_center_line_points_body: "
    //           << enter_center_line_points_body.size() << std::endl;
    // std::cout << __FILE__ << __LINE__
    //           << "exit_center_line_points_body: "
    //           << exit_center_line_points_body.size()
    //           << std::endl;
    // std::cout << __FILE__ << __LINE__ << "enter_center_line_points_body:{ ";
    // for(const auto& point : enter_center_line_points_body){
    //   std::cout << "[" << point.x << ", " << point.y << "], ";
    // }
    // std::cout << "}" << std::endl;
    // std::cout << __FILE__ << __LINE__ << "exit_center_line_points_body:{ ";
    // for(const auto& point : exit_center_line_points_body){
    //   std::cout << "[" << point.x << ", " << point.y << "], ";
    // }
    // std::cout << "}" << std::endl;


    intersection_center_line_body = GenerateInterpolatedLine(
        enter_center_line_points_body.back(), exit_center_line_points_body.front(), 2.5);
    Point2Dd enter_point_last = enter_center_line_points_body.back();  // P0 A1
    Point2Dd enter_point_second_last =
    enter_center_line_points_body[enter_center_line_points_body.size() - 3];  //  P1 A3
    Point2Dd exit_point_first = exit_center_line_points_body.front();   //   P3 B1
    Point2Dd exit_point_second = exit_center_line_points_body[2];   //    P2 B3
    // 系数和贝塞尔曲线点数
    double k = 0.5;
    // int n = 50;   //  贝塞尔曲线点的数量
    double two_point_distance = 1.25;

    Point2Dd  P1 = CalculateControlPoint(enter_point_last, enter_point_second_last, exit_point_first, k);
    Point2Dd  P2 = CalculateControlPoint(exit_point_first, exit_point_second, enter_point_last, k);

    // TODO(lxf)   最终下发的时候需要插值成一定的距离
    intersection_center_line_body = GenerateBezierCurve(enter_point_last, P1, P2, exit_point_first, two_point_distance);
  } else {
    std::cout << "section type is invalid" << std::endl;
    return false;
  }

  intersection_center_line_points_wgs84.length = CalculateLineLength(intersection_center_line_body);
  LineBodyToWGS84(intersection_center_line_body, ego_pose_point, intersection_center_line_points_wgs84.points);

  // std::cout << std::fixed << std::setprecision(14);
  // std::cout << "intersection_center_line_points_wgs84: "
  //           << intersection_center_line_points_wgs84.points.size() << ":{ ";
  // for (const auto& point : intersection_center_line_points_wgs84.points) {
  //   std::cout << "["<< point.x << ", " << point.y << "], ";
  // }
  // std::cout << "}" << std::endl;

  return true;
}

bool MakeMapFusion::GenerateCenterLines(const uint8_t& enter_ego_lane_idx, const uint8_t& exit_ego_lane_idx,
                                      const DoublePosePoint& ego_pose_point, const uint8_t& section_type,
                                      const bool& is_bigcurv_intersection, const uint16_t& intersection_id,
                                      std::vector<CrossLaneInfo>& enter_centerlines_body,
                                      std::vector<CrossLaneInfo>& exit_centerlines_body,
                                      std::vector<CrossLaneInfo>& intersection_center_lines_wgs84,
                                      bool& is_right_has_lane) {
  // std::cout << __FILE__ << __LINE__ << "enter_ego_lane_idx: " << int(enter_ego_lane_idx)
  //           << " exit_ego_lane_idx: " << int(exit_ego_lane_idx) << std::endl;
  // std::cout << __FILE__ << __LINE__ << "enter_centerlines_body: " << int(enter_centerlines_body.size())
  //           << " exit_centerlines_body: " << int(exit_centerlines_body.size()) << std::endl;
  if (enter_ego_lane_idx < 0 || enter_ego_lane_idx >= enter_centerlines_body.size() || exit_ego_lane_idx < 0
    || exit_ego_lane_idx >= exit_centerlines_body.size() || enter_centerlines_body.size() < 1
    || exit_centerlines_body.size() < 1) {
    std::cout << "enter or exit center line points wgs84 is empty" << std::endl;
    return false;
  }
  intersection_center_lines_wgs84.clear();
  std::cout << __FILE__ << __LINE__ << "enter_ego_lane_idx: " << int(enter_ego_lane_idx)
            << " exit_ego_lane_idx: " << int(exit_ego_lane_idx) << std::endl;

  std::vector<Point2Dd> tar_enter_center_line_body{};
  std::vector<std::vector<Point2Dd>> tar_exit_centerlines_body{};

  if (true == is_bigcurv_intersection) {
  // if (0) {
    // 拐弯的地方只生成一条中心线
    CrossLaneInfo tar_enter_center_line_body = enter_centerlines_body[enter_ego_lane_idx];
    CrossLaneInfo tar_exit_center_line_body = exit_centerlines_body[exit_ego_lane_idx];
    LineAttribute intersection_center_line_wgs84{};
    GenerateCenterLine(tar_enter_center_line_body.geometry, tar_exit_center_line_body.geometry, ego_pose_point,
                      section_type, intersection_center_line_wgs84);
    CrossLaneInfo intersection_center_line{};
    intersection_center_line.id = (intersection_id - 1) * 50 + 20;
    intersection_center_line.lane_idx = 1;
    intersection_center_line.pre_id_1 = tar_enter_center_line_body.id;
    intersection_center_line.next_id_1 = tar_exit_center_line_body.id;
    intersection_center_line.cross_id = intersection_id * 50;
    intersection_center_line.type = 2;
    intersection_center_line.length = intersection_center_line_wgs84.length;
    intersection_center_line.geometry = intersection_center_line_wgs84.points;
    enter_centerlines_body[enter_ego_lane_idx].next_id_1 = intersection_center_line.id;
    exit_centerlines_body[exit_ego_lane_idx].pre_id_1 = intersection_center_line.id;
    intersection_center_lines_wgs84.emplace_back(intersection_center_line);
  } else {
    // 直行路口中心线生成
    // 自车走过车道的右侧车道
    is_right_has_lane = false;
    if (enter_ego_lane_idx - 1 >= 0 && exit_ego_lane_idx - 1 >= 0) {
      is_right_has_lane = true;
      CrossLaneInfo tar_enter_center_line_body = enter_centerlines_body[enter_ego_lane_idx - 1];
      CrossLaneInfo tar_exit_center_line_body = exit_centerlines_body[exit_ego_lane_idx - 1];
      LineAttribute intersection_center_line_wgs84{};
      GenerateCenterLine(tar_enter_center_line_body.geometry, tar_exit_center_line_body.geometry, ego_pose_point,
                        section_type, intersection_center_line_wgs84);
      CrossLaneInfo intersection_center_line_right{};
      intersection_center_line_right.id = (intersection_id - 1) * 50 + 20 - 1;
      intersection_center_line_right.lane_idx = 1;
      intersection_center_line_right.pre_id_1 = tar_enter_center_line_body.id;
      intersection_center_line_right.next_id_1 = tar_exit_center_line_body.id;
      intersection_center_line_right.cross_id = intersection_id * 50;
      intersection_center_line_right.type = 2;
      intersection_center_line_right.length = intersection_center_line_wgs84.length;
      intersection_center_line_right.geometry = intersection_center_line_wgs84.points;
      enter_centerlines_body[enter_ego_lane_idx - 1].next_id_1 = intersection_center_line_right.id;
      exit_centerlines_body[exit_ego_lane_idx - 1].pre_id_1 = intersection_center_line_right.id;
      intersection_center_lines_wgs84.emplace_back(intersection_center_line_right);
    }

    CrossLaneInfo intersection_center_line{};
    if (true == is_right_has_lane) {
      intersection_center_line.lane_idx = 2;
    } else {
      intersection_center_line.lane_idx = 1;
    }
    // 自车走过的车道
    CrossLaneInfo tar_enter_center_line_body = enter_centerlines_body[enter_ego_lane_idx];
    CrossLaneInfo tar_exit_center_line_body = exit_centerlines_body[exit_ego_lane_idx];
    LineAttribute intersection_center_line_wgs84{};
    GenerateCenterLine(tar_enter_center_line_body.geometry, tar_exit_center_line_body.geometry, ego_pose_point,
                      section_type, intersection_center_line_wgs84);
    intersection_center_line.id = (intersection_id - 1) * 50 + 20;
    intersection_center_line.pre_id_1 = tar_enter_center_line_body.id;
    intersection_center_line.next_id_1 = tar_exit_center_line_body.id;
    intersection_center_line.cross_id = intersection_id * 50;
    intersection_center_line.type = 2;
    intersection_center_line.length = intersection_center_line_wgs84.length;
    intersection_center_line.geometry = intersection_center_line_wgs84.points;
    enter_centerlines_body[enter_ego_lane_idx].next_id_1 = intersection_center_line.id;
    exit_centerlines_body[exit_ego_lane_idx].pre_id_1 = intersection_center_line.id;
    intersection_center_lines_wgs84.emplace_back(intersection_center_line);

    // 自车走过车道的左侧车道
    if (enter_ego_lane_idx + 1 < enter_centerlines_body.size() &&
        exit_ego_lane_idx + 1 < exit_centerlines_body.size()) {
      CrossLaneInfo tar_enter_center_line_body = enter_centerlines_body[enter_ego_lane_idx + 1];
      CrossLaneInfo tar_exit_center_line_body = exit_centerlines_body[exit_ego_lane_idx + 1];
      LineAttribute intersection_center_line_wgs84{};
      GenerateCenterLine(tar_enter_center_line_body.geometry, tar_exit_center_line_body.geometry, ego_pose_point,
                        section_type, intersection_center_line_wgs84);
      CrossLaneInfo intersection_center_line_left{};
      if (true == is_right_has_lane) {
        intersection_center_line_left.lane_idx = 3;
      } else {
        intersection_center_line_left.lane_idx = 2;
      }
      intersection_center_line_left.id = (intersection_id - 1) * 50 + 20 + 1;
      intersection_center_line_left.pre_id_1 = tar_enter_center_line_body.id;
      intersection_center_line_left.next_id_1 = tar_exit_center_line_body.id;
      intersection_center_line_left.cross_id = intersection_id * 50;
      intersection_center_line_left.type = 2;
      intersection_center_line_left.length = intersection_center_line_wgs84.length;
      intersection_center_line_left.geometry = intersection_center_line_wgs84.points;
      enter_centerlines_body[enter_ego_lane_idx + 1].next_id_1 = intersection_center_line_left.id;
      exit_centerlines_body[exit_ego_lane_idx + 1].pre_id_1 = intersection_center_line_left.id;
      intersection_center_lines_wgs84.emplace_back(intersection_center_line_left);
    }
  }

  return true;
}

bool MakeMapFusion::StoreTarCenterLines(const uint8_t& lane_idx,
                          const std::vector<std::vector<Point2Dd>>& center_lines_body,
                          std::vector<std::vector<Point2Dd>>& tar_center_lines_body) {
    // 清空目标容器
    tar_center_lines_body.clear();

    // 确保索引在有效范围内
    if (lane_idx >= center_lines_body.size()) {
        std::cout << "exit_ego_lane_idx is out of bounds" << std::endl;
        return false;
    }

    // 存放自身及左右两根线
    if (lane_idx > 0) {
        tar_center_lines_body.emplace_back(center_lines_body[lane_idx - 1]);
    }
    tar_center_lines_body.emplace_back(center_lines_body[lane_idx]);
    if (lane_idx < center_lines_body.size() - 1) {
        tar_center_lines_body.emplace_back(center_lines_body[lane_idx + 1]);
    }

    return true;
}

std::vector<Point2Dd> MakeMapFusion::GenerateInterpolatedLine(
    const Point2Dd& end_point_first, const Point2Dd& start_point_second, double interval) {
    std::vector<Point2Dd> interpolated_line{};
    interpolated_line.clear();

    // 计算两点之间的距离
    double dx = start_point_second.x - end_point_first.x;
    double dy = start_point_second.y - end_point_first.y;
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    if (distance > 1000) {
      // std::cout << __FILE__ << __LINE__ << "distance: " << distance << std::endl;
      return interpolated_line;
    }

    // 根据距离和插值间隔计算插值点的数量
    int num_points = static_cast<int>(distance / interval);

    // 生成插值点
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = end_point_first.x * (1 - t) + start_point_second.x * t;
        double y = end_point_first.y * (1 - t) + start_point_second.y * t;
        interpolated_line.push_back(Point2Dd{x, y});
    }

    return interpolated_line;
}

// 计算控制点
Point2Dd MakeMapFusion::CalculateControlPoint(const Point2Dd& A1, const Point2Dd& A3, const Point2Dd& B1, double k) {
    double A1B1Dist = std::sqrt(std::pow(A1.x - B1.x, 2) + std::pow(A1.y - B1.y, 2));
    double A1A3Dist = std::sqrt(std::pow(A1.x - A3.x, 2) + std::pow(A1.y - A3.y, 2));
    Point2Dd P1;
    P1.x = A1.x + k * (A1.x - A3.x) * (A1B1Dist / A1A3Dist);
    P1.y = A1.y + k * (A1.y - A3.y) * (A1B1Dist / A1A3Dist);
    return P1;
}

// 计算贝塞尔曲线的点
std::vector<Point2Dd> MakeMapFusion::GenerateBezierCurve(const Point2Dd& P0, const Point2Dd& P1,
                                                         const Point2Dd& P2, const Point2Dd& P3,
                                                         const double& segment_length) {
    std::vector<Point2Dd> bezierPoints;
    bezierPoints.push_back(P0);

    double t = 0.0;
    Point2Dd prev_point = P0;

    while (t < 1.0) {
        double t1 = 1 - t;
        double t1_squared = t1 * t1;
        double t_squared = t * t;

        Point2Dd bezierPoint;
        bezierPoint.x = P0.x * t1 * t1_squared +
                        3 * P1.x * t1_squared * t +
                        3 * P2.x * t1 * t_squared +
                        P3.x * t_squared * t;
        bezierPoint.y = P0.y * t1 * t1_squared +
                        3 * P1.y * t1_squared * t +
                        3 * P2.y * t1 * t_squared +
                        P3.y * t_squared * t;

        double distance = std::sqrt(std::pow(bezierPoint.x - prev_point.x, 2) +
                          std::pow(bezierPoint.y - prev_point.y, 2));
        if (distance >= segment_length) {
            bezierPoints.push_back(bezierPoint);
            prev_point = bezierPoint;
        }

        t += 0.001;  // 增加t的步长以确保生成足够多的点
    }

    bezierPoints.push_back(P3);  // 确保最后一个点是P3

    return bezierPoints;
}

std::vector<Point2Dd> MakeMapFusion::BoundryGenerateCenterLine(
    const std::vector<Point2Dd>& lane1, const std::vector<Point2Dd>& lane2) {
    std::vector<Point2Dd> center_line{};
    const std::vector<Point2Dd>& shorter_lane = (lane1.size() <= lane2.size()) ? lane1 : lane2;
    const std::vector<Point2Dd>& longer_lane = (lane1.size() > lane2.size()) ? lane1 : lane2;
    // std::cout << __FILE__ << __LINE__ << "shorter_lane.size(): " << shorter_lane.size() << std::endl;
    // std::cout << __FILE__ << __LINE__ << "longer_lane.size(): " << longer_lane.size() << std::endl;

    // 计算每对对应点的中点
    for (size_t i = 0; i < shorter_lane.size(); ++i) {
      Point2Dd closest_point = FindClosestPoint(shorter_lane[i], longer_lane);
      double x = (shorter_lane[i].x + closest_point.x) / 2.0;
      double y = (shorter_lane[i].y + closest_point.y) / 2.0;
      center_line.emplace_back(Point2Dd{x, y});
    }
    // std::cout << __FILE__ << __LINE__ << "center_line.size(): " << center_line.size() << std::endl;

    return center_line;
}

bool MakeMapFusion::BoundrysGenerateCenterLines(
    const std::vector<std::vector<LPoint>>& enter_boundrys_wgs84,
    const uint16_t& intersection_id, const bool& is_enter,
    const DoublePosePoint& ego_pose_point,
    std::vector<CrossLaneInfo>& enter_centerline_body) {
    if (enter_boundrys_wgs84.empty()) {
        std::cout << "Error: The enter_boundrys_wgs84 is empty." << std::endl;
        return false;
    }
    // 0217lxf: 由于边界线的点数不一定相等，所以不再检查边界线的点数是否相等
    uint64_t cross_id = intersection_id * 50;
    uint8_t type = is_enter ? 1 : 3;
    uint64_t base_line_id = 0;
    if (true == is_enter) {
      base_line_id = (intersection_id - 1) * 50 + 10;
    } else {
      base_line_id = intersection_id * 50;
    }

    std::vector<std::vector<Point2Dd>> enter_boundrys_body{};
    enter_centerline_body.clear();

    std::vector<std::vector<Point2Dd>> enter_boundrys_wgs84_tmp{};
    enter_boundrys_wgs84_tmp.clear();
    for (const auto& boundary : enter_boundrys_wgs84) {
      std::vector<Point2Dd> boundary_tmp{};
      for (const auto& point : boundary) {
        boundary_tmp.emplace_back(Point2Dd{point.X, point.Y});
      }
      enter_boundrys_wgs84_tmp.emplace_back(boundary_tmp);
    }
    // 将边界线从WGS84坐标系转换成自车坐标系
    for (const auto& boundary_wgs84 : enter_boundrys_wgs84_tmp) {
        std::vector<Point2Dd> boundary_body{};
        boundary_body.clear();
        LineWGS84ToBody(boundary_wgs84, ego_pose_point, boundary_body);
        enter_boundrys_body.emplace_back(boundary_body);
    }
    // std::cout << __FILE__ << __LINE__ << "enter_boundrys_body.size(): " << enter_boundrys_body.size() << std::endl;

    // 生成中心线
    for (size_t i = 0; i < enter_boundrys_body.size() - 1; ++i) {
      CrossLaneInfo cross_lane_info{};
      cross_lane_info.cross_id = cross_id;
      cross_lane_info.lane_idx = i + 1;
      cross_lane_info.geometry = BoundryGenerateCenterLine(enter_boundrys_body[i], enter_boundrys_body[i + 1]);
      cross_lane_info.id = base_line_id + i;
      cross_lane_info.length = CalculateLineLength(cross_lane_info.geometry) * 100;
      cross_lane_info.type = type;
      enter_centerline_body.push_back(cross_lane_info);
    }

    return true;
}

uint8_t MakeMapFusion::GetMostFrequentRoadType(const LaneMappingMemory& memory_lane_mapping_info) {
    const auto& traj_history = memory_lane_mapping_info.TrajHistory;

    if (traj_history.size() < 2 || traj_history.size() > 5000) {
      std::cout << "Error: The number of points in the traj_history is invalid." << std::endl;
      return 0;
    }

    // 统计每种RoadType的出现次数
    std::map<uint8_t, size_t> road_type_count{};
    for (const auto& position : traj_history) {
        road_type_count[position.RoadType]++;
    }

    // 找到出现次数最多的RoadType
    uint8_t most_frequent_road_type = 0;
    size_t max_count = 0;
    for (const auto& entry : road_type_count) {
        if (entry.second > max_count) {
            most_frequent_road_type = entry.first;
            max_count = entry.second;
        }
    }

    return most_frequent_road_type;
}

uint8_t MakeMapFusion::GetMostFrequentLaneType(const LaneMappingMemory& memory_lane_mapping_info,
                                               const uint8_t& lane_road_type) {
  const auto& traj_history = memory_lane_mapping_info.TrajHistory;

  if (traj_history.size() < 2 || traj_history.size() > 5000) {
    std::cout << "Error: The number of points in the traj_history is invalid." << std::endl;
    return 0;
  }

  uint8_t lane_id = (lane_road_type == 1)
                    ? memory_lane_mapping_info.exit_ego_lane_id
                    : memory_lane_mapping_info.enter_ego_lane_id;

  std::map<uint8_t, size_t> lane_type_count{};
  for (const auto& position : traj_history) {
    if (lane_id < position.LaneType.size()) {
      lane_type_count[position.LaneType[lane_id]]++;
    }
  }

  uint8_t most_frequent_lane_type = 0;
  size_t max_count = 0;
  for (const auto& entry : lane_type_count) {
    if (entry.second > max_count) {
      most_frequent_lane_type = entry.first;
      max_count = entry.second;
    }
  }

  return most_frequent_lane_type;

}

double MakeMapFusion::CalculateLineLength(const std::vector<Point2Dd>& line) {
    double total_length = 0.0;

    for (size_t i = 1; i < line.size(); ++i) {
        double dx = line[i].x - line[i - 1].x;
        double dy = line[i].y - line[i - 1].y;
        total_length += std::sqrt(dx * dx + dy * dy);
    }

    return total_length;
}

uint64_t MakeMapFusion::FindNearestLineId(const Point2Dd& point, const std::vector<CrossLaneInfo>& lines) {
    uint64_t nearest_line_id = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& line : lines) {
        for (const auto& geometry_point : line.geometry) {
            double distance = DistanceSquare(point.x, point.y, geometry_point.x, geometry_point.y);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_line_id = line.id;
            }
        }
    }

    return nearest_line_id;
}

bool MakeMapFusion::FindLaneChangePoints(const std::vector<Line>& line_out,
                        std::vector<LaneChangePointInfo>* lane_change_points) {
  if (true == line_out.empty()) {
    std::cout << "Error: The line_out is empty." << std::endl;
    return false;
  }
  lane_change_points->clear();

  for (const auto& line_info : line_out) {
    std::vector<Point2d> line_points{};
    // 遍历每一根线，获取每一根线的点
    line_points = line_info.points;

    auto segment_start_point_it = line_info.traj_line_ids.begin();
    while (segment_start_point_it != line_info.traj_line_ids.end()) {
      auto first_point_it = segment_start_point_it;
      uint8_t point_gap = 3;
      std::distance(line_info.traj_line_ids.begin(), segment_start_point_it)  + point_gap
      <  line_info.traj_line_ids.size() ? point_gap = 3 : point_gap = 0;
      if (point_gap != 3) {
        break;
      }
      auto second_point_it = std::next(segment_start_point_it, point_gap);
      auto tmp_point_it = std::next(segment_start_point_it, point_gap - 1);
      if (second_point_it != line_info.traj_line_ids.end() && point_gap == 3) {
        // 取出一条线的第一段和第gap段
        uint32_t first_traj_point_id = first_point_it->first;
        uint32_t second_traj_point_id = second_point_it->first;
        uint32_t tmp_traj_point_id = tmp_point_it->first;
        uint32_t first_line_point_idx = first_point_it->second;
        uint32_t second_line_point_idx = second_point_it->second;
        std::vector<Point2Dd> segment_line_points_wgs84{}, segment_line_points_body{};
        DoublePosePoint first_traj_point{};
        Point2Dd first_traj_point_body{0.0, 0.0}, second_traj_point_wgs84{}, second_traj_point_body{},
                segment_line_first_point{}, segment_line_last_point{};
        // 拿到一段线的所有点的坐标
        for (size_t i = first_line_point_idx; i <= second_line_point_idx; ++i) {
          segment_line_points_wgs84.emplace_back(Point2Dd{line_points[i].x, line_points[i].y});
        }
        if (traj_points_position_map_.find(first_traj_point_id) != traj_points_position_map_.end()) {
          first_traj_point.x = traj_points_position_map_[first_traj_point_id].x;
          first_traj_point.y = traj_points_position_map_[first_traj_point_id].y;
          first_traj_point.yaw = traj_points_position_map_[first_traj_point_id].yaw;
          // std::cout << __FILE__ << __LINE__ << "first_traj_point_id: " << first_traj_point_id << std::endl;
          // std::cout << __FILE__ << __LINE__ << "first_traj_point.x: " << first_traj_point.x <<
          //  "first_traj_point.y: " << first_traj_point.y << std::endl;
        }
        if (traj_points_position_map_.find(second_traj_point_id) != traj_points_position_map_.end()) {
          second_traj_point_wgs84.x = traj_points_position_map_[second_traj_point_id].x;
          second_traj_point_wgs84.y = traj_points_position_map_[second_traj_point_id].y;
          // std::cout << __FILE__ << __LINE__ << "second_traj_point_id: " << second_traj_point_id << std::endl;
          // std::cout << __FILE__ << __LINE__ << "second_traj_point.x: " << second_traj_point_wgs84.x <<
          //  "second_traj_point.y: " << second_traj_point_wgs84.y << std::endl;
        }

        LineWGS84ToBody(segment_line_points_wgs84, first_traj_point, segment_line_points_body);
        WGS84ToBody(second_traj_point_wgs84, first_traj_point, second_traj_point_body);

        if (false == segment_line_points_body.empty()) {
          segment_line_first_point = segment_line_points_body.front();
          segment_line_last_point = segment_line_points_body.back();
        }

        // std::cout << __FILE__ << __LINE__ << "segment_line_first_point.x: " << segment_line_first_point.x <<
        //  "segment_line_first_point.y: " << segment_line_first_point.y << std::endl;
        // std::cout << __FILE__ << __LINE__ << "segment_line_last_point.x: " << segment_line_last_point.x <<
        //  "segment_line_last_point.y: " << segment_line_last_point.y << std::endl;

        // 如果在第一段和第三段之间找到了一个变道点，那么下一次从第三段开始查找
        if (IsLineCross(first_traj_point_body, second_traj_point_body, segment_line_first_point,
                        segment_line_last_point)) {
          LaneChangePointInfo lane_change_point{};
          std::cout << __FILE__ << __LINE__ << "first_traj_point_id: " << first_traj_point_id << std::endl;
          std::cout << __FILE__ << __LINE__ << "second_traj_point_id: " << second_traj_point_id << std::endl;
          std::cout << __FILE__ << __LINE__ << "first_traj_point_body.x: " << first_traj_point_body.x <<
           "first_traj_point_body.y: " << first_traj_point_body.y << std::endl;
          std::cout << __FILE__ << __LINE__ << "second_traj_point_body.x: " << second_traj_point_body.x <<
           "second_traj_point_body.y: " << second_traj_point_body.y << std::endl;

          Point2Dd segment_line_vector{segment_line_last_point.x - segment_line_first_point.x,
                                       segment_line_last_point.y - segment_line_first_point.y};
          Point2Dd first_point_line_start_vector{first_traj_point_body.x - segment_line_first_point.x,
                                                first_traj_point_body.y - segment_line_first_point.y};
          Point2Dd second_point_line_start_vector{second_traj_point_body.x - segment_line_first_point.x,
                                                 second_traj_point_body.y - segment_line_first_point.y};
          double cross_product_first = CrossProduct(segment_line_vector.x, segment_line_vector.y,
                                      first_point_line_start_vector.x, first_point_line_start_vector.y);
          double cross_product_second = CrossProduct(segment_line_vector.x, segment_line_vector.y,
                                      second_point_line_start_vector.x, second_point_line_start_vector.y);
          std::cout << __FILE__ << __LINE__ << "cross_product_first: " << cross_product_first << std::endl;
          std::cout << __FILE__ << __LINE__ << "cross_product_second: " << cross_product_second << std::endl;

          if (cross_product_first * cross_product_second < 0) {
            if (cross_product_first > 0 && cross_product_second < 0) {
              lane_change_point.change_dir = 2;
            lane_change_point.is_change = true;
            lane_change_point.point_raw_id = (second_traj_point_id + tmp_traj_point_id) / 2;
            lane_change_points->emplace_back(lane_change_point);
            } else if (cross_product_first < 0 && cross_product_second > 0) {
              lane_change_point.change_dir = 1;
              lane_change_point.is_change = true;
              lane_change_point.point_raw_id = (second_traj_point_id + tmp_traj_point_id) / 2;
              lane_change_points->emplace_back(lane_change_point);
            } else {
              lane_change_point.change_dir = 0;
            }
          }
          std::advance(segment_start_point_it, point_gap);
        } else {
          ++segment_start_point_it;
        }
      } else {
        // 结束对这条线的查找
        break;
      }
    }
  }


  return true;
}

bool MakeMapFusion::ProcessCrossLanePoints(std::vector<PositionPointInfo>& position_semantic_infos) {
  size_t position_semantic_infos_size = position_semantic_infos.size();
  if (position_semantic_infos_size < 2) {
    std::cout << "Error: The position_semantic_infos is empty." << std::endl;
    return false;
  }
  for (size_t s_idx = 1; s_idx < position_semantic_infos_size - 1; s_idx++) {
    if (position_semantic_infos[s_idx].changepoint_offset <
          position_semantic_infos[s_idx + 1].changepoint_offset ) {
      // 从路口点往回找
      for (int pre_idx = s_idx - 2; pre_idx >= 0; --pre_idx) {
        if (position_semantic_infos[pre_idx].changepoint_offset <
                position_semantic_infos[pre_idx + 1].changepoint_offset ||
                pre_idx < 2) {
          break;
        }
        if (true == position_semantic_infos[pre_idx].lane_change_point_info.is_change) {
          // 找到一个变道点
          std::cout << __FILE__ << __LINE__ << "find lane change point" << std::endl;
          std::cout << __FILE__ << __LINE__ << "position_semantic_infos.id: " <<
          position_semantic_infos[pre_idx].id << std::endl;
          uint8_t change_dir = position_semantic_infos[pre_idx].lane_change_point_info.change_dir;
          uint8_t cross_point_dir = 0;
          uint8_t cross_point_times = 0;

          cross_point_dir = change_dir;
          cross_point_times = 1;

          // 更新变道点之前的点
          for (int pre_idx2 = pre_idx -1 ; pre_idx2 >= 0; --pre_idx2) {
            std::cout << __FILE__ << __LINE__ << "cross_point_times: " << int(cross_point_times) << std::endl;
            position_semantic_infos[pre_idx2].to_cross_point_dir = cross_point_dir;
            position_semantic_infos[pre_idx2].to_cross_point_times = cross_point_times;

            if (true == position_semantic_infos[pre_idx2].lane_change_point_info.is_change) {
              uint8_t pre_change_dir = position_semantic_infos[pre_idx2].lane_change_point_info.change_dir;
              std::cout << __FILE__ << __LINE__ << "pre_change_dir: " << int(pre_change_dir) << std::endl;
              std::cout << __FILE__ << __LINE__ << "change_dir: " << int(change_dir) << std::endl;
              std::cout << __FILE__ << __LINE__ << "cross_point_times: " << int(cross_point_times) << std::endl;
              if (pre_change_dir == change_dir) {
                cross_point_times++;
              } else if (pre_change_dir != change_dir && cross_point_times > 1) {
                cross_point_times--;
              } else {
                cross_point_dir = 0;
                cross_point_times = cross_point_times > 0 ? cross_point_times - 1 : 0;
              }
              change_dir = pre_change_dir;
            }

            if ((position_semantic_infos[pre_idx2].changepoint_offset <
                position_semantic_infos[pre_idx2 + 1].changepoint_offset) || pre_idx2 < 2) {
              // 遇到下一个路口点，跳出循环
              pre_idx = pre_idx2;
              break;
            }
          }
        }
        if (position_semantic_infos[pre_idx].changepoint_offset <
                position_semantic_infos[pre_idx + 1].changepoint_offset ||
                pre_idx < 2) {
          break;
        }
      }
    }
  }

  return true;
}

bool MakeMapFusion::ProcessSplitLanePoints(std::vector<PositionPointInfo>& position_semantic_infos) {
  int position_semantic_infos_size = position_semantic_infos.size();
  double offset = 0;
  for (int e_idx = position_semantic_infos_size - 1; e_idx >= 0; --e_idx) {
    if (position_semantic_infos[e_idx].changepoint_type == 3) {
      position_semantic_infos[e_idx].changepoint_offset = offset;
      int split_back_point_num = 0;
      for (int tmp_idx = e_idx -1; tmp_idx > 0; --tmp_idx) {
        if (position_semantic_infos[tmp_idx].changepoint_type != 3 &&
            split_back_point_num > 3) {
          offset = 0;
          e_idx = tmp_idx;
          break;
        } else if (position_semantic_infos[tmp_idx].changepoint_type != 3) {
          split_back_point_num++;
          position_semantic_infos[tmp_idx].changepoint_type = 3;
        }

        Point2Dd cur_point{}, cur_point_ego{};
        DoublePosePoint next_point{};

        cur_point.x = position_semantic_infos[tmp_idx].position.x;
        cur_point.y = position_semantic_infos[tmp_idx].position.y;
        next_point.x = position_semantic_infos[tmp_idx + 1].position.x;
        next_point.y = position_semantic_infos[tmp_idx + 1].position.y;
        next_point.yaw = position_semantic_infos[tmp_idx + 1].heading;
        WGS84ToBody(cur_point, next_point, cur_point_ego);
        double distance = Distance(0.0, 0.0, cur_point_ego.x, cur_point_ego.y);
        offset += distance;
        std::cout << __FILE__ << __LINE__ << "distance: " << distance << std::endl;
        std::cout << __FILE__ << __LINE__ << "split_offset: " << offset << std::endl;
        std::cout << __FILE__ << __LINE__ << "static_cast<uint32_t>(offset): "
        << static_cast<uint32_t>(offset) << std::endl;

        if (offset > 100.0) {
          offset = 0;
          e_idx = tmp_idx;
          break;
        }

        position_semantic_infos[tmp_idx].changepoint_offset = static_cast<uint32_t>(offset);
      }
    }
  }
  return true;
}

}  // namespace MapFusion
}  // namespace CommuteMode
