#include "MapFusion.h"
#ifdef __QNX__
#include "gptp/gptp.h"
#endif

namespace CommuteMode {
namespace MapFusion {
MapFusionStaticVar MapFusion::map_fusion_static_var_{};
MapFusion::MapFusion() {
  MapFusionConfig("/mapfusion_config.yaml");
  make_input_data_ = std::make_shared<MakeInputData>();
  make_map_fusuion_ = std::make_shared<MakeMapFusion>();
  make_output_data_ = std::make_shared<MakeOutputData>();
}

bool MapFusion::Execute(const std::vector<LaneMappingMsg>& lane_mapping_infos, const std::vector<Line>& line_out,
                        const bool& is_map_saving, PPOINT2EFM* ppoint2efm, PPOLYGON2EFM* ppolygon2efm,
                        CLANE2EFM* clane2efm, MapFusionStatus* map_fusion_status) {
  // std::cout << "mapfusion_cntr: " << map_fusion_static_var_.mapfusion_cntr <<std::endl;
  map_fusion_static_var_.mapfusion_cntr++;
  ZTEXT("LaneMapping", "mapfusion_cntr: ", 10, 78, "mapfusion_cntr: {}", map_fusion_static_var_.mapfusion_cntr);
  ZTEXT("LaneMapping", "all_data_sent_cntr: ", 10, 82, "all_data_sent_cntr: {}",
        map_fusion_static_var_.all_data_sent_cntr);
  ZTEXT("LaneMapping", "is_all_data_sent: ", 10, 86, "is_all_data_sent: {}",
       static_cast<int>(map_fusion_static_var_.all_data_sent));
  if (false == InitMapFusion(lane_mapping_infos, is_map_saving)) {
    map_fusion_status->map_saving_status = 0;
    map_fusion_status->map_id = 0;
    map_fusion_status->map_distance = 0.0f;
    ppoint2efm->PathId = 0;
    ppolygon2efm->PathId = 0;
    clane2efm->PathId = 0;
    ZTEXT("LaneMapping", "mapfusion_init_ing: ", 10, 74, "mapfusion_init_ing: {}", 1);
    ZTEXT("LaneMapping", "map_fusion_inti_times: ", 10, 18, "map_fusion_inti_times: {}",
       static_cast<int>(map_fusion_static_var_.map_fusion_inti_times));
    return false;
  }
  ZTEXT("LaneMapping", "mapfusion_init_ing: ", 10, 74, "mapfusion_init_ing: {}", 0);
  ZTEXT("LaneMapping", "map_fusion_inti_times: ", 10, 18, "map_fusion_inti_times: {}",
       static_cast<int>(map_fusion_static_var_.map_fusion_inti_times));

  std::cout << "MapFusion::Execute()" << std::endl;

#ifdef __QNX__
        const std::string map_data_directory ="/userdata/tmp/";
#else
        const std::string map_data_directory = "./";
#endif

  // static int get_map_id_times = 0;
  std::cout << "map_fusion_static_var_.get_map_id_times: " << int(map_fusion_static_var_.get_map_id_times) << std::endl;
  if (map_fusion_static_var_.get_map_id_times == 0) {
    map_fusion_static_var_.map_id = GetMapId(map_data_directory);
    map_fusion_static_var_.get_map_id_times++;
  }
  ZTEXT("LaneMapping", "rawmapid: ", 10, 70, "rawmapid: {}", map_fusion_static_var_.map_id);
  std::cout << __FILE__ << __LINE__ << "map_id: " << int(map_fusion_static_var_.map_id) << std::endl;

  if (false == CheckMapId(map_fusion_status)) {
    return false;
  }

  if (map_fusion_static_var_.map_fusion_valid_cnt < 10) {
    map_fusion_static_var_.map_fusion_valid_cnt++;
    map_fusion_status->map_saving_status = 5;
    map_fusion_status->map_id = 0;
    map_fusion_status->map_distance = 0.0f;

    map_fusion_static_var_.is_map_process_end = false;
    // std::cout << __FILE__ << __LINE__ << "map saving flag" << std::endl;
    return false;
  }

  std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.is_map_process_end: " <<
  static_cast<int>(map_fusion_static_var_.is_map_process_end)<< std::endl;
  if (false == map_fusion_static_var_.is_map_process_end) {
    MapProcess(lane_mapping_infos, line_out);
    map_fusion_static_var_.is_map_process_end = true;
  }

  map_fusion_static_var_.position_semantic_infos_res_ = make_output_data_->GetPositionPointInfos();
  map_fusion_static_var_.links_polygon_ = make_output_data_->GetLinksPolygon();

  size_t position_bytes_sent = 0;
  size_t polygon_bytes_sent = 0;
  uint8_t same_packet_frame_cntr = 3;
  static bool last_packet_first_send = false;
  static uint8_t last_packet_sent_cntr = 1;

  // std::cout << "last_packet_first_send: " << static_cast<int>(last_packet_first_send) << std::endl;
  // std::cout << "map_fusion_static_var_.mapfusion_cntr: " << map_fusion_static_var_.mapfusion_cntr<< std::endl;

  if (map_fusion_static_var_.mapfusion_cntr % same_packet_frame_cntr == 1 &&
      false == map_fusion_static_var_.all_data_sent) {
    // if (false == map_fusion_static_var_.all_data_sent) {
      std::cout << "map_fusion_static_var_.position_semantic_infos_res_.size()_byte: "
              << map_fusion_static_var_.position_semantic_infos_res_.size() * sizeof(PositionPointInfo)<< std::endl;
      std::cout << "map_fusion_static_var_.links_polygon_.size()_byte: "
                << map_fusion_static_var_.links_polygon_.size() * sizeof(LinkPolygon) << std::endl;
      std::cout << "map_fusion_static_var_.position_semantic_infos_res_.size(): " <<
                map_fusion_static_var_.position_semantic_infos_res_.size() << std::endl;
      std::cout << "map_fusion_static_var_.links_polygon_.size(): " <<
                map_fusion_static_var_.links_polygon_.size() << std::endl;


      uint32_t max_packet_size = p_max_packet_size;
      // uint32_t max_packet_size = 512;
      // uint32_t max_packet_size = 512;
      std::cout << __FILE__ << __LINE__ <<  "max_packet_size: " << max_packet_size << std::endl;
      std::cout << __FILE__ << __LINE__ <<  "PositionPointInfo size: " <<
                static_cast<int>(sizeof(PositionPointInfo)) << std::endl;
      //   分包处理 position_semantic_infos_res_
      size_t position_start_index = map_fusion_static_var_.current_position_index_;
      ppoint2efm->PathId = map_fusion_static_var_.map_id;
      ppoint2efm->Index = map_fusion_static_var_.position_packet_index_;
      // 计算分包数量
      ppoint2efm->Count = (map_fusion_static_var_.position_semantic_infos_res_.size() * sizeof(PositionPointInfo)
                          + max_packet_size - 1) / max_packet_size;

      while (map_fusion_static_var_.current_position_index_ < map_fusion_static_var_.position_semantic_infos_res_.size()
            && position_bytes_sent < max_packet_size) {
          position_bytes_sent += sizeof(PositionPointInfo);
          map_fusion_static_var_.current_position_index_++;
      }

      ppoint2efm->point_infos.clear();
      for (size_t i = position_start_index; i < map_fusion_static_var_.current_position_index_; ++i) {
          const auto& pos_info = map_fusion_static_var_.position_semantic_infos_res_[i];
          PPoint ppoint;
          ppoint.id = pos_info.id;
          ppoint.path_offset = pos_info.path_offset/ 100;
          ppoint.changepoint_offset = pos_info.changepoint_offset;
          ppoint.changepoint_type = pos_info.changepoint_type;
          ppoint.changepoint_dir = pos_info.changepoint_dir;
          ppoint.lane_id = pos_info.line_id;
          ppoint.speed = pos_info.speed;
          ppoint.heading = pos_info.heading;
          ppoint.X = pos_info.position.x;
          ppoint.Y = pos_info.position.y;
          ppoint2efm->point_infos.push_back(ppoint);
      }

      // 分包处理 links_polygon_
      size_t polygon_start_index = map_fusion_static_var_.current_polygon_index_;
      ppolygon2efm->PathId = map_fusion_static_var_.map_id;
      ppolygon2efm->Index = map_fusion_static_var_.polygon_packet_index_;
      ppolygon2efm->Count = (map_fusion_static_var_.links_polygon_.size() * sizeof(LinkPolygon) + max_packet_size - 1)
                            / max_packet_size;

      while (map_fusion_static_var_.current_polygon_index_ < map_fusion_static_var_.links_polygon_.size()
            && polygon_bytes_sent < max_packet_size) {
          polygon_bytes_sent += sizeof(LinkPolygon);
          map_fusion_static_var_.current_polygon_index_++;
      }

      ppolygon2efm->polygon_infos.clear();
      for (size_t i = polygon_start_index; i < map_fusion_static_var_.current_polygon_index_; ++i) {
          const auto& link_poly = map_fusion_static_var_.links_polygon_[i];
          PPolygon ppolygon;
          ppolygon.id = link_poly.link_id;
          ppolygon.start_point_id = link_poly.start_point_id;
          ppolygon.end_point_id = link_poly.end_point_id;
          for (const auto& coord : link_poly.line_coordinates) {
              // LinePoint line_point;
              // line_point.x = coord.x;
              // line_point.y = coord.y;
              ppolygon.geom_info.push_back({coord.x, coord.y});
          }
          ppolygon2efm->polygon_infos.push_back(ppolygon);
      }

      // 保存历史包
      map_fusion_static_var_.last_ppoint2efm = *ppoint2efm;
      map_fusion_static_var_.last_ppolygon2efm = *ppolygon2efm;

      // 检查是否所有数据都已发送完毕
      // map_fusion_static_var_.all_data_sent = (map_fusion_static_var_.current_position_index_ >=
      //                       map_fusion_static_var_.position_semantic_infos_res_.size()) &&
      //                       (map_fusion_static_var_.current_polygon_index_ >=
      //                       map_fusion_static_var_.links_polygon_.size());
      last_packet_first_send = (map_fusion_static_var_.current_position_index_ >=
                            map_fusion_static_var_.position_semantic_infos_res_.size()) &&
                            (map_fusion_static_var_.current_polygon_index_ >=
                            map_fusion_static_var_.links_polygon_.size());

    // }
    std::cout << "func 1: " << std::endl;
  // } else if (map_fusion_static_var_.mapfusion_cntr % 3 != 1 && false == map_fusion_static_var_.all_data_sent) {
  } else if (map_fusion_static_var_.mapfusion_cntr % 3 != 1 && false == last_packet_first_send) {
    std::cout << "func 2: " << std::endl;
    ppoint2efm = &map_fusion_static_var_.last_ppoint2efm;
    ppolygon2efm = &map_fusion_static_var_.last_ppolygon2efm;
  } else if (map_fusion_static_var_.mapfusion_cntr % 3 != 1 && true == last_packet_first_send
            && map_fusion_static_var_.all_data_sent == false) {
    std::cout << "func 3: " << std::endl;
    ppoint2efm = &map_fusion_static_var_.last_ppoint2efm;
    ppolygon2efm = &map_fusion_static_var_.last_ppolygon2efm;
    last_packet_sent_cntr++;
    if (last_packet_sent_cntr > same_packet_frame_cntr - 1) {
      map_fusion_static_var_.all_data_sent = true;
    }
  } else {
    std::cout << "func 4: " << std::endl;
    ppoint2efm->PathId = 0;
    ppoint2efm->Index = 0;
    ppoint2efm->Count = 0;
    ppolygon2efm->PathId = 0;
    ppolygon2efm->Index = 0;
    ppolygon2efm->Count = 0;
    // TODO(lxf) lane
  }

  // if (true == map_fusion_static_var_.all_data_sent) {
  //   map_fusion_static_var_.last_packet_cntr++;
  // }

  std::cout << "ppoint2efm->PathId: " << int(ppoint2efm->PathId) << std::endl;
  std::cout << "ppoint2efm->Index: " << int(ppoint2efm->Index) << std::endl;
  std::cout << "ppoint2efm->Count: " << int(ppoint2efm->Count) << std::endl;
  std::cout << "ppolygon2efm->PathId: " << int(ppolygon2efm->PathId) << std::endl;
  std::cout << "ppolygon2efm->Index: " << int(ppolygon2efm->Index) << std::endl;
  std::cout << "ppolygon2efm->Count: " << int(ppolygon2efm->Count) << std::endl;
  std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.current_position_index_; " <<
            map_fusion_static_var_.current_position_index_ << std::endl;
  std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.current_polygon_index_; " <<
            map_fusion_static_var_.current_polygon_index_ << std::endl;

  std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.all_data_sent: " <<
              static_cast<int>(map_fusion_static_var_.all_data_sent) << std::endl;

  if (map_fusion_static_var_.all_data_sent) {
    if (map_fusion_static_var_.all_data_sent_cntr <= 0) {
        map_fusion_static_var_.all_data_sent_cntr = map_fusion_static_var_.mapfusion_cntr;
    }
    map_fusion_status->map_saving_status = 6;
    map_fusion_status->map_id = map_fusion_static_var_.map_id  > 0 ? map_fusion_static_var_.map_id : 0;

    if (!map_fusion_static_var_.position_semantic_infos_res_.empty()) {
      map_fusion_status->map_distance = map_fusion_static_var_.position_semantic_infos_res_.back().path_offset / 100;
    } else {
      map_fusion_status->map_distance = 0.0f;
      return false;
    }
    map_fusion_static_var_.current_position_index_ = 0;
    map_fusion_static_var_.current_polygon_index_ = 0;
    position_bytes_sent = 0;
    polygon_bytes_sent = 0;
    map_fusion_static_var_.position_packet_index_ = 1;
    map_fusion_static_var_.polygon_packet_index_ = 1;
    map_fusion_static_var_.links_polygon_.clear();
    std::cout << __FILE__ << __LINE__ << "map_distance: " << map_fusion_status->map_distance << std::endl;
    std::cout << __FILE__ << __LINE__<< "*************** all data sent ***************" << std::endl;
    return true;
  } else {
      if (map_fusion_static_var_.current_position_index_ < map_fusion_static_var_.position_semantic_infos_res_.size()
          && map_fusion_static_var_.mapfusion_cntr % 3 == 1) {
        map_fusion_static_var_.position_packet_index_++;
      }
      if (map_fusion_static_var_.current_polygon_index_ < map_fusion_static_var_.links_polygon_.size()
          && map_fusion_static_var_.mapfusion_cntr % 3 == 1) {
        map_fusion_static_var_.polygon_packet_index_++;
      }
      map_fusion_status->map_saving_status = 5;
      map_fusion_status->map_id = 0;
      map_fusion_status->map_distance = 0.0f;
      std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.position_packet_index_: "
      << int(map_fusion_static_var_.position_packet_index_) << std::endl;
      std::cout << __FILE__ << __LINE__ << "map_fusion_static_var_.polygon_packet_index_: "
      << int(map_fusion_static_var_.polygon_packet_index_) << std::endl;
      std::cout << __FILE__ << __LINE__<< "*************** data sentting ***************" << std::endl;
      return false;
  }

  return true;
}

uint8_t MapFusion::GetMapId(const std::string& directory) {
    std::regex mapdata_regex(R"(memory\d+)");
    uint8_t count = 0;

    DIR* dir = opendir(directory.c_str());
    if (dir == nullptr) {
        std::cout << "Failed to open directory: " << directory << std::endl;
        return 0;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string folder_name = entry->d_name;
        if (folder_name == "." || folder_name == "..") {
            continue;
        }

        std::string full_path = directory + "/" + folder_name;

        struct stat statbuf;
        if (stat(full_path.c_str(), &statbuf) != 0) {
            std::cout << "Failed to stat: " << full_path << std::endl;
            continue;
        }

        if (S_ISDIR(statbuf.st_mode)) {
            if (std::regex_match(folder_name, mapdata_regex)) {
                count++;
            }
        }
    }

    closedir(dir);

    return count + 1;
}

bool MapFusion::InitMapFusion(const std::vector<LaneMappingMsg>& lane_mapping_infos, const bool& is_map_saving) {
  #ifdef __QNX__
  if (true == lane_mapping_infos.empty() || false == is_map_saving) {
    return false;
  }
  #else
  if (true == lane_mapping_infos.empty()) {
    return false;
  }
  #endif
  //  没有重新上下电第二次建图初始化,在第一次建完图并下发成功之后5开始初始化
  #ifdef __QNX__
  #else
    // 验证x86下一个点火周期内下多次建图需要将该行注释掉
    map_fusion_static_var_.all_data_sent_cntr = map_fusion_static_var_.mapfusion_cntr;
  #endif

  if (map_fusion_static_var_.mapfusion_cntr > map_fusion_static_var_.all_data_sent_cntr + 30 &&
      map_fusion_static_var_.all_data_sent == true) {
        std::cout << "Mapfusion Init" << std::endl;
    map_fusion_static_var_.map_fusion_inti_times++;
    map_fusion_static_var_.all_data_sent_cntr = 0;
    map_fusion_static_var_.map_id = 0;
    map_fusion_static_var_.map_fusion_valid_cnt = 0;
    map_fusion_static_var_.is_map_process_end = false;
    map_fusion_static_var_.current_position_index_ = 0;
    map_fusion_static_var_.current_polygon_index_ = 0;
    map_fusion_static_var_.position_packet_index_ = 1;
    map_fusion_static_var_.polygon_packet_index_ = 1;
    map_fusion_static_var_.all_data_sent = false;
    map_fusion_static_var_.position_semantic_infos_res_.clear();
    map_fusion_static_var_.links_polygon_.clear();
    map_fusion_static_var_.last_ppoint2efm.point_infos.clear();
    map_fusion_static_var_.last_ppolygon2efm.PathId = 0;
    map_fusion_static_var_.last_ppolygon2efm.Index = 0;
    map_fusion_static_var_.last_ppolygon2efm.Count = 0;
    map_fusion_static_var_.last_ppolygon2efm.polygon_infos.clear();
    map_fusion_static_var_.last_clane2efm.PathId = 0;
    map_fusion_static_var_.last_clane2efm.Index = 0;
    map_fusion_static_var_.last_clane2efm.Count = 0;
    map_fusion_static_var_.last_clane2efm.Clane_infos.clear();
    map_fusion_static_var_.last_clane2efm.PathId = 0;
    map_fusion_static_var_.last_clane2efm.Index = 0;
    map_fusion_static_var_.last_clane2efm.Count = 0;
    map_fusion_static_var_.get_map_id_times = 0;
    return false;
  }
  return true;
}

bool MapFusion::CheckMapId(MapFusionStatus* map_fusion_status) {
  if (map_fusion_static_var_.map_id < 1 || map_fusion_static_var_.map_id > 10) {
    map_fusion_status->map_saving_status = 7;
    map_fusion_status->map_id = 0;
    map_fusion_status->map_distance = 0.0f;

    map_fusion_static_var_.map_fusion_valid_cnt = 0;
    map_fusion_static_var_.is_map_process_end = false;
    std::cout << "map_id error" << std::endl;
    return false;
  }
  return true;
}


bool MapFusion::MapProcess(const std::vector<LaneMappingMsg>& lane_mapping_infos, const std::vector<Line>& line_out) {
  if (make_input_data_->SaveData() == false) {
    // 保存数据
    // return false;
  }

  if (false == make_input_data_->LocalDataOK()) {
    // local data 未做成
    // return false;
  }

  std::cout << __FILE__ << __LINE__ << "MapProcess::make_input_data_()" << std::endl;
  if (make_input_data_->Execute(lane_mapping_infos, line_out, map_fusion_static_var_.map_id) == false) {
    // 数据匹配
    // to add error code
    // return false;
  }
  std::cout << __FILE__ << __LINE__ << "MapProcess::make_input_data_()" << std::endl;
  if (make_map_fusuion_->Execute(make_input_data_, line_out) == false) {
    // 中心线，split/merge, 连接关系
    // to add error code
    return false;
  }

  if (make_output_data_->Execute(make_map_fusuion_, map_fusion_static_var_.map_id) == false) {
    // geoJson输出
    // to add error code
    return false;
  }

  return true;
}

}  // namespace MapFusion
}  // namespace CommuteMode
