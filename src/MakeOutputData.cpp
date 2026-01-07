#include "MakeOutputData.h"

namespace CommuteMode {
namespace MapFusion {
MakeOutputTimes MakeOutputData::make_out_put_times_;

bool MakeOutputData::Execute(const std::shared_ptr<MakeMapFusion>& make_mapfusion_data, const uint8_t& route_id) {
    //  init variable
    map_common_tool_ = MapCommonTool::GetInstance();
    map_common_tool_->GetIntersectionInfos(intersection_infos_);
    map_common_tool_->GetPositionInfos(position_infos_);
    map_common_tool_->GetLinkDataInfos(link_data_infos_);
    map_common_tool_->GetMapHdnvInfos(map_hdnv_infos_);
    map_common_tool_->GetLaneMappingInfos(lane_mapping_infos_);
    traj_point_infos_ = make_mapfusion_data->GetTrajPointInfos();
    position_semantic_infos_ = make_mapfusion_data->GetPositionPointInfos();
    memory_cross_lane_infos_ = make_mapfusion_data->GetCrossLaneInfos();

    //  process
    if (false == Makelink()) {
    return false;
    }

    if (false == Makelinklane()) {
    return false;
    }

    if (false == MakePosition(position_semantic_infos_, route_id)) {
        return false;
    }

    if (false == Makepolygon(traj_point_infos_, route_id)) {
        return false;
    }

    if (false == MakeCrossLane(memory_cross_lane_infos_, route_id)) {
        return false;
    }

    return true;
}

bool MakeOutputData::Makelink() {
    return true;
}

bool MakeOutputData::Makelinklane() {
    return true;
}

bool MakeOutputData::MakePosition(const std::vector<PositionPointInfo>& position_semantic_infos,
                                const uint8_t& route_id) {
    // if (false == position_semantic_infos.empty() && make_out_put_times_.output_load_position_times == 0) {
    if (false == position_semantic_infos.empty()) {
        std::cout << "*************** position to geojson start ***************" << std::endl;
        // std::vector<PositionPointInfo> position_semantic_infos_res{};
        // ConvertAndCalculateOffsets(position_semantic_infos, &position_semantic_infos_res);
        position_semantic_infos_res_ = position_semantic_infos;
        CDataToJson data_to_json;
        data_to_json.MakeMemoryPosition(position_semantic_infos, route_id);
        // make_out_put_times_.output_load_position_times++;
        std::cout << "*************** position to geojson end ***************" << std::endl;
        return true;
    } else {
        return false;
    }
}

bool MakeOutputData::MakeCrossLane(const std::vector<CrossLaneInfo>& memory_cross_lane_infos, const uint8_t& route_id) {
    CDataToJson data_to_json;
    data_to_json.MakeCrossLaneInfos(memory_cross_lane_infos, route_id);
    return true;
}

bool MakeOutputData::Makepolygon(const std::vector<TrajPointInfo>& traj_point_infos,
                                const uint8_t& route_id) {
    size_t traj_point_infos_size = traj_point_infos.size();
    // if (traj_point_infos_size > 0 && make_out_put_times_.output_load_polygon_times == 0) {
    if (traj_point_infos_size > 0) {
        std::cout << "*************** position polygon process start ***************" << std::endl;
        std::vector<LinkPolygon> links_polygon{};
        std::vector<TrajInfo> traj_infos{};
        std::vector<TrajInfo> traj_infos_2{};
        uint16_t start_idx = 0;
        uint32_t id = 1;
        bool is_add_first_point = false;
        double accumulated_distance = 0.0;
        TrajPointInfo prev_point = traj_point_infos[0];
        for (size_t i = 1; i < traj_point_infos.size(); ++i) {
            TrajPointInfo curr_point = traj_point_infos[i];

            Point2Dd  cur_point{}, cur_point_ego{};
            DoublePosePoint pre_point{};
            cur_point.x =  traj_point_infos[i].position.PosLgt;
            cur_point.y =  traj_point_infos[i].position.PosLat;
            pre_point.x = prev_point.position.PosLgt;
            pre_point.y = prev_point.position.PosLat;
            pre_point.yaw = prev_point.position.Heading;
            WGS84ToBody(cur_point, pre_point, cur_point_ego);
            double distance = Distance(0.0, 0.0, cur_point_ego.x, cur_point_ego.y) *100;
            accumulated_distance += distance;

            if (accumulated_distance >= 125.0) {
                if (curr_point.road_id != prev_point.road_id ||
                    curr_point.position.LaneNumber != prev_point.position.LaneNumber ||
                    // curr_point.position.EgoInLane != prev_point.position.EgoInLane ||
                    curr_point.position.RoadType != prev_point.position.RoadType) {
                    // float point_distance = Haversine(prev_point.position.PosLat, prev_point.position.PosLgt,
                    //                                 curr_point.position.PosLat, curr_point.position.PosLgt);

                    // std::cout << "pre_point.id: " << prev_point.position.PositionID << std::endl;
                    // std::cout << "curr_point.id: " << curr_point.position.PositionID << std::endl;
                    Point2Dd  cur_point{}, cur_point_ego{};
                    DoublePosePoint pre_point{};
                    cur_point.x = curr_point.position.PosLgt;
                    cur_point.y = curr_point.position.PosLat;
                    pre_point.x = prev_point.position.PosLgt;
                    pre_point.y = prev_point.position.PosLat;
                    pre_point.yaw = prev_point.position.Heading;
                    WGS84ToBody(cur_point, pre_point, cur_point_ego);
                    double distance_tmp = Distance(0.0, 0.0, cur_point_ego.x, cur_point_ego.y);
                    float point_distance  = static_cast<float>(distance_tmp);
                    // std::cout << "point_distance: " << point_distance << std::endl;
                    TrajInfo traj_info;
                    traj_info.road_id = id++;
                    traj_info.nearest_line_id = prev_point.nearest_line_id;
                    traj_info.start_id = start_idx + 1;
                    traj_info.end_id = i;
                    traj_info.forward_distance = point_distance;

                    // if (i >= 1 && false == is_add_first_point) {
                    //     traj_info.positions.push_back(traj_point_infos[0].position);
                    //     is_add_first_point = true;
                    // }
                    for (size_t j = start_idx; j < i; ++j) {
                        traj_info.positions.push_back(traj_point_infos[j].position);
                    }

                    traj_infos.emplace_back(traj_info);
                    start_idx = i;
                }
                prev_point = curr_point;
                accumulated_distance = 0.0;
            }
        }

        // 处理最后一段
        if (start_idx < traj_point_infos.size()) {
            TrajInfo traj_info;
            traj_info.road_id = id;
            traj_info.nearest_line_id = traj_point_infos.back().nearest_line_id;
            traj_info.start_id = start_idx;
            traj_info.end_id = traj_point_infos.size();
            traj_info.forward_distance = 1.0f;
            for (size_t j = start_idx; j < traj_point_infos.size(); ++j) {
                traj_info.positions.push_back(traj_point_infos[j].position);
            }
            traj_infos.emplace_back(traj_info);
        }
        // for (const auto traj_info : traj_infos) {
        //     std::cout << "traj_info.start_id: " << traj_info.start_id << std::endl;
        //     std::cout << "traj_info.end_id: " << traj_info.end_id << std::endl;
        //     std::cout << "traj_info.PositionID: " << traj_info.positions.back().PositionID << std::endl;
        // }


        for (size_t i = 0; i < traj_infos.size(); ++i) {
            if (i == 0) {
                // 第一个 traj_info 直接 push 到 traj_infos_2
                traj_infos_2.push_back(traj_infos[i]);
                traj_infos_2.back().road_id = traj_infos_2.size();  //  设置 id
            } else {
                if (traj_infos[i].positions.size() < 8) {
                    // 将当前 traj_info 的 end_id 赋值给 traj_infos_2 中最后一个元素的 end_id
                    traj_infos_2.back().end_id = traj_infos[i].end_id;
                    traj_infos_2.back().forward_distance = traj_infos[i].forward_distance;

                    // 将当前 traj_info 的 positions 的点 push 到 traj_infos_2 中最后一个元素的 positions
                    traj_infos_2.back().positions.insert(traj_infos_2.back().positions.end(),
                                                        traj_infos[i].positions.begin(),
                                                        traj_infos[i].positions.end());
                } else {
                    // 将当前 traj_info push 到 traj_infos_2
                    traj_infos_2.push_back(traj_infos[i]);
                    traj_infos_2.back().road_id = traj_infos_2.size();  //  设置 id
                }
            }
        }

        // for (const auto traj_info : traj_infos_2) {
        //     std::cout << "traj_infos_2.start_id: " << traj_info.start_id << std::endl;
        //     std::cout << "traj_infos_2.end_id: " << traj_info.end_id << std::endl;
        //     std::cout << "traj_info.PositionID: " << traj_info.positions.back().PositionID << std::endl;
        // }

        PolygonCreate(traj_infos_2, &links_polygon);
        if (false == links_polygon.empty()) {
            std::cout << "*************** position polygon to geojson start ***************" << std::endl;
            links_polygon_ = links_polygon;
            CDataToJson data_to_json;
            data_to_json.MakeMemoryLinkPolygen(links_polygon, route_id);
            // make_out_put_times_.output_load_polygon_times++;
            std::cout << "*************** position polygon to geojson start ***************" << std::endl;
        }
        std::cout << "*************** position polygon process end ***************" << std::endl;
        return true;
    } else {
        return false;
    }
}

bool MakeOutputData::PolygonCreate(const std::vector<TrajInfo>& traj_infos,  std::vector<LinkPolygon>* links_polygon) {
    links_polygon->clear();

    if (true == traj_infos.empty()) {
        return false;
    }
    for (const auto& traj_info : traj_infos) {
        DoublePosePoint cor_trans_base_point;
        if (true == traj_info.positions.empty()) {
            std::cout << __FILE__ << __LINE__ << "traj_info.positions is empty" << std::endl;
            std::cout << __FILE__ << __LINE__ << "traj_info.road_id: " << traj_info.road_id << std::endl;
            continue;
        } else {
            cor_trans_base_point.x = traj_info.positions[0].PosLgt;
            cor_trans_base_point.y = traj_info.positions[0].PosLat;
            cor_trans_base_point.yaw = traj_info.positions[0].Heading;
        }

        std::vector<Point2Dd> points_ego{};
        for (const auto& point : traj_info.positions) {
            Point2Dd point_ego, raw_point;
            raw_point.x = point.PosLgt;
            raw_point.y = point.PosLat;
            WGS84ToBody(raw_point, cor_trans_base_point, point_ego);
            points_ego.emplace_back(point_ego);
        }
        uint8_t lane_id_gap = (traj_info.positions[0].LaneNumber - traj_info.positions[0].EgoInLane) > 0 ?
                            (traj_info.positions[0].LaneNumber - traj_info.positions[0].EgoInLane) : 0;
        double buffer_distance = 6.5;
        double left_translate_distance = lane_id_gap * 3.5 + buffer_distance;
        uint8_t ego_lane_id = traj_info.positions[0].EgoInLane > 1 ? traj_info.positions[0].EgoInLane - 1 : 1;
        double right_translate_distance = -((ego_lane_id - 1) * 3.5 + buffer_distance);;
        double extend_distance = traj_info.forward_distance + 0.2;
        std::vector<Point2Dd> extended_line_points = ExtendLine(points_ego, extend_distance);
        std::vector<Point2Dd> translated_left_line_tmp = TranslateLine(extended_line_points, left_translate_distance);
        std::vector<Point2Dd> translated_right_line_tmp = TranslateLine(extended_line_points, right_translate_distance);
        std::vector<Point2Dd> translated_left_line_gap3{}, translated_right_line_gap3{};
        size_t translated_left_line_tmp_size = translated_left_line_tmp.size();
        size_t translated_right_line_tmp_size = translated_right_line_tmp.size();
        for (size_t i = 0; i < translated_left_line_tmp_size; i++) {
            translated_left_line_gap3.push_back(translated_left_line_tmp[i]);
        }
        for (size_t i = 0; i < translated_right_line_tmp_size; i ++) {
            translated_right_line_gap3.push_back(translated_right_line_tmp[i]);
        }

        std::vector<Point2Dd> translated_left_line{}, translated_right_line{};
        double epsilon = 0.5;
        DougLasPeucker(translated_left_line_gap3, epsilon, &translated_left_line);
        DougLasPeucker(translated_right_line_gap3, epsilon, &translated_right_line);


        std::vector<Point2Dd> res_line1_gnss{}, res_line2_gnss{};
        for (const auto& point : translated_left_line) {
                Point2Dd raw_point, point_wgs84;
                raw_point.x = point.x;
                raw_point.y = point.y;
                BodyToWGS84(raw_point, cor_trans_base_point, point_wgs84);
                res_line1_gnss.emplace_back(point_wgs84);
        }
        for (const auto& point : translated_right_line) {
            Point2Dd raw_point, point_wgs84;
            raw_point.x = point.x;
            raw_point.y = point.y;
            BodyToWGS84(raw_point, cor_trans_base_point, point_wgs84);
            res_line2_gnss.emplace_back(point_wgs84);
        }

        LinkPolygon link_polygon;
        link_polygon.link_id = traj_info.road_id;
        link_polygon.start_point_id = traj_info.start_id;
        link_polygon.end_point_id = traj_info.end_id;
        auto link_pologon_points = GeneratePolygon(res_line1_gnss, res_line2_gnss);
        link_polygon.line_coordinates = link_pologon_points;
        links_polygon->emplace_back(link_polygon);
    }
    return true;
}

template <typename T>
std::vector<T> MakeOutputData::GeneratePolygon(const std::vector<T>& line1, const std::vector<T>& line2) {
    std::vector<T> polygon;
    for (const auto& point : line1) {
        polygon.push_back(point);
    }

    for (auto it = line2.rbegin(); it != line2.rend(); ++it) {
        polygon.push_back(*it);
    }

    return polygon;
}

template <typename T>
std::vector<T> MakeOutputData::TranslateLine(const std::vector<T>& line, double translate_distance) {
    std::vector<T> translated_line{};
    for (size_t i = 0; i < line.size() - 1; ++i) {
        T p1 = line[i];
        T p2 = line[i + 1];
        T normal = NormalVector(p1, p2);
        translated_line.push_back(TranslatePoint(p1, normal, translate_distance));
    }
    // 处理最后一个点
    if (!line.empty()) {
        T p1 = line[line.size() - 2];
        T p2 = line[line.size() - 1];
        T normal = NormalVector(p1, p2);
        translated_line.push_back(TranslatePoint(p2, normal, translate_distance));
    }
    return translated_line;
}

template <typename T>
std::vector<T> MakeOutputData::ExtendLine(const std::vector<T>& line, double extend_distance) {
    if (line.size() < 2) {
        std::cout << "line.size() < 2" << std::endl;
        return line;    // 如果线的点数少于2，无法延伸
    }

    if (extend_distance < 0.1) {
        extend_distance = 0.1;
    }

    std::vector<T> extended_line = line;

    // 计算起点的延伸点
    // T start_direction = DirectionVector(line[1], line[0]);
    // T extended_start = ExtendPoint(line[0], start_direction, extend_distance);
    // extended_line.insert(extended_line.begin(), extended_start);

    // 计算终点的延伸点
    T end_direction = DirectionVector(line[line.size() - 2], line[line.size() - 1]);
    T extended_end = ExtendPoint(line[line.size() - 1], end_direction, extend_distance);
    extended_line.push_back(extended_end);

    return extended_line;
}

void MakeOutputData::ConvertAndCalculateOffsets(const std::vector<PositionPointInfo>& position_semantic_infos,
                                std::vector<PositionPointInfo>* position_semantic_infos_res) {
    size_t total_points = position_semantic_infos.size();
    size_t segment_size = 100;
    double accumulated_distance = 0.0;

    for (size_t i = 0; i < total_points; i += segment_size) {
        size_t segment_end = std::min(i + segment_size, total_points);
        DoublePosePoint base_point_pos_wgs84;
        base_point_pos_wgs84.x = position_semantic_infos[i].position.x;
        base_point_pos_wgs84.y = position_semantic_infos[i].position.y;
        base_point_pos_wgs84.yaw = position_semantic_infos[i].heading;

        for (size_t j = i; j < segment_end; ++j) {
            Point2Dd point_body;
            WGS84ToBody(position_semantic_infos[j].position, base_point_pos_wgs84, point_body);
            if (j > i) {
                Point2Dd point_bod_pre;
                if (true == position_semantic_infos_res->empty()) {
                    point_bod_pre.x = 0;
                    point_bod_pre.y = 0;
                } else {
                    WGS84ToBody(position_semantic_infos_res->back().position, base_point_pos_wgs84, point_bod_pre);
                }
                double distance_tmp = Distance(point_bod_pre.x, point_bod_pre.y, point_body.x, point_body.y);
                accumulated_distance += distance_tmp;
            }

            PositionPointInfo new_point = position_semantic_infos[j];
            new_point.position = position_semantic_infos[j].position;
            new_point.path_offset = static_cast<uint32_t>(accumulated_distance * 100);
            position_semantic_infos_res->push_back(new_point);
        }
    }
    return;
}

void MakeOutputData::GetIntersectionArrow(std::vector<PositionPointInfo>* position_semantic_infos) {
    size_t last_intersection_end_point_idx = 0;
    size_t position_semantic_infos_size = position_semantic_infos->size();
    size_t min_changepoint_offset_idx = position_semantic_infos_size;  //  初始化为无效索引
    for (size_t endidx = 0; endidx < position_semantic_infos_size; ++endidx) {
        // 距离路口的距离小于5米
        if (position_semantic_infos->at(endidx).changepoint_offset < 5) {
            // 记录 changepoint_offset 最小的点
            if (min_changepoint_offset_idx == position_semantic_infos_size ||
                position_semantic_infos->at(endidx).changepoint_offset <
                position_semantic_infos->at(min_changepoint_offset_idx).changepoint_offset) {
                min_changepoint_offset_idx = endidx;
            }
        } else if (position_semantic_infos->at(endidx).changepoint_offset >= 100 &&
                min_changepoint_offset_idx != position_semantic_infos_size) {
            // 找到某个路口的最近点的箭头方向
            uint8_t intersection_chang_dir = position_semantic_infos->at(min_changepoint_offset_idx).changepoint_dir;

            // 从某个路口的最近点开始往回找符合特征的点并修改对应的箭头方向
            for (size_t curidx = min_changepoint_offset_idx; curidx > last_intersection_end_point_idx; --curidx) {
                if (position_semantic_infos->at(curidx).path_offset + 100 <=
                    position_semantic_infos->at(min_changepoint_offset_idx).path_offset ||
                    (position_semantic_infos->at(curidx).changepoint_offset != 2000 &&
                        position_semantic_infos->at(curidx).changepoint_offset <
                        position_semantic_infos->at(min_changepoint_offset_idx).changepoint_offset)) {
                    position_semantic_infos->at(curidx).changepoint_dir = intersection_chang_dir;
                }
            }
            last_intersection_end_point_idx = min_changepoint_offset_idx;
            min_changepoint_offset_idx = position_semantic_infos_size;  // 重置为无效索引
        }
    }
    // for (size_t endidx = 0; endidx < position_semantic_infos_size; ++endidx) {
    //     // 距离路口的距离小于5米
    //     if (position_semantic_infos[endidx].changepoint_offset < 5) {
    //         for (size_t startidx = endidx + 1; startidx < position_semantic_infos_size; ++startidx) {
    //             // 距离路口的距离大于100米
    //             if (position_semantic_infos[startidx].changepoint_offset >= 100) {
    //                 // 找到某个路口的最近点的箭头方向
    //                 uint8_t intersection_chang_dir = position_semantic_infos[endidx].changepoint_dir;

    //                 // 从某个路口的最近点开始往回找符合特征的点并修改对应的箭头方向
    //                 for (size_t curidx = endidx - 1; curidx > last_intersection_end_point_idx; --curidx) {
    //                     if (position_semantic_infos[curidx].path_offset + 100 <=
    //                         position_semantic_infos[endidx].path_offset||
    //                         (position_semantic_infos[curidx].changepoint_offset != 2000 &&
    //                          position_semantic_infos[curidx].changepoint_offset <
    //                          position_semantic_infos[endidx].changepoint_offset)) {
    //                         position_semantic_infos[curidx].changepoint_dir = intersection_chang_dir;
    //                     }
    //                 }
    //                 last_intersection_end_point_idx = endidx;
    //                 break;
    //             }
    //         }
    //     }
    // }
    return;
}


}  // namespace MapFusion
}  // namespace CommuteMode
