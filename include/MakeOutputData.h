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

#include "common_data_memory.h"
#include "data_to_geojson.h"
#include "trans_coordinate.h"
#include "MapCommonTool.h"
#include "MakeMapFusion.h"
#include "common_math.h"

namespace CommuteMode {
namespace MapFusion {

class MakeOutputData {
 public:
  MakeOutputData() : map_common_tool_(nullptr) {}
  ~MakeOutputData() = default;
  bool Execute(const std::shared_ptr<MakeMapFusion>& make_input_data, const uint8_t& route_id);
  std::vector<PositionPointInfo> GetPositionPointInfos() { return position_semantic_infos_res_; }
  std::vector<LinkPolygon> GetLinksPolygon() { return links_polygon_; }

 private:
    std::shared_ptr<CDataToJson> data_to_json_;
 
 private:
    //  做成memory_link.geojson
    bool Makelink();

    //  memory_linklane.geojson
    bool Makelinklane();

    //  memory_polygon.geojson
    bool Makepolygon(const std::vector<TrajPointInfo>& traj_point_infos_, const uint8_t& route_id);
    bool MakePosition(const std::vector<PositionPointInfo>& position_semantic_infos, const uint8_t& route_id);
    bool PolygonCreate(const std::vector<TrajInfo>& position_semantic_traj_infos,
                       std::vector<LinkPolygon>* links_polygon);
    bool MakeCrossLane(const std::vector<CrossLaneInfo>& memory_cross_lane_infos, const uint8_t& route_id);

    void ConvertAndCalculateOffsets(const std::vector<PositionPointInfo>& position_semantic_infos,
                                std::vector<PositionPointInfo>* position_semantic_infos_res);
    template <typename T>
    std::vector<T> GeneratePolygon(const std::vector<T>& line1, const std::vector<T>& line2);
    void GetIntersectionArrow(std::vector<PositionPointInfo>* position_semantic_infos);

    template <typename T>
    T NormalVector(const T& point1, const T& point2) {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;
        double length = std::sqrt(dx * dx + dy * dy);
        return { -dy / length, dx / length };
    }

    template <typename T>
    T DirectionVector(const T& point1, const T& point2) {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;
        double length = std::sqrt(dx * dx + dy * dy);
        return { dx / length, dy / length };
    }

    template <typename T>
    T TranslatePoint(const T& p, const T& normal, double translate_distance) {
        return { p.x + normal.x * translate_distance, p.y + normal.y * translate_distance };
    }

    template <typename T>
    T ExtendPoint(const T& p, const T& direction, double extend_distance) {
        return { p.x + direction.x * extend_distance, p.y + direction.y * extend_distance };
    }

    template <typename T>
    std::vector<T> TranslateLine(const std::vector<T>& line, double translate_distance);

    template <typename T>
    std::vector<T> ExtendLine(const std::vector<T>& line, double extend_distance);

    MapCommonTool* map_common_tool_;
    std::vector<Intersection> intersection_infos_{};
    std::vector<PositionInfo> position_infos_{};
    std::vector<LinkMemory> link_data_infos_{};
    std::vector<MapHdnvInfo> map_hdnv_infos_{};
    std::vector<LaneMappingMsg> lane_mapping_infos_{};
    std::vector<TrajPointInfo> traj_point_infos_{};
    std::vector<PositionPointInfo> position_semantic_infos_{};
    std::vector<PositionPointInfo> position_semantic_infos_res_{};
    std::vector<LinkPolygon> links_polygon_{};
    std::vector<CrossLaneInfo> memory_cross_lane_infos_{};
    static MakeOutputTimes make_out_put_times_;

};

}  // namespace MapFusion
}  // namespace CommuteMode
