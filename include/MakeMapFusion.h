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
#include "common/framework.h"
#include "store/yaml.h"
#include "MapCommonTool.h"
#include "CrossPlanning.h"
#include "trans_coordinate.h"
#include "data_to_geojson.h"
#include "calibration.h"
#include "MakeInputData.h"
#include "common_math.h"

namespace CommuteMode {
namespace MapFusion {

class MakeMapFusion {
public:
    MakeMapFusion() {};
    ~MakeMapFusion() = default;
    bool Execute(const std::shared_ptr<MakeInputData>& make_input_data, const std::vector<Line>& line_out);
    std::vector<TrajPointInfo> GetTrajPointInfos() { return traj_point_infos_; }
    std::vector<PositionPointInfo> GetPositionPointInfos() { return position_semantic_infos_; }
    std::vector<CrossLaneInfo> GetCrossLaneInfos() { return memory_cross_lane_infos_; }

private:
    //合并linklist
    bool ChangeLinkList();
    //做成中心线
    //参数：
    bool MakeCenter(const std::shared_ptr<MakeInputData>& make_input_data);
    //做成路段内中心线
    //参数：
    bool MakeRoadSectionCenter();
    //做成split中心线
    //参数：
    bool MakeSplitCenter();
    //做成merge中心线
    //参数：
    bool MakeMergeCenter();
    //做成路口内中心线
    //参数：
    bool MakeIntersectionCenter();

    //做成连接关系，最多一分二
    bool MakeConnect();

    bool MakePosition(const std::shared_ptr<MakeInputData>& make_input_data,
                      const std::vector<Line>& line_out);

    //做成限速、道路类型等其他属性
    bool MakeOtherField();

    bool cmp(LinkMemory& data1, LinkMemory& data2);

    //     //获取merge、split、roadsection、intersection需要生成中心线的link
    // bool GetSpecialLinksInfo(const std::vector<uint32_t>& link_list, const std::vector<LinkMemory>& link_data_infos,
    //                         std::vector<GenerateCenterLineLink>& links_info);

    bool GenerateCenterLine(const std::vector<Point2Dd> & enter_center_line_points_body, 
                            const std::vector<Point2Dd> & exit_center_line_points_body,
                            const DoublePosePoint& ego_pose_point, const uint8_t& section_type,
                            LineAttribute & intersection_center_line_points_wgs84);
    bool GenerateCenterLines(const uint8_t& enter_ego_lane_idx,  const uint8_t& exit_ego_lane_idx,
                              const DoublePosePoint& ego_pose_point, const uint8_t& section_type,
                              const bool& is_bigcurv_intersection, const uint16_t& intersection_id, 
                              std::vector<CrossLaneInfo>& enter_centerlines_body,
                              std::vector<CrossLaneInfo>& exit_centerlines_body,
                              std::vector<CrossLaneInfo>& intersection_center_lines_wgs84,
                              bool& is_right_has_lane);
    bool StoreTarCenterLines(const uint8_t& lane_idx,const std::vector<std::vector<Point2Dd>>& center_lines_body,
                             std::vector<std::vector<Point2Dd>>& tar_center_lines_body);
    std::vector<Point2Dd> GenerateInterpolatedLine(const Point2Dd& end_point_first, const Point2Dd& start_point_second, double interval);
    Point2Dd CalculateControlPoint(const Point2Dd& A1, const Point2Dd& A3, const Point2Dd& B1, double k);
    std::vector<Point2Dd> GenerateBezierCurve(const Point2Dd& P0, const Point2Dd& P1, const Point2Dd& P2,
                                                const Point2Dd& P3, const double& segment_length);
    std::vector<Point2Dd> BoundryGenerateCenterLine(const std::vector<Point2Dd>& lane1, const std::vector<Point2Dd>& lane2);
    bool BoundrysGenerateCenterLines(const std::vector<std::vector<LPoint>>& enter_boundrys_wgs84,
        const uint16_t& intersection_id, const bool& is_enter, const DoublePosePoint& ego_pose_point,
        std::vector<CrossLaneInfo>& enter_centerline_body);
    uint8_t GetMostFrequentRoadType(const LaneMappingMemory& memory_lane_mapping_infos);
    uint8_t GetMostFrequentLaneType(const LaneMappingMemory& memory_lane_mapping_info, const uint8_t& lane_road_type);
    double CalculateLineLength(const std::vector<Point2Dd>& line);
    uint64_t FindNearestLineId(const Point2Dd& point, const std::vector<CrossLaneInfo>& lines);
    bool FindLaneChangePoints(const std::vector<Line>& line_out,
                            std::vector<LaneChangePointInfo>* lane_change_points);
    bool ProcessCrossLanePoints(std::vector<PositionPointInfo>& position_semantic_infos);
    bool ProcessSplitLanePoints(std::vector<PositionPointInfo>& position_semantic_infos);

    std::vector<LPoint> ConvertToLPoints(const std::vector<Point2Dd>& points) {
        std::vector<LPoint> lpoints{};
        for (const auto& point : points) {
            lpoints.push_back(LPoint{0, 0, point.x, point.y, 0});
        }
        return lpoints;
    }
 
    MapCommonTool* map_common_tool_;
    std::vector<Intersection> intersection_infos_{};
    std::vector<PositionInfo> position_infos_{};
    std::vector<LinkMemory> link_data_infos_{};
    std::vector<MapHdnvInfo> map_hdnv_infos_{};
    std::vector<LaneMappingMsg> lane_mapping_infos_{};
    std::vector<uint32_t> link_list_{};
    std::vector<PositionPointInfo> position_semantic_infos_{};
    std::vector<uint8_t> position_points_lanenum_{};
    std::vector<TrajInfo> lane_mapping_traj_infos_{};
    std::vector<TrajPointInfo> traj_point_infos_{};
    std::vector<CrossLaneInfo> memory_cross_lane_infos_{};
    std::vector<LaneMappingMemory> memory_lane_mapping_infos_{};
    std::unordered_map<uint32_t, DoublePosePoint> traj_points_position_map_{};

    uint32_t max_link_id_;
    static MemoryFileTimes memory_file_times_;


};

}  // namespace MapFusion
}  // namespace CommuteMode
