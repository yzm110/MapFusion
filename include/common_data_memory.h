#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip> 
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <sstream>

#include "common_point.h"
// #include "commom_line.h"
#include "topic/topic_trait.h"
#include "compile_config.h"
#include "common/framework.h"
#include "store/yaml.h"
#include "message.h"
#include "../mapping_out.hpp"

typedef message::map_link_data::s_LinkData_t LinkData;
typedef message::map_link_data::s_Intersection_t Intersection;
typedef message::map_link_data::s_Lane_t Lane;
typedef message::map_link_data::s_LaneConnect_t LaneConnect;
typedef message::map_hdnv_info::map_hdnv_info MapHdnvInfo;
typedef message::ppoint2efm::ppoint2efm PPOINT2EFM;
typedef message::ppolygon2efm::ppolygon2efm PPOLYGON2EFM;
typedef message::clane2efm::clane2efm CLANE2EFM;
typedef message::ppoint2efm::s_PPoint_t PPoint;
typedef message::ppolygon2efm::s_PPolygon_t PPolygon;



namespace CommuteMode {
namespace MapFusion {
using namespace earth::shell::framework;
// using MapPositionMsg = earth::shell::framework::TopicTrait::MapPositionMsg;
#define EFM_MIN(x, y) ((x) < (y) ? (x) : (y))
#define EFM_MAX(x, y) ((x) > (y) ? (x) : (y))


enum RefDataType{
    TRACE_STRAIGHT = 0,
    LANE_CHANGE_LEFT = 1,
    LANE_CHANGE_RIGHT = 2,
    TRACE_LEFT = 3,
    TRACE_RIGHT = 4,
};

enum TrafficStatusType{
    AVAILABLE = 0,
    NOT_AVAILABLE = 1,
    DECELERATE = 2,
};

enum  LineTypeDot{
    INSIDE_LANE = 0,
    LANE_CHANGE = 1,
    INTERSECTION = 2,
};

enum DirType{
    DirType_UNKNOW = 0,
    DirType_STRAIGHT = 1,//Intersection
    DirType_LEFT = 2,//Intersection
    DirType_RIGHT = 3,//Intersection
    DirType_SPLIT_STRAIGHT = 4,
    DirType_SPLIT_LEFT = 5,
    DirType_SPLIT_RIGHT = 6,
    DirType_LEFT_UTURN = 7,//Intersection
    DirType_RIGHT_UTURN = 8,//Intersection
    DirType_MERGE_STRAIGHT = 9,
    DirType_MERGE_TO_LEFT = 10,
    DirType_MERGE_TO_RIGHT = 11,
};

struct Point{
    Point()
    : x(0.0)
    , y(0.0)
    {}
    Point(double x_in, double y_in) : x(x_in), y(y_in) {}
    ~Point() = default;

    double x;  //longitude:eg. 最小值: -180°（西经）最大值: 180°（东经）
    double y;  //latitude:eg. 30.0 最小值: -90°（南极）最大值: 90°（北极）

    // 点+
    Point operator +(const Point &b) const {
        return Point(x + b.x, y + b.y);
    }

    // 点-
    Point operator -(const Point &b) const {
        return Point(x - b.x, y - b.y);
    }

    // 点积
    double operator *(const Point &b) const {
        return x * b.x + y * b.y;
    }

    // 叉乘
    double operator ^(const Point &b) const {
        return x * b.y - y * b.x;
    }
};

struct LineAttribute{
    double length{0.0};
    std::vector<Point2Dd> points{};
};


// struct Position {
//   uint8_t LaneNumber{0};
//   uint8_t EgoInLane{0};
//   uint8_t RoadType{0};
//   uint8_t SpeedLimitMax{0};
//   uint64_t PosTime{0};
//   uint64_t PositionID{0};
//   double StopLineOffset{0.0};
//   double PosLat{0.0};
//   double PosLgt{0.0};
//   double Heading{0.0};
//   std::vector<uint8_t> LaneType{};  // 数量为LaneNumber
//   std::vector<uint32_t> RelateLineID{};
// };

struct LaneChangePointInfo {
        uint32_t point_raw_id{0};
        bool is_change{false};
        uint8_t change_dir{0};  // 1: left, 2: right
};

struct TrajInfo {
    uint32_t road_id{0};
    uint64_t nearest_line_id{0};
    uint16_t start_id;
    uint16_t end_id;
    float forward_distance{0.0};
    std::vector<Position> positions{};
};

struct TrajPointInfo{
    uint32_t road_id{0};
    uint64_t nearest_line_id{0};
    LaneChangePointInfo lane_change_point_info{};
    Position position{};
};


// struct LPoint {
//   uint8_t LineType{0};
//   uint8_t Color{0};
//   double X{0.0};
//   double Y{0.0};
//   uint64_t PositionID{0};
// };

// struct LaneMappingMsg {
//   uint32_t RoadId{0};
//   std::vector<Position> TrajHistory;
//   std::map<uint32_t, std::vector<LPoint>> LaneData;  // key为LineId
// };

struct LaneInfo {
  uint8_t ArrowType{0};
  uint8_t LaneType{0};
  uint16_t LineValidNum{0};
  uint32_t MappingId{0};
  std::vector<LPoint> LineDatas{};
};

struct MemoryFileTimes{
    uint8_t memory_postion_times{0};
    uint8_t memory_position_polygon_times{0};
    uint8_t memory_cross_lane_info_times{0};
    // uint8_t output_load_position_times{0};
    // uint8_t output_load_polygon_times{0};
};

struct MakeOutputTimes{
    uint8_t output_load_position_times{0};
    uint8_t output_load_polygon_times{0};
};

struct GenerateCenterLineLane{
    uint8_t pre_lane_id{0};
    uint8_t cur_lane_id{0};
    uint8_t next_lane_id{0};
    DirType cur_lane_dir_type{DirType_UNKNOW};
    uint8_t cur_lane_arrow_type{0};
};

struct GenerateCenterLineLink{
    uint32_t pre_link_id{0};
    uint32_t cur_link_id{0};
    uint32_t next_link_id{0};
    uint16_t link_memory_info_idx{0};
    std::vector<GenerateCenterLineLane> lanes_connect_info{};
    std::vector<LaneInfo> LineDatas{};
    TrajInfo TrajHistory{};
};


struct LaneMappingMemory{
  uint32_t LinkId{0};
  std::vector<std::vector<LPoint>> LaneData{};  // 从右往左排
  double link_heading{0};                       // 正北方向为0，正东为90度
  std::vector<Position> TrajHistory{};
  uint8_t enter_ego_lane_id{0};
  uint8_t exit_ego_lane_id{0};
};

struct lane_atteri{
  uint8_t LaneID{0};
  std::vector<LPoint> LBoundary{};      // 左边线行点
  std::vector<LPoint> RBoundary{};      // 右边线行点
  std::vector<LPoint> Center{};         // 中心线行点
  DirType  lane_dir{DirType_STRAIGHT};
  uint8_t next_lane_id{0};
  uint8_t back_lane_id{0};
};

struct LinkMemory{
    LinkData LinkData_info{};
    LaneMappingMemory lane_mapping{};

    //  std::vector<DirType> lane_dir{};// 每根车道一个记录
    uint32_t link_length{0};    // 单位cm
    uint32_t path_offset{0};    // 单位cm
    uint32_t path_all_length{0};    // 单位cm
    uint32_t new_link_id{0};    // 重新排列数据，重新生成ID，从1开始
    uint32_t org_link_id{0};
    std::vector<lane_atteri> lanes;
};

struct LaneTopo {
uint32_t road_id{0};
uint8_t lane_num{0};
uint32_t lane_id{0};
uint8_t topo_dir{0};
};
struct LaneConnectInfo {
uint32_t RoadId{0};
uint8_t LaneNum{0};
uint32_t LaneId{0};
std::vector<LaneTopo> Nexts{};
std::vector<LaneTopo> Pres{};
int64_t CategoryVehicle{0};
int64_t CategorySpecial{0};
int64_t CategoryLocation{0};
uint8_t Transition{0};
int8_t isVirtual{0};
uint8_t lane_arrow_type{0};
};

struct CenterLinePointInfo{
    double longitude{0.0};
    double latitude{0.0};
    double heading{0.0};
    double curvature{0.0};
    double sloope{0.0};
    double crossslope{0.0};
    uint32_t offset{0};
};
struct LinkDataCenterLine{
    LinkData link_data_info{};
    LaneConnectInfo lane_connect_infos{};
    std::vector<CenterLinePointInfo> center_line_infos{};
};

struct InterSectionCenterLine{
    uint32_t pre_link_id{0};
    uint32_t cur_link_id{0};
    uint32_t next_link_id{0};
    uint8_t pre_lane_id{0};
    uint8_t cur_lane_id{0};
    uint8_t next_lane_id{0};
    std::vector<Point2Dd> center_line{};
};

struct PositionInfo{
    double longitude{0.0};
    double latitude{0.0};
    float heading{0.0f};
    float speed{0.0f};
    float xAcc{0.0f};
    float yAcc{0.0f};
    float zAcc{0.0f};
    float angular_velocity_x{0.0f};
    float angular_velocity_y{0.0f};
    float angular_velocity_z{0.0f};
    uint8_t fail_safe_loc_status{0};
    uint8_t geo_fence_judge_status{0};
    uint8_t geo_fence_judge_type{0};
    uint32_t path_id{0};
    uint32_t link_id{0};
    uint8_t lane_id{0};
    uint32_t path_offset{0};
    uint64_t timestamp{0};
    uint64_t position_age{0};
};

struct PositionPointInfo{
    uint32_t id{0};
    uint32_t path_offset{0};
    uint32_t changepoint_offset{0};
    uint8_t changepoint_type{0};
    uint8_t changepoint_dir{0};
    uint64_t line_id{0};           //  离轨迹点最近的线的id
    uint8_t speed{0};
    float heading{0.0f};
    Point2Dd position{};
    uint32_t cross_point{0};
    uint8_t left_lane_type{0};
    uint8_t right_lane_type{0};
    uint8_t to_cross_point_dir{0};
    uint8_t to_cross_point_times{0};
    uint32_t raw_position_point_id{0};
    bool is_stop_line_end{false};
    LaneChangePointInfo lane_change_point_info{};
};

struct CrossLaneInfo{
    uint64_t id{0};
    uint8_t lane_idx{0};
    uint64_t pre_id_1{0};
    uint64_t pre_id_2{0};
    uint64_t pre_id_3{0};
    uint64_t next_id_1{0};
    uint64_t next_id_2{0};
    uint64_t next_id_3{0};
    uint64_t cross_id{0};
    uint8_t type{0};
    uint64_t length{0};
    std::vector<Point2Dd> geometry{};
};

struct IntersectionInfo{
    uint32_t path_id{0};
    uint32_t start_offset{0};
    uint32_t end_offset{0};
    uint32_t intersection_id{0};
    uint8_t type{0};
    std::vector<uint32_t> inner_road_ids{};
    std::vector<uint32_t> out_road_ids{};
    std::vector<Point> points{};
};

struct VehicleInfo{
    uint32_t link_id{0};
    uint8_t lane_id{0};
    Point position_wgs84{};
    float heading{0.0f};
    uint32_t offset{0}; // 车辆在当前link上的偏移量 单位cm
};

struct LinkLaneInfo{
    uint32_t link_id{0};
    uint8_t lane_num{0};
    DirType dir{DirType_UNKNOW};
};

struct MemoryLink{
    uint16_t id{0};
    float heading{0.0f};
    uint8_t lane_num{0};
    uint32_t link_id{0};
    DirType dir{DirType_UNKNOW};
    uint8_t route{0};
    uint32_t length{0};
    float lanewidth{0.0f};
    LinkLaneInfo link_lane_info1{};
    LinkLaneInfo link_lane_info2{};
    LinkLaneInfo link_lane_info3{};
    uint8_t stop_line{0};
    uint8_t traffic_light{0};
    uint8_t split_merge{0}; // 1：分歧向左;2：分歧向右;3：分歧直行;4：合流直行;5：合流向左;6：合流向右
    uint8_t s_laneid{0};
    uint8_t e_laneid{0};
    uint16_t speed{0};
    uint8_t is_memory{0};
    uint32_t path_offset{0};
    uint32_t path_length{0};
    std::vector<Point> coordinates{};
};

struct MemoryLinkLane{
    uint16_t id{0};
    uint32_t link_id{0};
    uint8_t lane_id{0};
    uint32_t rlength{0};
    uint8_t next_lane{0};
    uint32_t next_link{0};
};

struct MemoryPolygon{
    uint16_t id{0};
    uint32_t link_id{0};
    double maxx{0.0};
    double minx{0.0};
    double maxy{0.0};
    double miny{0.0};
    float heading{0.0f};
    std::vector<Point> coordinates{};
};

struct RefDataInfo{
    uint16_t s_node{0};
    uint16_t e_node{0};
    uint8_t reflaneid{0};
    RefDataType type{RefDataType::TRACE_STRAIGHT};
};

struct StaticLocationMoudle{
    bool is_make_init{false};
};

struct LaneElementMemory{
    std::vector<uint32_t> links{};
    std::vector<uint8_t> lanes{};
    std::vector<uint32_t> lengths{};
    std::vector<uint8_t> routes{};
    std::vector<uint8_t> splitmerges{};
    std::vector<DirType> nextdirs{};
    std::vector<DirType> dirs{};
    uint32_t rdist{0};
    uint8_t  cross_times{0};
    uint8_t  lane_number{0};
    bool     is_dest{false};
};

struct MemoryFilePath{
    std::string file_path_memory_link = " ";
    std::string file_path_memory_linklane = " ";
    std::string file_path_memory_polygon = " ";
};

struct LinkPolygon{
    uint32_t link_id{0};
    uint32_t start_point_id{0};
    uint32_t end_point_id{0};
    std::vector<Point2Dd> line_coordinates{};
};

struct MapFusionStaticVar{
    uint8_t map_id{0};
    uint32_t map_fusion_valid_cnt{0};
    bool is_map_process_end{false};
    size_t current_position_index_{0};
    size_t current_polygon_index_{0};
    uint8_t position_packet_index_{1};
    uint8_t polygon_packet_index_{1};
    bool all_data_sent{false};
    std::vector<PositionPointInfo> position_semantic_infos_res_{};
    std::vector<LinkPolygon> links_polygon_{};
    uint64_t mapfusion_cntr{0};
    PPOINT2EFM last_ppoint2efm{};
    PPOLYGON2EFM last_ppolygon2efm{};
    CLANE2EFM last_clane2efm{};
    uint64_t all_data_sent_cntr{0};
    uint8_t get_map_id_times{0};
    uint8_t map_fusion_inti_times{0};
};
struct LineInfo{
    uint8_t s_laneid{0};
    std::vector<Point> line_coordinates{};

    LineInfo(uint8_t laneid, const std::vector<Point>& coordinates)
        : s_laneid(laneid), line_coordinates(coordinates) {}
};

struct HQRawDataMap{
    HQRawDataMap()
    : intersection_InnerRoadsIds_map{}
    , link_data_RoadId_map{}
    , lane_mapping_memory_link_map{}
    , position_info_id_map{}
    , lane_mapping_msg_link_map{}{};
    ~HQRawDataMap() = default;

    std::unordered_map<uint32_t, std::vector<int>> intersection_InnerRoadsIds_map;//key-intersectio的InRoadsIdsroad_id;val-index in hq section vector;
    std::unordered_map<uint32_t, int> link_data_RoadId_map;//key-s_LinkData_t的RoadId;val-index in hq linkData vector;
    std::unordered_map<uint32_t, int> lane_mapping_memory_link_map;//key-link_id; val std::vector<lanemappingMemory>的索引
    std::unordered_map<uint64_t, int> link_memory_map;//key-s_LinkData_t的RoadId;val-index in LinkMemory vector;    
    std::unordered_map<uint32_t, int> position_info_id_map;//key-position_id; val std::vector<lanemappingMemory>的索引
    std::unordered_map<uint32_t, int> lane_mapping_msg_link_map;//key-LaneMappingMsg的LinkId;val-index in LaneMappingMsg vector;
};


struct MakeInputStasticVar{
    uint8_t json_write_times{0};
    uint8_t json_write_memory_center_line_times{0};
};


}
}
