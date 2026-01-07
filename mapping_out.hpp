#pragma once

#include <map>
#include <vector>

typedef message::ppoint2efm::ppoint2efm PPOINT2EFM;
typedef message::ppolygon2efm::ppolygon2efm PPOLYGON2EFM;
typedef message::clane2efm::clane2efm CLANE2EFM;

struct Position {
  uint8_t LaneNumber;
  uint8_t EgoInLane;
  uint8_t RoadType;
  uint8_t SpeedLimitMax;
  uint64_t PosTime;
  uint64_t PositionID;
  double StopLineOffset;
  double PosLat;
  double PosLgt;
  double Heading;
  bool is_split{false};
  std::vector<uint8_t> LaneType;  // 数量为LaneNumber
  std::vector<uint32_t> RelateLineID;
};

struct LPoint {
  uint8_t LineType;
  uint8_t Color;
  double X;
  double Y;
  uint64_t PositionID;
};

struct LaneMappingMsg {
  uint32_t RoadId;
  std::vector<Position> TrajHistory;
  std::map<uint32_t, std::vector<LPoint>> LaneData;  // key为LaneId
};

struct MapFusionStatus {
  uint8_t map_saving_status{0};
  uint8_t map_id{0};
  float map_distance{0.0f};
};

struct Point2d {
  double x;
  double y;
};
struct Line {
  uint8_t categroy;  // 该车道线类别（边界线：2, 车道线：1，其他：0）
  uint32_t id;                  // 该车道线id
  std::vector<Point2d> points;  // 该车道线的点的坐标（WGS84）
  std::map<uint32_t, uint32_t>
      traj_line_ids;  // 和轨迹点的绑定关系：key为轨迹点的id，
                      // value为该车道线点id，map的size和points数量一样
};
