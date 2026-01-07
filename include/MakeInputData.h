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
#include <list>

#include "common_data_memory.h"
#include "common/framework.h"
// #include "json/json.h"
#include "map_fusion_config.h"
#include "store/yaml.h"
#include "calibration.h"
#include "MapCommonTool.h"
#include "data_to_geojson.h"
#include "common_math.h"
#include "common_point.h"
#include "trans_coordinate.h"
// #define PRO_DEAL

namespace CommuteMode {
namespace MapFusion {
// using json = nlohmann::json;


class MakeInputData {
public:
    struct OriPosition{
        double x;
        double y;
        double heading;
    };
    struct BunchLines{
        uint64_t start_position_id;
        uint64_t end_position_id;
        OriPosition ori_point_wgs84;
        std::vector<uint64_t> lint_out_indexes;
    };
    MakeInputData() {};
    ~MakeInputData() = default;
    bool Execute(const std::vector<LaneMappingMsg>& lane_mapping_infos, const std::vector<Line>& line_out,
                const uint8_t& route_id);
    //保存好建图和SD数据，后期判断是否执行
    bool ReadConfig();
    bool SaveData();
    bool LocalDataOK();
    HQRawDataMap* hq_raw_data_map(){return &hq_raw_data_map_;}
    std::vector<LaneMappingMemory> GetInputLaneMappingMemories(){return lane_mapping_memories_;}
    std::vector<PositionInfo> GetInputPositionInfos(){return position_infos_;}
    std::vector<LaneMappingMsg> GetRawLaneMappingMsg(){return lane_mapping_infos_;}
    
private:
    static MakeInputStasticVar make_input_stastic_var_;

    //合并link list
    bool MakeLinkList();

    // 修改建图数据
    // 参数：
    bool ModityOrgData();
    // 修改SD数据
    //参数：
    bool ModitySdData();
    //合并SD和建图数据
    // 参数：
    bool CombineData();
    // 建立索引
    //参数：
    bool MakeIndex(const std::vector<Intersection>& intersection_infos,const std::vector<LinkMemory>& link_data_infos,const std::vector<LaneMappingMsg>& lane_mapping_msg);

    bool ReadPositionCsv(const std::string& file_path, std::vector<PositionInfo>& position_infos);
    bool ReadMapHdnvInfoCsv(const std::string& file_path, std::vector<MapHdnvInfo>& map_hdnv_infos);
    bool ReadIntersectionCsv(const std::string& file_path, std::vector<Intersection>& intersection_infos);
    bool ReadLaneMappingInfoJson(const std::string &file_path, std::vector<LaneMappingMsg>& lane_mapping_infos);
    bool ReadLinkDataCsv(const std::string& file_path, std::vector<LinkDataCenterLine>& link_data_infos);
    
    //输入hq的intersection数据，构造map数据
    bool GenerateIntersectionDataMap(const std::vector<Intersection>& intersection_infos);

    //输入hq的linkdata数据，构map数据
    bool GenerateLinkDataMap(const std::vector<LinkMemory>& link_data_infos);

    //输入local map的LaneMappingMsg数据，构map数据
    bool GenerateLaneMappingMsgDataMap(const std::vector<LaneMappingMsg>& lane_mapping_msg);

    // 输入link memory数据，构map数据
    bool GenerateLinkMemoryDataMap(const std::vector<LinkMemory>& link_memorys);    

    //输入local map的position infos数据，构map数据
    bool GeneratePositionInfosDataMap(const std::vector<PositionInfo> position_infos);

    //对lane进行排序
    bool SortLane(LaneMappingMemory & lane_mapping); 
    
    //选出自车的三车道
    bool PickLane(LaneMappingMemory & lane_mapping, int section_num);   
    
    //对自建图的线平切,输出直角坐标的线，以及转换的原点信息
    bool CutLane(const LaneMappingMemory & lane_mapping , int section_num, 
                 std::vector<std::vector<Point2Dd>>& cut_lane_lines_xy,DoublePosePoint& ori_point,int& base_lane_index);
    bool FixLane(const std::vector<std::vector<Point2Dd>>& cut_lane_lines_xy,const DoublePosePoint& ori_point,int base_lane_index,LaneMappingMemory & lane_mapping);

    bool WideLaneJudge(const std::vector<Line>& line_out,  const std::vector<LaneMappingMemory>& lane_mapping_memories, std::map<uint64_t, std::vector<uint64_t>>& wide_lane_position_id_map);
    int BunchProcessStep1(const std::vector<Line>& line_out, const BunchLines& bunch);
    bool BunchProcessStep2(const std::vector<Line>& line_out, int base_line_index, const BunchLines& bunch, std::vector<uint64_t>& wide_lane_position_id_vec);
    bool GetBunchLines(const std::vector<Line>& line_out,  const std::vector<LaneMappingMemory>& lane_mapping_memories, std::unordered_map<uint32_t,BunchLines>& bunch_map);
    bool OutputWideLane(const std::map<uint64_t, std::vector<uint64_t>>& wide_lane_position_id_map, std::vector<LaneMappingMemory>& lane_mapping_memories);
#ifdef PRO_DEAL
    bool createInput(std::vector<std::vector<double>> lines, uint32_t link_id, LaneMappingMsg& lane_mapping_msg);
    bool createInputPosition(std::vector<std::array<double,3>> pos_vec, std::vector<Position>& TrajHistory);
#endif
    HQRawDataMap hq_raw_data_map_;
    MapCommonTool* map_common_tool_;
    std::vector<LinkMemory> link_memory_{};
    std::vector<Intersection> intersection_infos_{};
    std::vector<PositionInfo> position_infos_{};
    std::vector<LinkMemory> link_data_infos_{};
    std::vector<LinkDataCenterLine> center_line_data_infos_{};
    std::vector<MapHdnvInfo> map_hdnv_infos_{};
    std::vector<LaneMappingMsg> lane_mapping_infos_{};
    std::vector<uint32_t> link_list_{};
    std::vector<LaneMappingMemory> lane_mapping_memories_{};
    std::vector<Line> line_out_{};
    std::unordered_map<uint32_t, DoublePosePoint> traj_points_position_map_{};//key positionID, value-轨迹坐标
};

}  // namespace MapFusion
}  // namespace CommuteMode
