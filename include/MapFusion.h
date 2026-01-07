#pragma once

#include <dirent.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include <filesystem>
#include <regex>  // NOLINT
#include <string>

#include "MakeInputData.h"
#include "MakeMapFusion.h"
#include "MakeOutputData.h"
#include "common/framework.h"
#include "calibration.h"
#include "store/yaml.h"
#include "map_fusion_config.h"
#include "common_data_memory.h"

namespace CommuteMode {
namespace MapFusion {
using namespace earth::shell::framework;

class MapFusion {
 public:
  MapFusion();
   ~MapFusion() = default;

  bool Execute(const std::vector<LaneMappingMsg>& lane_mapping_infos, const std::vector<Line>& line_out,
                const bool& is_map_saving, PPOINT2EFM* ppoint2efm, PPOLYGON2EFM* ppolygon2efm,
                CLANE2EFM* clane2efm, MapFusionStatus* map_fusion_status);

  std::shared_ptr<MakeInputData> make_input_data_;
  std::shared_ptr<MakeMapFusion> make_map_fusuion_;
  std::shared_ptr<MakeOutputData> make_output_data_;

  private:  // NOLINT
    bool MapProcess(const std::vector<LaneMappingMsg>& lane_mapping_infos, const std::vector<Line>& line_out);
    uint8_t GetMapId(const std::string& directory);
    bool CheckMapId(MapFusionStatus* map_fusion_status);
    bool InitMapFusion(const std::vector<LaneMappingMsg>& lane_mapping_infos, const bool& is_map_saving);


  private:  // NOLINT
    static MapFusionStaticVar map_fusion_static_var_;
};

}  // namespace framework
}  // namespace shell
