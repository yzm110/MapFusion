#include "map_fusion_config.h"
#include "store/yaml.h"
#include "topic/topic_trait.h"

void MapFusionConfig(const std::string file_name) {
#ifdef __QNX__
    std::string path = "/mnt/appfs/JICA/noa/conf" + file_name;
#else
    std::string path = "/home/ecarx/gitlab/repo84m2/dx11_noa/application/MapFusion/deps" + file_name;
#endif
    earth::mantle::store::yaml::Node config;
    earth::mantle::store::yaml::parse(config, path.c_str());
    try {
        p_memory_route_id = config["p_memory_route_id"].As<int>();
        p_memory_file_platform = config["p_memory_file_platform"].As<int>();
    } catch (const std::exception &e) {
            std::cerr << "YAML Exception: " << e.what() << std::endl;
        }
    }
