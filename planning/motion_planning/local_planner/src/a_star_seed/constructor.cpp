//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    std::string AStarSeed::getSeedFileNameAStarSeed(ros::NodeHandle& node_handle) {
        std::string ss;
        node_handle.getParam("local_planner/seed_file2", ss);
        return ss;
    }

    std::string quickReflex::getSeedFileNameQuickReflex(ros::NodeHandle& node_handle) {
        std::string ss;
        node_handle.getParam("local_planner/seed_file8", ss);
        return ss;
    }

    AStarSeed::AStarSeed(ros::NodeHandle& nodeHandle) : node_handle(nodeHandle) {
        std::stringstream ss;
        ss << ros::package::getPath("local_planner");
        ss << getSeedFileNameAStarSeed(node_handle);
        SEEDS_FILE = ss.str();
        loadGivenSeeds();
        node_handle.getParam("local_planner/debug_current_state", debug_current_state);
        node_handle.getParam("local_planner/distance_transform", distance_transform);
    }

    quickReflex::quickReflex(ros::NodeHandle& nodeHandle) : node_handle(nodeHandle) {
        std::stringstream ss;
        ss << ros::package::getPath("local_planner") + getSeedFileNameQuickReflex(node_handle);
        seeds_file = ss.str();
    }
}
