//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    void AStarSeed::distanceTransform() {
        cv::Mat binary_img, transformed_img;
        int i, j;

        cv::threshold(fusion_map, binary_img, 100, 255, CV_THRESH_BINARY);

        binary_img = 255 - binary_img;
        cv::distanceTransform(binary_img, transformed_img, CV_DIST_L2, 3);
        float dt_threshold = 100;
        for (i = 0; i < fusion_map.rows; i++)
            for (j = 0; j < fusion_map.cols; j++)
                if (transformed_img.at<float>(i, j) > dt_threshold)
                    transformed_img.at<float>(i, j) = dt_threshold;
        cv::normalize(transformed_img, transformed_img, 0, 1, cv::NORM_MINMAX);
        double min_val, max_val;
        minMaxLoc(transformed_img, &min_val, &max_val);
        binary_img.convertTo(binary_img, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
        transformed_img.convertTo(binary_img, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

        binary_img = 255 - binary_img;
        fusion_map = binary_img;
    }

    void quickReflex::distanceTransform() {
        cv::Mat binary_img, transformed_img;
        int i, j;
        int binary_min_threshold, binary_max_threshold;
        node_handle.getParam("local_planner/max_threshold", binary_max_threshold);
        node_handle.getParam("local_planner/min_threshold", binary_min_threshold);
        cv::threshold(fusion_map, binary_img, 100, 255, CV_THRESH_BINARY);

        binary_img = 255 - binary_img;
        cv::distanceTransform(binary_img, transformed_img, CV_DIST_L2, 3);
        float DtThresh = 100;
        for (i = 0; i < fusion_map.rows; i++)
            for (j = 0; j < fusion_map.cols; j++)
                if (transformed_img.at<float>(i, j) > DtThresh)
                    transformed_img.at<float>(i, j) = DtThresh;
        cv::normalize(transformed_img, transformed_img, 0, 1, cv::NORM_MINMAX);
        double minVal, maxVal;
        minMaxLoc(transformed_img, &minVal, &maxVal);
        binary_img.convertTo(binary_img, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        transformed_img.convertTo(binary_img, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        binary_img = 255 - binary_img;
        fusion_map = binary_img;
    }
}
