//
// Created by ben on 30/08/2026.
//

#ifndef RRT_LIBRARY_PATH_REFINEMENT_H
#define RRT_LIBRARY_PATH_REFINEMENT_H

#include "RRT.h"

namespace refine {
    //path shortcutting
    [[nodiscard]] inline std::vector<std::vector<double>> greedyShortcutting(const std::vector<std::vector<double>> &path, const System &sys)  {
        if (path.empty()) return {};

        std::vector<std::vector<double>> smoothed_path;
        const PointsToPath* points_to_path = sys.getPointsToPath();
        smoothed_path.push_back(path[0]);
        unsigned int last_step_index = root_index;
        unsigned int nodes_skipped = 0;
        for (unsigned int i = 1; i < path.size(); i++) {
            bool skippable = true;
            for (unsigned int j = 0; j < nodes_skipped; j++) {
                std::vector<double> start = points_to_path->sample(smoothed_path[smoothed_path.size() - 1], path[i], static_cast<double>(j)/nodes_skipped);
                std::vector<double> end = points_to_path->sample(smoothed_path[smoothed_path.size() - 1], path[i], static_cast<double>(j+1)/nodes_skipped);
                if (!sys.validSegment(start, end)) {
                    if (last_step_index == root_index) continue;
                    smoothed_path.push_back(path[last_step_index]);
                    last_step_index = root_index;
                    j = nodes_skipped;
                    nodes_skipped = 0;
                    skippable = false;
                }

            }
            if (skippable) {
                last_step_index = i;
                nodes_skipped++;
            }
        }
        smoothed_path.push_back(path[path.size() - 1]);
        return smoothed_path;
    }
    [[nodiscard]] inline std::vector<std::vector<double>> randomShortcutting(const std::vector<std::vector<double>> &path, const System &sys, const unsigned int iterations)  {
        if (path.empty()) return {};
        std::vector<std::vector<double>> smoothed_path = path;
        const PointsToPath* points_to_path = sys.getPointsToPath();

        for (unsigned int i = 0; i < iterations; i++) {
            unsigned int random_number1 = rand() % smoothed_path.size();
            unsigned int random_number2 = rand() % smoothed_path.size();
            if (random_number1 > random_number2) {
                const unsigned int temp = random_number1;
                random_number1 = random_number2;
                random_number2 = temp;
            }

            if (random_number1 == random_number2) {
                i--;
                continue;
            }
            bool skippable = true;
            for (unsigned int j = 0; j < random_number2 - random_number1; j++) {
                std::vector<double> start = points_to_path->sample(smoothed_path[random_number1], smoothed_path[random_number2], static_cast<double>(j) / (random_number2 - random_number1));
                std::vector<double> end = points_to_path->sample(smoothed_path[random_number1], smoothed_path[random_number2], static_cast<double>(j+1) / (random_number2 - random_number1));
                if (!sys.validSegment(start, end)) {
                    skippable = false;
                    break;
                }
            }
            if (skippable) smoothed_path.erase(smoothed_path.begin() + random_number1 + 1, smoothed_path.begin() + random_number2);
        }

        return smoothed_path;
    }
    [[nodiscard]] inline std::vector<std::vector<double>> stepShortcutting(const std::vector<std::vector<double>> &path, const System &sys) {
        if (path.empty()) return {};
        std::vector<std::vector<double>> smoothed_path = path;
        const PointsToPath* points_to_path = sys.getPointsToPath();

        for (unsigned int step_size = 2; step_size < smoothed_path.size(); step_size *= 2) {
            for (unsigned int start_point = 0; start_point + step_size < smoothed_path.size(); start_point++) {
                bool skippable = true;
                for (unsigned int i = 0; i < step_size; i++) {
                    const std::vector<double> start = points_to_path->sample(path[start_point], path[start_point + step_size], static_cast<double>(i)/(step_size-1));
                    const std::vector<double> end = points_to_path->sample(path[start_point], path[start_point + step_size], static_cast<double>(i+1)/(step_size-1));
                    if (!sys.validSegment(start, end)) {
                        skippable = false;
                        break;
                    }
                }
                if (skippable) {
                    smoothed_path.erase(smoothed_path.begin() + start_point + 1, smoothed_path.begin() + start_point + step_size);
                }
            }
        }
        return smoothed_path;
    }
    [[nodiscard]] inline std::vector<std::vector<double>> maximalSkipShortcutting(const std::vector<std::vector<double>> &path, const System &sys) {
        if (path.empty()) return {};
        std::vector<std::vector<double>> smoothed_path;
        smoothed_path.push_back(path[0]);
        const PointsToPath* points_to_path = sys.getPointsToPath();

        for (unsigned int start = 0; start < path.size(); start++) {
            for (unsigned int end = path.size() - 1; end > start; end--) {
                bool valid_skip = true;
                for (unsigned int i = 0; i < end - start; i++) {
                    std::vector<double> start_point = points_to_path->sample(path[start], path[end], static_cast<double>(i)/(end - start));
                    std::vector<double> end_point = points_to_path->sample(path[start], path[end], static_cast<double>(i+1)/(end - start));
                    if (!sys.validSegment(start_point, end_point)) {
                        valid_skip = false;
                        break;
                    }
                }
                if (valid_skip) {
                    smoothed_path.push_back(path[end]);
                    start = end - 1;
                    break;
                }
            }
        }
        return smoothed_path;
    }
    /*
    [[nodiscard]] inline std::vector<std::vector<double>> exhaustiveShortcutting(const std::vector<std::vector<double>> &path, const System &sys, const DistanceStrategy &distance) {
        std::vector<std::vector<double>> result(path.size());
        const PointsToPath* points_to_path = sys.getPointsToPath();

        std::vector<std::vector<double>> connection_matrix(path.size());
        for (unsigned int i = 0; i < path.size(); i++) {
            connection_matrix[i].resize(path.size());
        }

        for (unsigned int i = 0; i < path.size(); i++) {
            for (unsigned int j = 0; j < path.size(); j++) {
                if (i == j) {
                    connection_matrix[i][j] = -1;
                    continue;
                }
                //if valid
                connection_matrix[i][j] = distance.getDistance(path[i], path[j], sys.getWrapping());
            }
        }

        //use connection matrix to find best path

        return result;
    }
    */
    //path smoothing

    //other
}

#endif //RRT_LIBRARY_PATH_REFINEMENT_H