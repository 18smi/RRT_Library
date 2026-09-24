//
// Created by ben on 19/06/2026.
//

#ifndef RRT_LIBRARY_OLD_RRT_H
#define RRT_LIBRARY_OLD_RRT_H

#include <SFML/Graphics.hpp>
#include <vector>


struct Point2 {
    Point2(double x, double y) : x(x), y(y) {}
    double x;
    double y;

    bool operator==(std::vector<Point2>::const_reference value) const{
        return x == value.x && y == value.y;
    }
};
struct Point3 {
    Point3(double x, double y, double z) : x(x), y(y), z(z) {}
    double x;
    double y;
    double z;

    bool operator==(std::vector<Point3>::const_reference value) const{
        return x == value.x && y == value.y && z == value.z;
    }
};


struct Obstacle2 {
    [[nodiscard]] virtual bool intersectsSegment(Point2 a, Point2 b) const = 0;
    virtual void draw(sf::RenderWindow& window, sf::Color colour) const = 0;
    virtual ~Obstacle2() = default;
};
struct CircleObstacle final : public Obstacle2 {
    CircleObstacle(const Point2 center, const double radius) : center(center), radius(radius) {}
    Point2 center;
    double radius;

    [[nodiscard]] bool intersectsSegment(Point2 a, Point2 b) const override {
        double deltaX = b.x - a.x;
        double deltaY = b.y - a.y;
        double deltaX_AtC = a.x - center.x;
        double deltaY_AtC = a.y - center.y;

        double Qa = deltaX*deltaX + deltaY*deltaY;
        double Qb = 2.0 * (deltaX_AtC * deltaX + deltaY_AtC * deltaY);
        double Qc = (deltaX_AtC*deltaX_AtC + deltaY_AtC*deltaY_AtC) - (radius * radius);

        double discriminant = Qb * Qb - 4.0 * Qa * Qc;
        if (discriminant < 0) {
            return false;
        }
        if (Qa == 0.0) {
            return (deltaX_AtC*deltaX_AtC + deltaY_AtC*deltaY_AtC) <= radius*radius;
        }

        double answer1 = (-Qb - std::sqrt(discriminant)) / (2.0 * Qa);
        double answer2 = (-Qb + std::sqrt(discriminant)) / (2.0 * Qa);

        //if intersection occurs between (start of line(0) <= formula <= end of line(1))
        return (answer1 >= 0.0 && answer1 <= 1.0) || (answer2 >= 0.0 && answer2 <= 1.0);
    }
    void draw(sf::RenderWindow& window, sf::Color colour) const override {
        sf::CircleShape circle(static_cast<float>(radius));
        circle.setFillColor(colour);
        circle.setOrigin(static_cast<float>(radius), static_cast<float>(radius));
        circle.setPosition(static_cast<float>(center.x), static_cast<float>(center.y));
        window.draw(circle);
    }
};
struct RectangleObstacle final : public Obstacle2 {
    RectangleObstacle(const Point2 center, const double width, const double height, const double rotation) : center(center), width(width), height(height), rotation(rotation) {}
    Point2 center;
    double width;
    double height;
    double rotation;

    [[nodiscard]] bool intersectsSegment(const Point2 a, const Point2 b) const override {
        double distance_to_corner = std::sqrt((width*width + height*height) / 4);
        double angle_TR = atan2(height / 2, width / 2) + (rotation/360 * 2*M_PI);
        double angle_TL = atan2(height / 2, -width / 2) + (rotation/360 * 2*M_PI);
        double angle_BR = atan2(-height / 2, width / 2) + (rotation/360 * 2*M_PI);
        double angle_BL = atan2(-height / 2, -width / 2) + (rotation/360 * 2*M_PI);

        Point2 top_right(center.x + cos(angle_TR)*distance_to_corner, center.y + sin(angle_TR)*distance_to_corner);
        Point2 top_left(center.x + cos(angle_TL)*distance_to_corner, center.y + sin(angle_TL)*distance_to_corner);
        Point2 bottom_right(center.x + cos(angle_BR)*distance_to_corner, center.y + sin(angle_BR)*distance_to_corner);
        Point2 bottom_left(center.x + cos(angle_BL)*distance_to_corner, center.y + sin(angle_BL)*distance_to_corner);



        bool line_crosses_TR_TL = std::signbit((top_left.x - top_right.x)*(a.y - top_right.y) - (top_left.y - top_right.y)*(a.x - top_right.x)) != std::signbit((top_left.x - top_right.x)*(b.y - top_right.y) - (top_left.y - top_right.y)*(b.x - top_right.x));
        bool TR_TL_crosses_line = std::signbit((b.x - a.x)*(top_left.y - a.y) - (b.y - a.y)*(top_left.x - a.x)) != std::signbit((b.x - a.x)*(top_right.y - a.y) - (b.y - a.y)*(top_right.x - a.x));
        bool line_crosses_TR_BR = std::signbit((bottom_right.x - top_right.x)*(a.y - top_right.y) - (bottom_right.y - top_right.y)*(a.x - top_right.x)) != std::signbit((bottom_right.x - top_right.x)*(b.y - top_right.y) - (bottom_right.y - top_right.y)*(b.x - top_right.x));
        bool TR_BR_crosses_line = std::signbit((b.x - a.x)*(bottom_right.y - a.y) - (b.y - a.y)*(bottom_right.x - a.x)) != std::signbit((b.x - a.x)*(top_right.y - a.y) - (b.y - a.y)*(top_right.x - a.x));
        bool line_crosses_TL_BL = std::signbit((bottom_left.x - top_left.x)*(a.y - top_left.y) - (bottom_left.y - top_left.y)*(a.x - top_left.x)) != std::signbit((bottom_left.x - top_left.x)*(b.y - top_left.y) - (bottom_left.y - top_left.y)*(b.x - top_left.x));
        bool TL_BL_crosses_line = std::signbit((b.x - a.x)*(bottom_left.y - a.y) - (b.y - a.y)*(bottom_left.x - a.x)) != std::signbit((b.x - a.x)*(top_left.y - a.y) - (b.y - a.y)*(top_left.x - a.x));
        bool line_crosses_BR_BL = std::signbit((bottom_left.x - bottom_right.x)*(a.y - bottom_right.y) - (bottom_left.y - bottom_right.y)*(a.x - bottom_right.x)) != std::signbit((bottom_left.x - bottom_right.x)*(b.y - bottom_right.y) - (bottom_left.y - bottom_right.y)*(b.x - bottom_right.x));
        bool BR_BL_crosses_line = std::signbit((b.x - a.x)*(bottom_left.y - a.y) - (b.y - a.y)*(bottom_left.x - a.x)) != std::signbit((b.x - a.x)*(bottom_right.y - a.y) - (b.y - a.y)*(bottom_right.x - a.x));

        if (line_crosses_TR_TL && TR_TL_crosses_line) {
            return true;
        }
        if (line_crosses_TR_BR && TR_BR_crosses_line) {
            return true;
        }
        if (line_crosses_TL_BL && TL_BL_crosses_line) {
            return true;
        }
        if (line_crosses_BR_BL && BR_BL_crosses_line) {
            return true;
        }

        return false;
    }
    void draw(sf::RenderWindow& window, const sf::Color colour) const override {
        sf::RectangleShape rect(sf::Vector2f(static_cast<float>(width), static_cast<float>(height)));
        rect.setOrigin(static_cast<float>(width / 2.0), static_cast<float>(height / 2.0));
        rect.setPosition(static_cast<float>(center.x), static_cast<float>(center.y));
        rect.setRotation(static_cast<float>(rotation));
        rect.setFillColor(colour);
        window.draw(rect);
    }
};


class RRT2BaseSingle {
public:
    virtual ~RRT2BaseSingle() = default;

    [[nodiscard]] virtual bool pathFound() const = 0;
    virtual std::vector<Point2>& getFoundPath() = 0;
    virtual void generateNewDot(int border_x, int border_y) = 0;

    virtual const std::vector<Point2>& getDots() = 0;
    virtual const std::vector<int>& getParentIndex() = 0;
    [[nodiscard]] virtual const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() const = 0;

    [[nodiscard]] virtual Point2 getEndPoint() const = 0;
    [[nodiscard]] virtual double getEndBuffer() const = 0;
    virtual void reset() = 0;
};
class RRT2BaseDouble {
public:
    virtual ~RRT2BaseDouble() = default;

    [[nodiscard]] virtual bool pathFound() const = 0;
    virtual std::vector<Point2>& getFoundPath() = 0;
    virtual void generateNewDot(int border_x, int border_y) = 0;

    virtual std::vector<Point2>& getDotsFromStart() = 0;
    virtual std::vector<Point2>& getDotsFromEnd() = 0;
    virtual std::vector<int>& getParentIndexFromStart() = 0;
    virtual std::vector<int>& getParentIndexFromEnd() = 0;
    virtual const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() = 0;
    [[nodiscard]] virtual Point2 getStartConnector() const = 0;
    [[nodiscard]] virtual Point2 getEndConnector() const = 0;
    virtual void reset() = 0;
};
class RRT2BaseGraph {

};
class RRT2BaseKinodynamic {

};


// I should move the random point generator into its own function
class RRT2 final : public RRT2BaseSingle {
public:
    RRT2(const Point2 start, const Point2 end, const double end_buffer, const int max_length) : end(end), end_buffer(end_buffer), max_length(max_length) {
        dots.push_back(start);
        parent_index.push_back(-1);
    }


    [[nodiscard]] bool pathFound() const override {
        return end_found;
    }

    void generateNewDot(const int border_x, const int border_y) override {
        if (end_found) {
            return;
        }
        for (int i = 0; i < max_attempts_for_adding_a_new_dot; i++) {
            const Point2 new_point(rand() % border_x, rand() % border_y);

            const int closest_point_index = findClosestPointIndex(new_point);
            const Point2 adjusted_point = limitByDistance(new_point, dots[closest_point_index]);
            if (!validConnection(adjusted_point, dots[closest_point_index])) {
                continue;
            }
            dots.push_back(adjusted_point);
            parent_index.push_back(closest_point_index);

            if (std::sqrt((adjusted_point.x - end.x)*(adjusted_point.x - end.x) + (adjusted_point.y - end.y)*(adjusted_point.y - end.y)) < end_buffer) {
                end_found = true;
            }
            return;
        }

    }

    void addCircleObstacle(Point2 center, double radius) {
        obstacles.push_back(std::make_unique<CircleObstacle>(center, radius));
    }
    void addRectangleObstruction(Point2 center, double width, double height, double rotation) {
        obstacles.push_back(std::make_unique<RectangleObstacle>(center, width, height, rotation));
    }


    [[nodiscard]] std::vector<Point2>& getFoundPath() override {
        if (!end_found) {
            return found_path;
        }

        std::vector<int> reverse_path_index;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[i].x-end.x)*(dots[i].x-end.x) + (dots[i].y-end.y)*(dots[i].y-end.y)) <= end_buffer) {
                reverse_path_index.push_back(i);
                break;
            }
        }
        int previous_index = reverse_path_index[0];
        while (parent_index[previous_index] != -1) {
            previous_index = parent_index[previous_index];
            reverse_path_index.push_back(previous_index);
        }

        found_path.clear();
        for (int i = static_cast<int>(reverse_path_index.size())-1; i >= 0; i--) {
            found_path.push_back(dots[reverse_path_index[i]]);
        }
        return found_path;
    }

    [[nodiscard]] const std::vector<Point2>& getDots() override {
        return dots;
    }
    [[nodiscard]] const std::vector<int>& getParentIndex() override {
        return parent_index;
    }
    [[nodiscard]] double getEndBuffer() const override {
        return end_buffer;
    }
    [[nodiscard]] Point2 getEndPoint() const override {
        return end;
    }
    [[nodiscard]] const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() const override {
        return obstacles;
    }

    void setMaxAttemptsForAddingDots(const int max_attempts_for_adding_dots) {
        max_attempts_for_adding_a_new_dot = max_attempts_for_adding_dots;
    }

    void reset() override {
        const Point2 start = dots[0];
        dots.clear();
        parent_index.clear();
        dots.push_back(start);
        parent_index.push_back(-1);
        end_found = false;
        found_path.clear();
    }

private:
    std::vector<Point2> dots;
    std::vector<int> parent_index;
    std::vector<std::unique_ptr<Obstacle2>> obstacles;
    Point2 end;
    double end_buffer;
    int max_length;
    bool end_found = false;
    int max_attempts_for_adding_a_new_dot = 100;
    std::vector<Point2> found_path;


    [[nodiscard]] int findClosestPointIndex(const Point2 point) const {
        int closest_point_index = 0;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[closest_point_index].x-point.x)*(dots[closest_point_index].x-point.x) + (dots[closest_point_index].y-point.y)*(dots[closest_point_index].y-point.y)) > std::sqrt((dots[i].x-point.x)*(dots[i].x-point.x) + (dots[i].y-point.y)*(dots[i].y-point.y))) {
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    [[nodiscard]] Point2 limitByDistance(const Point2 new_point, const Point2 old_point) const {
        if (std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y)) > max_length) {
            const double distance_scalar = max_length / std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y));
            return {old_point.x + (new_point.x - old_point.x) * distance_scalar, old_point.y + (new_point.y - old_point.y) * distance_scalar};
        }
        return new_point;
    }

    [[nodiscard]] bool validConnection(const Point2 new_point, const Point2 old_point) const {
        for (const auto & obstacle : obstacles) {
            if (obstacle->intersectsSegment(new_point, old_point)) {
                return false;
            }
        }
        return true;
    }
};
class RRT2STAR final : public RRT2BaseSingle {
public:
    RRT2STAR(const Point2 start, const Point2 end, const double end_buffer, const int max_length) : end(end), end_buffer(end_buffer), max_length(max_length) {
        dots.push_back(start);
        parent_index.push_back(-1);
        costs.push_back(0);
    }


    [[nodiscard]] bool pathFound() const override {
        return end_found;
    }


    void generateNewDot(const int border_x, const int border_y) override {
        for (int i = 0; i < max_attempts_for_adding_a_new_dot; i++) {
            const Point2 new_point(rand() % border_x, rand() % border_y);

            const int closest_point_index = findClosestPointIndex(new_point);
            const Point2 adjusted_point = limitByDistance(new_point, dots[closest_point_index]);
            if (!validConnection(adjusted_point, dots[closest_point_index])) {
                continue;
            }

            const std::vector<int> neighbours_index = findNeighboursIndex(adjusted_point, currentAdjustmentRadius());
            const int bestParentIndex = findBestNeighbour(adjusted_point, closest_point_index, neighbours_index);

            dots.push_back(adjusted_point);
            parent_index.push_back(bestParentIndex);
            costs.push_back(costCalculator(adjusted_point, bestParentIndex));

            rewireNeighbours(static_cast<int>(dots.size())-1, neighbours_index);

            if (std::sqrt((adjusted_point.x - end.x)*(adjusted_point.x - end.x) + (adjusted_point.y - end.y)*(adjusted_point.y - end.y)) < end_buffer) {
                end_found = true;
            }
            return;
        }
    }


    void addCircleObstacle(Point2 center, double radius) {
        obstacles.push_back(std::make_unique<CircleObstacle>(center, radius));
    }
    void addRectangleObstruction(Point2 center, double width, double height, double rotation) {
        obstacles.push_back(std::make_unique<RectangleObstacle>(center, width, height, rotation));
    }


    [[nodiscard]] std::vector<Point2>& getFoundPath() override {
        if (!end_found) {
            return found_path;
        }

        std::vector<int> reverse_path_index;
        int best_end_index = -1;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[i].x-end.x)*(dots[i].x-end.x) + (dots[i].y-end.y)*(dots[i].y-end.y)) <= end_buffer) {
                if (best_end_index == -1) {
                    best_end_index = i;
                    continue;
                }
                if (costs[i] < costs[best_end_index]) {
                    best_end_index = i;
                }
            }
        }
        reverse_path_index.push_back(best_end_index);
        int previous_index = reverse_path_index[0];
        while (parent_index[previous_index] != -1) {
            previous_index = parent_index[previous_index];
            reverse_path_index.push_back(previous_index);
        }

        found_path.clear();
        for (int i = static_cast<int>(reverse_path_index.size())-1; i >= 0; i--) {
            found_path.push_back(dots[reverse_path_index[i]]);
        }
        return found_path;
    }

    [[nodiscard]] const std::vector<Point2>& getDots() override {
        return dots;
    }
    [[nodiscard]] const std::vector<int>& getParentIndex() override {
        return parent_index;
    }
    [[nodiscard]] double getEndBuffer() const override {
        return end_buffer;
    }
    [[nodiscard]] Point2 getEndPoint() const override {
        return end;
    }
    [[nodiscard]] const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() const override {
        return obstacles;
    }

    void setMaxAttemptsForAddingDots(const int max_attempts_for_adding_dots) {
        max_attempts_for_adding_a_new_dot = max_attempts_for_adding_dots;
    }
    void setModeSlowOptimize() {
        shrinking_radius = true;
    }
    void setModeFastStable() {
        shrinking_radius = false;
    }
    void setFixedRadius(const double new_fixed_radius) {
        fixed_radius = new_fixed_radius;
    }
    void setGamma(const double new_gamma) {
        gamma = new_gamma;
    }
    void setMaxRadius(const double new_max_radius) {
        max_radius = new_max_radius;
    }

    void reset() override {
        Point2 start = dots[0];
        dots.clear();
        parent_index.clear();
        costs.clear();
        dots.push_back(start);
        parent_index.push_back(-1);
        costs.push_back(0);
        end_found = false;
        found_path.clear();
    }


private:
    std::vector<Point2> dots;
    std::vector<int> parent_index;
    std::vector<double> costs;
    std::vector<std::unique_ptr<Obstacle2>> obstacles;
    Point2 end;
    double end_buffer;
    int max_length;
    bool end_found = false;
    int max_attempts_for_adding_a_new_dot = 100;
    bool shrinking_radius = false;
    double fixed_radius = 75.0;
    double gamma = 225.0;
    double max_radius = 120.0;
    std::vector<Point2> found_path;


    [[nodiscard]] int findClosestPointIndex(const Point2 point) const {
        int closest_point_index = 0;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[closest_point_index].x-point.x)*(dots[closest_point_index].x-point.x) + (dots[closest_point_index].y-point.y)*(dots[closest_point_index].y-point.y)) > std::sqrt((dots[i].x-point.x)*(dots[i].x-point.x) + (dots[i].y-point.y)*(dots[i].y-point.y))) {
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    [[nodiscard]] Point2 limitByDistance(const Point2 new_point, const Point2 old_point) const {
        if (std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y)) > max_length) {
            const double distance_scalar = max_length / std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y));
            return {old_point.x + (new_point.x - old_point.x) * distance_scalar, old_point.y + (new_point.y - old_point.y) * distance_scalar};
        }
        return new_point;
    }

    [[nodiscard]] bool validConnection(const Point2 new_point, const Point2 old_point) const {
        for (const auto & obstacle : obstacles) {
            if (obstacle->intersectsSegment(new_point, old_point)) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] double currentAdjustmentRadius() const {
        if (!shrinking_radius) {
            return fixed_radius;
        }
        const int n = static_cast<int>(dots.size());
        const double radius = gamma * std::sqrt(std::log(n) / n);
        return std::min(radius, max_radius);
    }

    [[nodiscard]] std::vector<int> findNeighboursIndex(const Point2 point, const double radius) const {
        std::vector<int> neighbours;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[i].x - point.x)*(dots[i].x - point.x) + (dots[i].y - point.y)*(dots[i].y - point.y)) < radius) {
                neighbours.push_back(i);
            }
        }
        return neighbours;
    }

    [[nodiscard]] double costCalculator(const Point2 point, const int posable_parent_index) const {
        return costs[posable_parent_index] + std::sqrt((dots[posable_parent_index].x - point.x)*(dots[posable_parent_index].x - point.x) + (dots[posable_parent_index].y - point.y)*(dots[posable_parent_index].y - point.y));
    }

    [[nodiscard]] int findBestNeighbour(const Point2 point, const int closest_point_index, const std::vector<int>& neighbours_index) const {
        int best_candidate_index = closest_point_index;
        for (const int j : neighbours_index) {
            if (validConnection(point, dots[j])) {
                if (costCalculator(point, j) < costCalculator(point, best_candidate_index)) {
                    best_candidate_index = j;
                }
            }
        }
        return best_candidate_index;
    }

    void rewireNeighbours(const int point_index, const std::vector<int>& neighbours_index) {
        for (const int i : neighbours_index) {
            if (i == parent_index[point_index]) {
                continue;
            }
            if (!validConnection(dots[point_index], dots[i])) {
                continue;
            }
            double new_cost = costCalculator(dots[i], point_index);
            if (new_cost < costs[i]) {
                parent_index[i] = point_index;
                costs[i] = new_cost;
                updateDescendantsCost(i);
            }
        }
    }

    void updateDescendantsCost(const int index) {
        for (int i = 0; i < dots.size(); i++) {
            if (parent_index[i] == index) {
                costs[i] = costCalculator(dots[i], index);
                updateDescendantsCost(i);
            }
        }
    }
};
class RRT2CONNECT final : public RRT2BaseDouble {
    public:
    RRT2CONNECT(const Point2 start, const Point2 end, const double max_length) : max_length(max_length) {
        dots_from_start.push_back(start);
        dots_from_end.push_back(end);
        parent_index_from_start.push_back(-1);
        parent_index_from_end.push_back(-1);
    }


    [[nodiscard]] bool pathFound() const override {
        return !found_path.empty();
    }

    void generateNewDot(const int border_x, const int border_y) override {
        if (!found_path.empty()) {
            return;
        }
        for (int i = 0; i < max_attempts_for_adding_a_new_dot; i++) {
            const Point2 new_point(rand() % border_x, rand() % border_y);

            const int closest_point_index = bestParentIndex(new_point);
            const Point2 adjusted_point = limitByDistance(new_point, start_tree_is_active? dots_from_start[closest_point_index] : dots_from_end[closest_point_index]);
            if (!validConnection(adjusted_point, start_tree_is_active? dots_from_start[closest_point_index] : dots_from_end[closest_point_index])) {
                continue;
            }
            start_tree_is_active? dots_from_start.push_back(adjusted_point) : dots_from_end.push_back(adjusted_point);
            start_tree_is_active? parent_index_from_start.push_back(closest_point_index) : parent_index_from_end.push_back(closest_point_index);

            start_tree_is_active = !start_tree_is_active;

            //extend non-active towards the newly added node
            const Point2 target_point = adjusted_point;
            while (true) {
                const int closest_target_point_index = bestParentIndex(target_point);
                const Point2 adjusted_target_point = limitByDistance(target_point, start_tree_is_active? dots_from_start[closest_target_point_index] : dots_from_end[closest_target_point_index]);
                if (!validConnection(adjusted_target_point, start_tree_is_active? dots_from_start[closest_target_point_index] : dots_from_end[closest_target_point_index])) {
                    break;
                }
                if (adjusted_target_point == target_point) {
                    start_connector = dots_from_start[dots_from_start.size() - 1];
                    end_connector = dots_from_end[dots_from_end.size() - 1];
                    createPath();
                    return;
                }

                start_tree_is_active? dots_from_start.push_back(adjusted_target_point) : dots_from_end.push_back(adjusted_target_point);
                start_tree_is_active? parent_index_from_start.push_back(closest_target_point_index) : parent_index_from_end.push_back(closest_target_point_index);
            }
            return;
        }
    }

    void addCircleObstacle(Point2 center, double radius) {
        obstacles.push_back(std::make_unique<CircleObstacle>(center, radius));
    }
    void addRectangleObstruction(Point2 center, double width, double height, double rotation) {
        obstacles.push_back(std::make_unique<RectangleObstacle>(center, width, height, rotation));
    }
    void setMaxAttemptsForAddingDots(const int attempts) {
        max_attempts_for_adding_a_new_dot = attempts;
    }

    [[nodiscard]] std::vector<Point2>& getFoundPath() override {
        return found_path;
    }

    [[nodiscard]] std::vector<Point2>& getDotsFromStart() override {
        return dots_from_start;
    }
    [[nodiscard]] std::vector<Point2>& getDotsFromEnd() override {
        return dots_from_end;
    }
    [[nodiscard]] std::vector<int>& getParentIndexFromStart() override {
        return parent_index_from_start;
    }
    [[nodiscard]] std::vector<int>& getParentIndexFromEnd() override {
        return parent_index_from_end;
    }
    [[nodiscard]] const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() override {
        return obstacles;
    }
    [[nodiscard]] Point2 getStartConnector() const override {
        return start_connector;
    }
    [[nodiscard]] Point2 getEndConnector() const override {
        return end_connector;
    }

    void reset() override {
        start_tree_is_active = true;
        const Point2 start = dots_from_start[0];
        const Point2 end = dots_from_end[0];
        dots_from_start.clear();
        dots_from_end.clear();
        parent_index_from_start.clear();
        parent_index_from_end.clear();
        start_connector = {0, 0};
        end_connector = {0, 0};
        found_path.clear();
        dots_from_start.push_back(start);
        dots_from_end.push_back(end);
        parent_index_from_start.push_back(-1);
        parent_index_from_end.push_back(-1);
    }

private:
    double max_length;
    std::vector<Point2> dots_from_start;
    std::vector<Point2> dots_from_end;
    std::vector<int> parent_index_from_start;
    std::vector<int> parent_index_from_end;
    std::vector<std::unique_ptr<Obstacle2>> obstacles;
    int max_attempts_for_adding_a_new_dot = 100;
    std::vector<Point2> found_path;
    bool start_tree_is_active = true;
    Point2 start_connector = {0, 0};
    Point2 end_connector = {0, 0};



    [[nodiscard]] int bestParentIndex(const Point2 point) const {
        int closest_point_index = 0;
        const std::vector<Point2> dots = start_tree_is_active? dots_from_start : dots_from_end;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[closest_point_index].x-point.x)*(dots[closest_point_index].x-point.x) + (dots[closest_point_index].y-point.y)*(dots[closest_point_index].y-point.y)) > std::sqrt((dots[i].x-point.x)*(dots[i].x-point.x) + (dots[i].y-point.y)*(dots[i].y-point.y))) {
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    [[nodiscard]] Point2 limitByDistance(const Point2 new_point, const Point2 old_point) const {
        if (std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y)) > max_length) {
            const double distance_scalar = max_length / std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y));
            return {old_point.x + (new_point.x - old_point.x) * distance_scalar, old_point.y + (new_point.y - old_point.y) * distance_scalar};
        }
        return new_point;
    }

    [[nodiscard]] bool validConnection(const Point2 new_point, const Point2 old_point) const {
        for (const auto & obstacle : obstacles) {
            if (obstacle->intersectsSegment(new_point, old_point)) {
                return false;
            }
        }
        return true;
    }

    void createPath() {
        std::vector<int> reverse_path_index;
        reverse_path_index.push_back(static_cast<int>(dots_from_start.size()) - 1);

        int previous_index = reverse_path_index[0];
        while (parent_index_from_start[previous_index] != -1) {
            previous_index = parent_index_from_start[previous_index];
            reverse_path_index.push_back(previous_index);
        }

        found_path.clear();
        for (int i = static_cast<int>(reverse_path_index.size())-1; i >= 0; i--) {
            found_path.push_back(dots_from_start[reverse_path_index[i]]);
        }

        previous_index = static_cast<int>(dots_from_end.size()) - 1;
        found_path.push_back(dots_from_end[previous_index]);
        while (parent_index_from_end[previous_index] != -1) {
            previous_index = parent_index_from_end[previous_index];
            found_path.push_back(dots_from_end[previous_index]);
        }
    }
};
class BI_RRT2 final : public RRT2BaseDouble {
    public:
    BI_RRT2(const Point2 start, const Point2 end, const double max_length) : max_length(max_length) {
        dots_from_start.push_back(start);
        dots_from_end.push_back(end);
        parent_index_from_start.push_back(-1);
        parent_index_from_end.push_back(-1);
    }


    [[nodiscard]] bool pathFound() const override {
        return !found_path.empty();
    }

    void generateNewDot(const int border_x, const int border_y) override {
        if (!found_path.empty()) {
            return;
        }
        for (int i = 0; i < max_attempts_for_adding_a_new_dot; i++) {
            const Point2 new_point(rand() % border_x, rand() % border_y);

            const int closest_point_index = bestParentIndex(new_point);
            const Point2 adjusted_point = limitByDistance(new_point, start_tree_is_active? dots_from_start[closest_point_index] : dots_from_end[closest_point_index]);

            if (!validConnection(adjusted_point, start_tree_is_active? dots_from_start[closest_point_index] : dots_from_end[closest_point_index])) {
                continue;
            }
            start_tree_is_active? dots_from_start.push_back(adjusted_point) : dots_from_end.push_back(adjusted_point);
            start_tree_is_active? parent_index_from_start.push_back(closest_point_index) : parent_index_from_end.push_back(closest_point_index);

            tryLinkingTrees();

            start_tree_is_active = !start_tree_is_active;
            return;
        }
    }

    void addCircleObstacle(Point2 center, double radius) {
        obstacles.push_back(std::make_unique<CircleObstacle>(center, radius));
    }
    void addRectangleObstruction(Point2 center, double width, double height, double rotation) {
        obstacles.push_back(std::make_unique<RectangleObstacle>(center, width, height, rotation));
    }
    void setMaxAttemptsForAddingDots(const int attempts) {
        max_attempts_for_adding_a_new_dot = attempts;
    }

    [[nodiscard]] std::vector<Point2>& getFoundPath() override {
        return found_path;
    }

    [[nodiscard]] std::vector<Point2>& getDotsFromStart() override {
        return dots_from_start;
    }
    [[nodiscard]] std::vector<Point2>& getDotsFromEnd() override {
        return dots_from_end;
    }
    [[nodiscard]] std::vector<int>& getParentIndexFromStart() override {
        return parent_index_from_start;
    }
    [[nodiscard]] std::vector<int>& getParentIndexFromEnd() override {
        return parent_index_from_end;
    }
    [[nodiscard]] const std::vector<std::unique_ptr<Obstacle2>>& getObstacles() override {
        return obstacles;
    }
    [[nodiscard]] Point2 getStartConnector() const override {
        return start_connector;
    }
    [[nodiscard]] Point2 getEndConnector() const override {
        return end_connector;
    }

    void reset() override {
        const Point2 start = dots_from_start[0];
        const Point2 end = dots_from_end[0];
        dots_from_start.clear();
        dots_from_end.clear();
        parent_index_from_start.clear();
        parent_index_from_end.clear();
        found_path.clear();
        start_tree_is_active = true;
        start_connector = {0, 0};
        end_connector = {0, 0};
        dots_from_start.push_back(start);
        dots_from_end.push_back(end);
        parent_index_from_start.push_back(-1);
        parent_index_from_end.push_back(-1);
    }

private:
    double max_length;
    std::vector<Point2> dots_from_start;
    std::vector<Point2> dots_from_end;
    std::vector<int> parent_index_from_start;
    std::vector<int> parent_index_from_end;
    std::vector<std::unique_ptr<Obstacle2>> obstacles;
    int max_attempts_for_adding_a_new_dot = 100;
    std::vector<Point2> found_path;
    bool start_tree_is_active = true;
    Point2 start_connector = {0, 0};
    Point2 end_connector = {0, 0};



    [[nodiscard]] int bestParentIndex(const Point2 point) const {
        int closest_point_index = 0;
        const std::vector<Point2> dots = start_tree_is_active? dots_from_start : dots_from_end;
        for (int i = 0; i < dots.size(); i++) {
            if (std::sqrt((dots[closest_point_index].x-point.x)*(dots[closest_point_index].x-point.x) + (dots[closest_point_index].y-point.y)*(dots[closest_point_index].y-point.y)) > std::sqrt((dots[i].x-point.x)*(dots[i].x-point.x) + (dots[i].y-point.y)*(dots[i].y-point.y))) {
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    [[nodiscard]] Point2 limitByDistance(const Point2 new_point, const Point2 old_point) const {
        if (std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y)) > max_length) {
            const double distance_scalar = max_length / std::sqrt((new_point.x - old_point.x)*(new_point.x - old_point.x) + (new_point.y - old_point.y)*(new_point.y - old_point.y));
            return {old_point.x + (new_point.x - old_point.x) * distance_scalar, old_point.y + (new_point.y - old_point.y) * distance_scalar};
        }
        return new_point;
    }

    [[nodiscard]] bool validConnection(const Point2 new_point, const Point2 old_point) const {
        for (const auto & obstacle : obstacles) {
            if (obstacle->intersectsSegment(new_point, old_point)) {
                return false;
            }
        }
        return true;
    }

    void createPath(const int start_connection_index, const int end_connection_index) {
        std::vector<int> reverse_path_index;
        reverse_path_index.push_back(start_connection_index);

        int previous_index = start_connection_index;
        while (parent_index_from_start[previous_index] != -1) {
            previous_index = parent_index_from_start[previous_index];
            reverse_path_index.push_back(previous_index);
        }

        found_path.clear();
        for (int i = static_cast<int>(reverse_path_index.size())-1; i >= 0; i--) {
            found_path.push_back(dots_from_start[reverse_path_index[i]]);
        }

        previous_index = end_connection_index;
        found_path.push_back(dots_from_end[previous_index]);
        while (parent_index_from_end[previous_index] != -1) {
            previous_index = parent_index_from_end[previous_index];
            found_path.push_back(dots_from_end[previous_index]);
        }

        start_connector = dots_from_start[start_connection_index];
        end_connector = dots_from_end[end_connection_index];
    }

    void tryLinkingTrees() {//link point with !start_tree_is_active
        const Point2 point = start_tree_is_active? dots_from_start[dots_from_start.size()-1] : dots_from_end[dots_from_end.size()-1];
        const std::vector<Point2> connection_tree = !start_tree_is_active ? dots_from_start : dots_from_end;

        for (int i = 0; i < connection_tree.size(); i++) {
            if (!validConnection(point, connection_tree[i])) {
                continue;
            }
            if (std::sqrt((point.x - connection_tree[i].x)*(point.x - connection_tree[i].x) + (point.y - connection_tree[i].y)*(point.y - connection_tree[i].y)) > max_length) {
                continue;
            }
            createPath(start_tree_is_active? static_cast<int>(dots_from_start.size())-1 : i, start_tree_is_active? i : static_cast<int>(dots_from_end.size())-1);
            return;
        }
    }
};



class RRT2_visualizer {
    public:
    RRT2_visualizer(RRT2BaseSingle &RRTree, sf::RenderWindow &window) : RRTreeS(&RRTree), window(window) {}
    RRT2_visualizer(RRT2BaseDouble &RRTree, sf::RenderWindow &window) : RRTreeD(&RRTree), window(window) {}
    RRT2_visualizer(RRT2BaseGraph &RRTree, sf::RenderWindow &window) : RRTreeG(&RRTree), window(window) {}
    RRT2_visualizer(RRT2BaseKinodynamic &RRTree, sf::RenderWindow &window) : RRTreeK(&RRTree), window(window) {}



    void setDotSize(const float new_dot_size) {
        this->dot_size = new_dot_size;
    }
    void setPathDotSize(const float new_path_dot_size) {
        path_size = new_path_dot_size;
    }
    void setStartDotColour(const sf::Color colour) {
        start_colour = colour;
    }
    void setEndBufferColourUnreached(const sf::Color colour) {
        end_buffer_colour_unreached = colour;
    }
    void setEndBufferColourReached(const sf::Color colour) {
        end_buffer_colour_reached = colour;
    }
    void setDotColour(const sf::Color colour) {
        dot_colour = colour;
    }
    void setObstacleColour(const sf::Color colour) {
        obstacle_colour = colour;
    }
    void setFoundPathColour(const sf::Color colour) {
        found_path_colour = colour;
    }
    void setEndPathColour(const sf::Color colour) {
        end_buffer_colour_unreached = colour;
    }


    void RRT2Display() const {
        if (RRTreeS) {
            std::vector<Point2> dots = RRTreeS->getDots();
            std::vector<int> parent_index = RRTreeS->getParentIndex();
            auto end_buffer = static_cast<float>(RRTreeS->getEndBuffer());
            bool path_found = RRTreeS->pathFound();
            float end_point_x = static_cast<float>(RRTreeS->getEndPoint().x);
            float end_point_y = static_cast<float>(RRTreeS->getEndPoint().y);
            std::vector<Point2> found_path = RRTreeS->getFoundPath();

            window.clear();

            sf::CircleShape end_zone(end_buffer == -1? dot_size : end_buffer);
            end_zone.setFillColor(path_found? end_buffer_colour_reached: end_buffer_colour_unreached);
            end_zone.setOrigin(end_zone.getRadius(), end_zone.getRadius());
            end_zone.setPosition(end_point_x, end_point_y);
            window.draw(end_zone);

            // could save computation by making obstacles variable
            for (const auto & i : RRTreeS->getObstacles()) {
                i->draw(window, obstacle_colour);
            }

            for (int i = 0; i < dots.size(); i++) {
                sf::CircleShape circle;
                if (path_found) {

                    bool isPathDot = false;
                    for (auto j : found_path) {
                        if (dots[i] == j) {
                            isPathDot = true;
                            break;
                        }
                    }
                    circle.setRadius(isPathDot? path_size : dot_size);
                    circle.setFillColor(isPathDot? found_path_colour : dot_colour);
                }
                else {
                    circle.setRadius(dot_size);
                    circle.setFillColor(i? dot_colour : start_colour);
                }
                circle.setOrigin(circle.getRadius(), circle.getRadius());
                circle.setPosition(static_cast<float>(dots[i].x), static_cast<float>(dots[i].y));
                window.draw(circle);
                if (i) {
                    sf::Vertex line[2] = {sf::Vector2f(static_cast<float>(dots[i].x), static_cast<float>(dots[i].y)), sf::Vector2f(static_cast<float>(dots[parent_index[i]].x), static_cast<float>(dots[parent_index[i]].y))};
                    window.draw(line, 2, sf::Lines);
                }
            }


            window.display();
            return;
        }
        if (RRTreeD) {
            std::vector<Point2> dots = RRTreeD->getDotsFromStart();
            std::vector<int> parent_index = RRTreeD->getParentIndexFromStart();
            bool path_found = RRTreeD->pathFound();
            std::vector<Point2> found_path = RRTreeD->getFoundPath();
            window.clear();

            //could save computation by making an obstacles variable
            for (const auto & i : RRTreeD->getObstacles()) {
                i->draw(window, obstacle_colour);
            }

            for (int i = 0; i < dots.size(); i++) {
                sf::CircleShape circle;
                if (path_found) {

                    bool isPathDot = false;
                    for (auto j : found_path) {
                        if (dots[i] == j) {
                            isPathDot = true;
                            break;
                        }
                    }
                    circle.setRadius(isPathDot? path_size : dot_size);
                    circle.setFillColor(isPathDot? found_path_colour : dot_colour);
                }
                else {
                    circle.setRadius(dot_size);
                    circle.setFillColor(i? dot_colour : start_colour);
                }
                circle.setOrigin(circle.getRadius(), circle.getRadius());
                circle.setPosition(static_cast<float>(dots[i].x), static_cast<float>(dots[i].y));
                window.draw(circle);
                if (i) {
                    sf::Vertex line[2] = {sf::Vector2f(static_cast<float>(dots[i].x), static_cast<float>(dots[i].y)), sf::Vector2f(static_cast<float>(dots[parent_index[i]].x), static_cast<float>(dots[parent_index[i]].y))};
                    window.draw(line, 2, sf::Lines);
                }
            }

            if (path_found) {
                sf::Vertex line[2] = {sf::Vector2f(static_cast<float>(RRTreeD->getStartConnector().x), static_cast<float>(RRTreeD->getStartConnector().y)), sf::Vector2f(static_cast<float>(RRTreeD->getEndConnector().x), static_cast<float>(RRTreeD->getEndConnector().y))};
                window.draw(line, 2, sf::Lines);
            }

            std::vector<Point2> end_dots = RRTreeD->getDotsFromEnd();
            parent_index = RRTreeD->getParentIndexFromEnd();

            for (int i = 0; i < end_dots.size(); i++) {
                sf::CircleShape circle;
                if (path_found) {

                    bool isPathDot = false;// could cut the repetition maybe
                    for (auto j : found_path) {
                        if (end_dots[i] == j) {
                            isPathDot = true;
                            break;
                        }
                    }
                    circle.setRadius(isPathDot? path_size : dot_size);
                    circle.setFillColor(isPathDot? found_path_colour : end_buffer_colour_unreached);
                }
                else {
                    circle.setRadius(dot_size);
                    circle.setFillColor(i? end_buffer_colour_unreached : start_colour);
                }
                circle.setOrigin(circle.getRadius(), circle.getRadius());
                circle.setPosition(static_cast<float>(end_dots[i].x), static_cast<float>(end_dots[i].y));
                window.draw(circle);
                if (i) {
                    sf::Vertex line[2] = {sf::Vector2f(static_cast<float>(end_dots[i].x), static_cast<float>(end_dots[i].y)), sf::Vector2f(static_cast<float>(end_dots[parent_index[i]].x), static_cast<float>(end_dots[parent_index[i]].y))};
                    window.draw(line, 2, sf::Lines);
                }
            }


            window.display();
        }
        if (RRTreeG) {
            return;
        }
        if (RRTreeK) {
            return;
        }
    }

    RRT2BaseSingle* RRTreeS = nullptr;
    RRT2BaseDouble* RRTreeD = nullptr;
    RRT2BaseGraph* RRTreeG = nullptr;
    RRT2BaseKinodynamic* RRTreeK = nullptr;

    sf::RenderWindow &window;
    float dot_size = 7.5;
    float path_size = 10;
    sf::Color start_colour = sf::Color::Cyan;
    sf::Color end_buffer_colour_unreached = sf::Color::Blue;
    sf::Color end_buffer_colour_reached = sf::Color::Green;
    sf::Color dot_colour = sf::Color::Red;
    sf::Color obstacle_colour = sf::Color::Magenta;
    sf::Color found_path_colour = sf::Color::White;
};


#endif //RRT_LIBRARY_OLD_RRT_H