//
// Created by ben on 19/06/2026.
//

#ifndef RRT_LIBRARY_RRT_VISUALIZER_H
#define RRT_LIBRARY_RRT_VISUALIZER_H

#include <SFML/Graphics.hpp>
#include "RRT.h"

class DrawableConstraint : public Constraint {
public:
    virtual void draw(sf::RenderWindow&, unsigned int, unsigned int) const = 0;
};
class DrawableHyperRectangle final : public DrawableConstraint {
    public:
    DrawableHyperRectangle(const std::vector<double> &min, const std::vector<double> &max) : base({min, max}), min(min), max(max) {
        if (min.size() != max.size()) throw std::invalid_argument("Size Mismatch (DrawableHyperRectangle)");
    }

    [[nodiscard]] bool validConnection(const std::vector<double> &start, const std::vector<double> &end, const double margin) const override {
        if (start.size() != end.size() || start.size() != min.size()) throw std::invalid_argument("Size Mismatch (DrawableHyperRectangle)");
        return base.validConnection(start, end, margin);
    }
    void draw(sf::RenderWindow &window, const unsigned int x_index, const unsigned int y_index) const override {
        sf::RectangleShape rectangle;
        rectangle.setPosition({static_cast<float>(min[x_index]), static_cast<float>(min[y_index])});
        rectangle.setSize({static_cast<float>(max[x_index] - min[x_index]), static_cast<float>(max[y_index] - min[y_index])});
        rectangle.setFillColor(sf::Color::Magenta);

        window.draw(rectangle);
    }

private:
    HyperRectangle base;
    std::vector<double> min;
    std::vector<double> max;
};
class DrawableHyperSphere final : public DrawableConstraint {
public:
    DrawableHyperSphere(const double radius, const std::vector<double> &center) : base({radius, center}), radius(radius), center(center) {}

    [[nodiscard]] bool validConnection(const std::vector<double> &start, const std::vector<double> &end, const double margin) const override {
        if (start.size() != end.size() ||start.size() != center.size()) throw std::invalid_argument("Size Mismatch (DrawableHyperSphere)");
        return base.validConnection(start, end, margin);
    }
    void draw(sf::RenderWindow &window, const unsigned int x_index, const unsigned int y_index) const override {
        sf::CircleShape circle;
        circle.setOrigin({static_cast<float>(radius), static_cast<float>(radius)});
        circle.setPosition({static_cast<float>(center[x_index]), static_cast<float>(center[y_index])});
        circle.setRadius(static_cast<float>(radius));
        circle.setFillColor(sf::Color::Magenta);

        window.draw(circle);
    }

private:
    HyperSphere base;
    double radius;
    std::vector<double> center;
};

class DrawableObstacle : public Obstacle {
public:
    virtual void draw(sf::RenderWindow&) const = 0;
};
class DrawableCuboid final : public DrawableObstacle {
public:
    DrawableCuboid(const std::array<double, 3> &center, const std::array<double, 3> &size, const std::array<double, 3> &orientation_deg) : base(center, size, orientation_deg){
        this->center = center;
        half_size[0] = size[0] / 2.0;
        half_size[1] = size[1] / 2.0;
        half_size[2] = size[2] / 2.0;
        rotation_matrix = createRotationMatrix(orientation_deg[0], orientation_deg[1], orientation_deg[2]);
        bounding_box = createBoundingBox();
    }


    [[nodiscard]] bool validBoundingBox(const BoundingBox &swept_volume, const double margin) const override {
        return base.validBoundingBox(swept_volume, margin);
    }
    [[nodiscard]] bool validCapsule(const Capsule &swept_volume) const override{
        return base.validCapsule(swept_volume);
    }
    [[nodiscard]] bool validSDF(const SDF &swept_volume) const override{
        return base.validSDF(swept_volume);
    }
    void draw(sf::RenderWindow &window) const override {}

private:
    Cuboid base;
    std::array<double, 3> center;
    std::array<double, 3> half_size;
    std::array<std::array<double, 3>, 3> rotation_matrix;
    std::array<std::array<double, 2>, 3> bounding_box;


    [[nodiscard]] static std::array<std::array<double, 3>, 3>  createRotationMatrix(const double roll_deg, const double pitch_deg, const double yaw_deg) {
        std::array<std::array<double, 3>, 3> orientation{};
        const double roll  = roll_deg * std::numbers::pi / 180.0;
        const double pitch = pitch_deg * std::numbers::pi / 180.0;
        const double yaw   = yaw_deg * std::numbers::pi / 180.0;

        const double cos_x = cos(roll);
        const double sin_x = sin(roll);
        const double cos_y = cos(pitch);
        const double sin_y = sin(pitch);
        const double cos_z = cos(yaw);
        const double sin_z = sin(yaw);

        orientation[0][0] = cos_z*cos_y;
        orientation[0][1] = cos_z*sin_y*sin_x - sin_z*cos_x;
        orientation[0][2] = cos_z*sin_y*cos_x + sin_z*sin_x;

        orientation[1][0] = sin_z*cos_y;
        orientation[1][1] = sin_z*sin_y*sin_x + cos_z*cos_x;
        orientation[1][2] = sin_z*sin_y*cos_x - cos_z*sin_x;

        orientation[2][0] = -sin_y;
        orientation[2][1] = cos_y*sin_x;
        orientation[2][2] = cos_y*cos_x;
        return orientation;
    }
    //quaternion to matrix
    [[nodiscard]] std::array<std::array<double, 2>, 3> createBoundingBox() const {
        std::array<std::array<double, 2>, 3> new_bounding_box{};
        const double x_chainge = rotation_matrix[0][0]*half_size[0] + rotation_matrix[0][1]*half_size[1] + rotation_matrix[0][2]*half_size[2];
        const double y_chainge = rotation_matrix[1][0]*half_size[0] + rotation_matrix[1][1]*half_size[1] + rotation_matrix[1][2]*half_size[2];
        const double z_chainge = rotation_matrix[2][0]*half_size[0] + rotation_matrix[2][1]*half_size[1] + rotation_matrix[2][2]*half_size[2];

        new_bounding_box[0][0] = center[0] - x_chainge;
        new_bounding_box[0][1] = center[1] - y_chainge;
        new_bounding_box[0][2] = center[2] - z_chainge;
        new_bounding_box[1][0] = center[0] + x_chainge;
        new_bounding_box[1][1] = center[1] + y_chainge;
        new_bounding_box[1][2] = center[2] + z_chainge;
        return new_bounding_box;
    }

};

class PositionFromPoint {
public:
    virtual ~PositionFromPoint() = default;

    [[nodiscard]] virtual std::array<double, 3> getPosition(const std::vector<double>&) const = 0;
};
class DirectPositionFromPoint : public PositionFromPoint {
public:
    [[nodiscard]] std::array<double, 3> getPosition(const std::vector<double> &point) const override {
        if (point.size() < 2 || point.size() > 3) throw std::invalid_argument("Invalid point size (DirectPositionFromPoint)");
        std::array<double, 3> position{};
        position[0] = point[0];
        position[1] = point[1];
        position[2] = point.size() == 2? 1 : point[2];
        return position;
    }
};


class RRT_Visualiser {
public:
    explicit RRT_Visualiser(sf::RenderWindow &window, System &system) : window(window), system(system) {
        points_to_path = system.getPointsToPath();
    }


    void setGeometricBounds(const std::array<double, 3> &new_xyz_min, const std::array<double, 3> &new_xyz_max) {
        for (unsigned int i = 0; i < 3; i++) {
            if (new_xyz_min[i] > new_xyz_max[i]) return;
        }
        xyz_min = new_xyz_min;
        xyz_max = new_xyz_max;
    }
    void estimateGeometricBounds(const unsigned int samples, const double margin, const PositionFromPoint* position_from_point) {//switch to sobol sampling for improvements

        const std::vector<double> state_bounds_min = system.getMinBounds();
        const std::vector<double> state_bounds_max = system.getMaxBounds();

        std::vector<double> new_point;
        new_point.resize(state_bounds_min.size());
        for (unsigned int j = 0; j < state_bounds_min.size(); j++) {
            new_point[j] = fmod(rand(), state_bounds_max[j] - state_bounds_min[j]) + state_bounds_min[j];
        }
        std::array<double, 3> new_xyz_point = position_from_point->getPosition(new_point);

        std::array<double, 3> new_xyz_min = {new_xyz_point[0], new_xyz_point[1], new_xyz_point[2]};
        std::array<double, 3> new_xyz_max = new_xyz_min;

        for (unsigned int i = 0; i < samples; i++) {
            for (unsigned int j = 0; j < state_bounds_min.size(); j++) {
                new_point[j] = fmod(rand(), state_bounds_max[j] - state_bounds_min[j]) + state_bounds_min[j];
            }
            new_xyz_point = position_from_point->getPosition(new_point);
            if (new_xyz_point[0] < new_xyz_min[0]) new_xyz_min[0] = new_xyz_point[0];
            if (new_xyz_point[0] > new_xyz_max[0]) new_xyz_max[0] = new_xyz_point[0];
            if (new_xyz_point[1] < new_xyz_min[1]) new_xyz_min[1] = new_xyz_point[1];
            if (new_xyz_point[1] > new_xyz_max[1]) new_xyz_max[1] = new_xyz_point[1];
            if (new_xyz_point[2] < new_xyz_min[2]) new_xyz_min[2] = new_xyz_point[2];
            if (new_xyz_point[2] > new_xyz_max[2]) new_xyz_max[2] = new_xyz_point[2];
        }
        xyz_min = {new_xyz_min[0]*margin, new_xyz_min[1]*margin, new_xyz_min[2]*margin};
        xyz_max = {new_xyz_max[0]*margin, new_xyz_max[1]*margin, new_xyz_max[2]*margin};
    }
    void setConnectionSteps(const unsigned int steps) {
        connection_steps = steps;
    }


    void drawFlat(const RRT &tree, const unsigned int x_index, const unsigned int y_index) const {
        window.clear();

        const std::vector<double> min = system.getMinBounds();
        const std::vector<double> max = system.getMaxBounds();
        if (std::max(x_index, y_index) >= min.size()) throw std::out_of_range("RRT::Visualizer::drawFlat needs x_index and y_index to be valid");


        const float dot_radius = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
        const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(max[x_index] - min[x_index]);
        const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(max[y_index] - min[y_index]);

        const std::vector<std::vector<double>> dots = tree.getTree();
        const std::vector<std::vector<double>> path = tree.getPath();
        const std::vector<double> end_point = tree.getEndPoint();
        const std::vector<unsigned int> parentIndexes = tree.getParentIndexes();


        //draw constraints
        sf::View world_view;
        const sf::Vector2f worldSize = {static_cast<float>(max[x_index] - min[x_index]), -static_cast<float>(max[y_index] - min[y_index])};
        world_view.setCenter({static_cast<float>(min[x_index] + worldSize.x * 0.5f), static_cast<float>(min[y_index] - worldSize.y * 0.5f)});
        world_view.setSize(worldSize);
        world_view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
        window.setView(world_view);
        for (const std::vector<const Constraint*> constraints = system.getConstraints(); const Constraint* constraint : constraints) {
            if (const DrawableConstraint* drawable_constraint = dynamic_cast<const DrawableConstraint*>(constraint)) {
                drawable_constraint->draw(window, x_index, y_index);
            }
        }
        sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
        screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
        window.setView(screen_view);


        //draw start and end
        sf::CircleShape start_end(dot_radius*2);
        start_end.setOrigin({dot_radius*2, dot_radius*2});
        start_end.setPosition({static_cast<float>(dots[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Cyan);
        window.draw(start_end);
        start_end.setPosition({static_cast<float>(end_point[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(end_point[y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Green);
        window.draw(start_end);


        //dots and lines
        sf::CircleShape tree_dot(dot_radius);
        tree_dot.setOrigin({dot_radius, dot_radius});
        tree_dot.setFillColor(sf::Color::Red);
        for (unsigned int i = 1; i < dots.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }

        //path
        sf::CircleShape path_dot(dot_radius*1.5f);
        path_dot.setOrigin({dot_radius*1.5f, dot_radius*1.5f});
        path_dot.setFillColor(sf::Color::White);
        for (const std::vector<double>& i : path) {
            path_dot.setPosition({static_cast<float>(i[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[y_index] - min[y_index]) * scale_y});
            window.draw(path_dot);
        }

        window.display();
    }
    void projectDraw(const RRT &tree, const PositionFromPoint* position_from_point) const {
        if (xyz_max.empty()) throw std::invalid_argument("RRT_Visualiser::projectDraw() Requires geometric bounds \nplease call RRT_Visualiser::setGeometricBounds() or RRT_Visualiser::estimateGeometricBounds()");
        window.clear();




        // screen xy scaling + projection

        const std::vector<std::vector<double>> dots = tree.getTree();
        const std::vector<std::vector<double>> path = tree.getPath();
        std::vector<std::array<double, 3>> xyz_dots(dots.size());
        std::vector<std::array<double, 3>> xyz_path(path.size());
        for (unsigned int i = 0; i < dots.size(); i++) {
            xyz_dots[i] = position_from_point->getPosition(dots[i]);
        }
        for (unsigned int i = 0; i < path.size(); i++) {
            xyz_path[i] = position_from_point->getPosition(path[i]);
        }
        const std::array<double, 3> end_point = position_from_point->getPosition(tree.getEndPoint());
        const std::vector<unsigned int> parentIndexes = tree.getParentIndexes();
        std::vector<const Obstacle*> obstacles = system.getObstacles();

        //draw obstacles


        //draw end


        //draw dots from bottom to top if parent = root colour = cyan

        //highlight path

        window.display();
    }
    void camProjectDraw(const RRT &tree, const PositionFromPoint &position_from_point, const double roll, const double pitch, const double yaw) const {
        window.clear();

        if (xyz_max.empty()) {
            std::cout << "set bounds\n";
            return;
        }

        // sort dots from far to close, draw in order (255 alpha?) (maybe not needed is alpha is 100)


        std::vector<double> center(3);
        for (unsigned int i = 0; i < 3; i++) {
            center[i] = (xyz_max[i] - xyz_min[i]) / 2.0 + xyz_min[i];
        } // <- pivot point

        if (!roll && !pitch && !yaw) {
            float dot_size_min = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
            float dot_size_max = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.03);
            const double scale_z = xyz_min[2] == xyz_max[2]? 0 : (dot_size_max - dot_size_min) / (xyz_max[2] - xyz_min[2]);
            const double offset = xyz_min[2] == xyz_max[2] ? (dot_size_max - dot_size_min)/2 + dot_size_min : dot_size_min - (xyz_min[2]*(dot_size_max - dot_size_min)) / (xyz_max[2] - xyz_min[2]);
            const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(xyz_max[0] - xyz_min[0]);
            const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(xyz_max[1] - xyz_min[1]);


            const std::vector<std::vector<double>> dots = tree.getTree();
            const std::vector<std::vector<double>> path = tree.getPath();
            std::vector<std::array<double, 3>> xyz_dots(dots.size());
            std::vector<std::array<double, 3>> xyz_path(path.size());
            for (unsigned int i = 0; i < dots.size(); i++) xyz_dots[i] = position_from_point.getPosition(dots[i]);
            for (unsigned int i = 0; i < path.size(); i++) xyz_path[i] = position_from_point.getPosition(path[i]);
            const std::array<double, 3> end_point = position_from_point.getPosition(tree.getEndPoint());
            const std::vector<unsigned int> parentIndexes = tree.getParentIndexes();


            sf::View view = window.getView();
            const sf::Vector2f worldSize = {static_cast<float>(xyz_max[0] - xyz_min[0]), -static_cast<float>(xyz_max[1] - xyz_min[1])};
            view.setCenter({static_cast<float>(center[0]), static_cast<float>(center[1])});
            view.setSize(worldSize);
            view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
            window.setView(view);


            //draw obstacles

            sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
            screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
            window.setView(screen_view);



            //dots and lines
            for (unsigned int i = 1; i < dots.size(); i++) {
                const float dot_radius = static_cast<float>(dots[i][2]*scale_z +offset);
                sf::CircleShape tree_dot(dot_radius);
                tree_dot.setOrigin({dot_radius, dot_radius});
                tree_dot.setFillColor(sf::Color(255, 0, 0, 100));
                for (unsigned int j = 0; j < connection_steps; j++) {
                    std::array<double, 3> line_start = position_from_point.getPosition(points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j)/connection_steps));
                    std::array<double, 3> line_end = position_from_point.getPosition(points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j + 1)/connection_steps));
                    sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                    line[0].position = sf::Vector2f(static_cast<float>(line_start[0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[1] - xyz_min[1]) * scale_y);
                    line[1].position = sf::Vector2f(static_cast<float>(line_end[0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[1] - xyz_min[1]) * scale_y);
                    window.draw(line);
                }
                tree_dot.setPosition({static_cast<float>(dots[i][0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[i][1] - xyz_min[1]) * scale_y});
                window.draw(tree_dot);
            }


            //path
            for (const std::vector<double>& i : path) {
                const float path_radius = static_cast<float>(i[2]*scale_z + offset);
                sf::CircleShape path_dot(path_radius);
                path_dot.setOrigin({path_radius, path_radius});
                path_dot.setFillColor(sf::Color::White);
                path_dot.setPosition({static_cast<float>(i[0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[1] - xyz_min[1]) * scale_y});
                window.draw(path_dot);
            }

            //draw start and end
            const float start_radius = static_cast<float>(dots[0][2]*scale_z + offset);
            sf::CircleShape start(start_radius);
            start.setOrigin({start_radius, start_radius});
            start.setPosition({static_cast<float>(dots[0][0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[0][1] - xyz_min[1]) * scale_y});
            start.setFillColor(sf::Color(0, 255, 255, 100));
            window.draw(start);
            const float end_radius = static_cast<float>(end_point[2]*scale_z + offset);
            sf::CircleShape end(end_radius);
            end.setOrigin({end_radius, end_radius});
            end.setPosition({static_cast<float>(end_point[0] - xyz_min[0]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(end_point[1] - xyz_min[1]) * scale_y});
            end.setFillColor(sf::Color(0, 255, 0, 100));
            window.draw(end);
        }


        window.display();
    }

    void drawFlat(const RRT_Star &tree, const unsigned int x_index, const unsigned int y_index) const {
        window.clear();

        const std::vector<double> min = system.getMinBounds();
        const std::vector<double> max = system.getMaxBounds();
        if (std::max(x_index, y_index) >= min.size()) throw std::out_of_range("RRT::Visualizer::drawFlat needs x_index and y_index to be valid");

        const float dot_radius = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
        const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(max[x_index] - min[x_index]);
        const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(max[y_index] - min[y_index]);


        const std::vector<std::vector<double>> dots = tree.getTree();
        const std::vector<std::vector<double>> path = tree.getPath();
        const std::vector<double> costs = tree.getCosts();
        const std::vector<double> end_point = tree.getEndPoint();
        const std::vector<unsigned int> parentIndexes = tree.getParentIndexes();

        double max_cost = -1;
        for (double cost : costs) if (cost > max_cost) max_cost = cost;
        std::vector<double> red_cost(costs.size());
        for (unsigned int i = 0; i < costs.size(); i++) red_cost[i] = (costs[i] / max_cost) * 255;


        //draw constraints
        sf::View world_view;
        const sf::Vector2f worldSize = {static_cast<float>(max[x_index] - min[x_index]), -static_cast<float>(max[y_index] - min[y_index])};
        world_view.setCenter({static_cast<float>(min[x_index] + worldSize.x * 0.5f), static_cast<float>(min[y_index] - worldSize.y * 0.5f)});
        world_view.setSize(worldSize);
        world_view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
        window.setView(world_view);
        for (const std::vector<const Constraint*> constraints = system.getConstraints(); const Constraint* constraint : constraints) {
            if (const DrawableConstraint* drawable_constraint = dynamic_cast<const DrawableConstraint*>(constraint)) {
                drawable_constraint->draw(window, x_index, y_index);
            }
        }
        sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
        screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
        window.setView(screen_view);


        //draw start and end
        sf::CircleShape start_end(dot_radius*2);
        start_end.setOrigin({dot_radius*2, dot_radius*2});
        start_end.setPosition({static_cast<float>(dots[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Cyan);
        window.draw(start_end);
        start_end.setPosition({static_cast<float>(end_point[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(end_point[y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Green);
        window.draw(start_end);


        //dots and lines
        sf::CircleShape tree_dot(dot_radius);
        tree_dot.setOrigin({dot_radius, dot_radius});
        tree_dot.setFillColor(sf::Color::Red);
        for (unsigned int i = 1; i < dots.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j + 1)/connection_steps);
                unsigned char start_red_cost = static_cast<unsigned char>((red_cost[i] - red_cost[parentIndexes[i]])*(static_cast<double>(j)/connection_steps) + red_cost[parentIndexes[i]]);
                unsigned char end_red_cost = static_cast<unsigned char>((red_cost[i] - red_cost[parentIndexes[i]])*(static_cast<double>(j+1)/connection_steps) + red_cost[parentIndexes[i]]);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                line[0].color = sf::Color(start_red_cost, 255 - start_red_cost, 0);
                line[1].color = sf::Color(end_red_cost, 255 - end_red_cost, 0);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }


        //path
        sf::CircleShape path_dot(dot_radius*1.5f);
        path_dot.setOrigin({dot_radius*1.5f, dot_radius*1.5f});
        path_dot.setFillColor(sf::Color::White);
        for (const std::vector<double>& i : path) {
            path_dot.setPosition({static_cast<float>(i[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[y_index] - min[y_index]) * scale_y});
            window.draw(path_dot);
        }


        window.display();
    }
    void projectDraw(const RRT_Star &tree) const {}
    void camProjectDraw(const RRT_Star &tree, double roll, double pitch, double, double yaw) const {}

    void drawFlat(const Bi_RRT &tree, const unsigned int x_index, const unsigned int y_index) const {
        window.clear();

        const std::vector<double> min = system.getMinBounds();
        const std::vector<double> max = system.getMaxBounds();
        if (std::max(x_index, y_index) >= min.size()) throw std::out_of_range("RRT::Visualizer::drawFlat needs x_index and y_index to be valid");

        const float dot_radius = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
        const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(max[x_index] - min[x_index]);
        const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(max[y_index] - min[y_index]);

        const std::vector<std::vector<double>> dots1 = tree.getTree1();
        const std::vector<std::vector<double>> dots2 = tree.getTree2();
        const std::vector<unsigned int> parentIndexes1 = tree.getParentIndexes1();
        const std::vector<unsigned int> parentIndexes2 = tree.getParentIndexes2();
        const std::vector<std::vector<double>> path = tree.getPath();


        //draw constraints
        sf::View world_view;
        const sf::Vector2f worldSize = {static_cast<float>(max[x_index] - min[x_index]), -static_cast<float>(max[y_index] - min[y_index])};
        world_view.setCenter({static_cast<float>(min[x_index] + worldSize.x * 0.5f), static_cast<float>(min[y_index] - worldSize.y * 0.5f)});
        world_view.setSize(worldSize);
        world_view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
        window.setView(world_view);
        for (const std::vector<const Constraint*> constraints = system.getConstraints(); const Constraint* constraint : constraints) {
            if (const DrawableConstraint* drawable_constraint = dynamic_cast<const DrawableConstraint*>(constraint)) {
                drawable_constraint->draw(window, x_index, y_index);
            }
        }
        sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
        screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
        window.setView(screen_view);

        //dots and lines
        sf::CircleShape tree_dot(dot_radius);
        tree_dot.setOrigin({dot_radius, dot_radius});
        tree_dot.setFillColor(sf::Color::Red);
        for (unsigned int i = 1; i < dots1.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots1[parentIndexes1[i]], dots1[i], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots1[parentIndexes1[i]], dots1[i], static_cast<double>(j + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots1[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots1[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }
        tree_dot.setFillColor(sf::Color::Blue);
        for (unsigned int i = 1; i < dots2.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots2[i], dots2[parentIndexes2[i]], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots2[i], dots2[parentIndexes2[i]], static_cast<double>(j + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots2[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots2[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }


        //draw start and end
        sf::CircleShape start_end(dot_radius*2);
        start_end.setOrigin({dot_radius*2, dot_radius*2});
        start_end.setPosition({static_cast<float>(dots1[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots1[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Cyan);
        window.draw(start_end);
        start_end.setPosition({static_cast<float>(dots2[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots2[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Green);
        window.draw(start_end);


        //path
        sf::CircleShape path_dot(dot_radius*1.5f);
        path_dot.setOrigin({dot_radius*1.5f, dot_radius*1.5f});
        path_dot.setFillColor(sf::Color::White);
        for (const std::vector<double>& i : path) {
            path_dot.setPosition({static_cast<float>(i[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[y_index] - min[y_index]) * scale_y});
            window.draw(path_dot);
        }


        //linking the trees
        if (tree.endFound()) {
            for (unsigned int i = 0; i < connection_steps; i++) {
                std::vector<double> line_start = points_to_path->sample(dots1[tree.getStartLink()], dots2[tree.getEndLink()], static_cast<double>(i)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots1[tree.getStartLink()], dots2[tree.getEndLink()], static_cast<double>(i + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
        }

        window.display();
    }
    void projectDraw(const Bi_RRT &tree) const {}
    void camProjectDraw(const Bi_RRT &tree, double roll, double pitch, double, double yaw) const {}

    void drawFlat(const RRT_Connect &tree, const unsigned int x_index, const unsigned int y_index) const {
        window.clear();

        const std::vector<double> min = system.getMinBounds();
        const std::vector<double> max = system.getMaxBounds();
        if (std::max(x_index, y_index) >= min.size()) throw std::out_of_range("RRT::Visualizer::drawFlat needs x_index and y_index to be valid");

        const float dot_radius = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
        const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(max[x_index] - min[x_index]);
        const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(max[y_index] - min[y_index]);

        const std::vector<std::vector<double>> dots1 = tree.getTree1();
        const std::vector<std::vector<double>> dots2 = tree.getTree2();
        const std::vector<unsigned int> parentIndexes1 = tree.getParentIndexes1();
        const std::vector<unsigned int> parentIndexes2 = tree.getParentIndexes2();
        const std::vector<std::vector<double>> path = tree.getPath();
        const std::vector<const Constraint*> constraints = system.getConstraints();


        //draw constraints
        sf::View world_view;
        const sf::Vector2f worldSize = {static_cast<float>(max[x_index] - min[x_index]), -static_cast<float>(max[y_index] - min[y_index])};
        world_view.setCenter({static_cast<float>(min[x_index] + worldSize.x * 0.5f), static_cast<float>(min[y_index] - worldSize.y * 0.5f)});
        world_view.setSize(worldSize);
        world_view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
        window.setView(world_view);
        for (const std::vector<const Constraint*> constraints = system.getConstraints(); const Constraint* constraint : constraints) {
            if (const DrawableConstraint* drawable_constraint = dynamic_cast<const DrawableConstraint*>(constraint)) {
                drawable_constraint->draw(window, x_index, y_index);
            }
        }
        sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
        screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
        window.setView(screen_view);

        //dots and lines
        sf::CircleShape tree_dot(dot_radius);
        tree_dot.setOrigin({dot_radius, dot_radius});
        tree_dot.setFillColor(sf::Color::Red);
        for (unsigned int i = 1; i < dots1.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots1[parentIndexes1[i]], dots1[i], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots1[parentIndexes1[i]], dots1[i], static_cast<double>(j + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots1[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots1[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }
        tree_dot.setFillColor(sf::Color::Blue);
        for (unsigned int i = 1; i < dots2.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots2[i], dots2[parentIndexes2[i]], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots2[i], dots2[parentIndexes2[i]], static_cast<double>(j + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots2[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots2[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }


        //draw start and end
        sf::CircleShape start_end(dot_radius*2);
        start_end.setOrigin({dot_radius*2, dot_radius*2});
        start_end.setPosition({static_cast<float>(dots1[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots1[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Cyan);
        window.draw(start_end);
        start_end.setPosition({static_cast<float>(dots2[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots2[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Green);
        window.draw(start_end);


        //path
        sf::CircleShape path_dot(dot_radius*1.5f);
        path_dot.setOrigin({dot_radius*1.5f, dot_radius*1.5f});
        path_dot.setFillColor(sf::Color::White);
        for (const std::vector<double>& i : path) {
            path_dot.setPosition({static_cast<float>(i[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[y_index] - min[y_index]) * scale_y});
            window.draw(path_dot);
        }


        //linking the trees
        if (tree.endFound()) {
            for (unsigned int i = 0; i < connection_steps; i++) {
                std::vector<double> line_start = points_to_path->sample(dots1[tree.getStartLink()], dots2[tree.getEndLink()], static_cast<double>(i)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots1[tree.getStartLink()], dots2[tree.getEndLink()], static_cast<double>(i + 1)/connection_steps);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                window.draw(line);
            }
        }

        window.display();
    }
    void projectDraw(const RRT_Connect &tree) const {}
    void camProjectDraw(const RRT_Connect &tree, double roll, double pitch, double, double yaw) const {}

    void drawFlat(const Informed_RRT_Star &tree, const unsigned int x_index, const unsigned int y_index) const {
        window.clear();

        const std::vector<double> min = system.getMinBounds();
        const std::vector<double> max = system.getMaxBounds();
        if (std::max(x_index, y_index) >= min.size()) throw std::out_of_range("RRT::Visualizer::drawFlat needs x_index and y_index to be valid");

        const float dot_radius = static_cast<float>(std::min(window.getSize().x, window.getSize().y)*0.005);
        const float scale_x = static_cast<float>(window.getSize().x) / static_cast<float>(max[x_index] - min[x_index]);
        const float scale_y = static_cast<float>(window.getSize().y) / static_cast<float>(max[y_index] - min[y_index]);


        const std::vector<std::vector<double>> dots = tree.getTree();
        const std::vector<std::vector<double>> path = tree.getPath();
        const std::vector<double> costs = tree.getCosts();
        const std::vector<double> end_point = tree.getEndPoint();
        const std::vector<unsigned int> parentIndexes = tree.getParentIndexes();

        double max_cost = -1;
        for (double cost : costs) if (cost > max_cost) max_cost = cost;
        std::vector<double> red_cost(costs.size());
        for (unsigned int i = 0; i < costs.size(); i++) red_cost[i] = (costs[i] / max_cost) * 255;


        //draw constraints
        sf::View world_view;
        const sf::Vector2f worldSize = {static_cast<float>(max[x_index] - min[x_index]), -static_cast<float>(max[y_index] - min[y_index])};
        world_view.setCenter({static_cast<float>(min[x_index] + worldSize.x * 0.5f), static_cast<float>(min[y_index] - worldSize.y * 0.5f)});
        world_view.setSize(worldSize);
        world_view.setViewport(sf::FloatRect{{0.f, 0.f}, {1.f, 1.f}});
        window.setView(world_view);
        for (const std::vector<const Constraint*> constraints = system.getConstraints(); const Constraint* constraint : constraints) {
            if (const DrawableConstraint* drawable_constraint = dynamic_cast<const DrawableConstraint*>(constraint)) {
                drawable_constraint->draw(window, x_index, y_index);
            }
        }
        sf::View screen_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
        screen_view.setCenter({static_cast<float>(window.getSize().x) / 2.f,static_cast<float>(window.getSize().y) / 2.f});
        window.setView(screen_view);


        //draw start and end
        sf::CircleShape start_end(dot_radius*2);
        start_end.setOrigin({dot_radius*2, dot_radius*2});
        start_end.setPosition({static_cast<float>(dots[0][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[0][y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Cyan);
        window.draw(start_end);
        start_end.setPosition({static_cast<float>(end_point[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(end_point[y_index] - min[y_index]) * scale_y});
        start_end.setFillColor(sf::Color::Green);
        window.draw(start_end);


        //dots and lines
        sf::CircleShape tree_dot(dot_radius);
        tree_dot.setOrigin({dot_radius, dot_radius});
        tree_dot.setFillColor(sf::Color::Red);
        for (unsigned int i = 1; i < dots.size(); i++) {
            for (unsigned int j = 0; j < connection_steps; j++) {
                std::vector<double> line_start = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j)/connection_steps);
                std::vector<double> line_end = points_to_path->sample(dots[parentIndexes[i]], dots[i], static_cast<double>(j + 1)/connection_steps);
                unsigned char start_red_cost = static_cast<unsigned char>((red_cost[i] - red_cost[parentIndexes[i]])*(static_cast<double>(j)/connection_steps) + red_cost[parentIndexes[i]]);
                unsigned char end_red_cost = static_cast<unsigned char>((red_cost[i] - red_cost[parentIndexes[i]])*(static_cast<double>(j+1)/connection_steps) + red_cost[parentIndexes[i]]);
                sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                line[0].position = sf::Vector2f(static_cast<float>(line_start[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_start[y_index] - min[y_index])*scale_y);
                line[1].position = sf::Vector2f(static_cast<float>(line_end[x_index] - min[x_index])*scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(line_end[y_index] - min[y_index])*scale_y);
                line[0].color = sf::Color(start_red_cost, 255 - start_red_cost, 0);
                line[1].color = sf::Color(end_red_cost, 255 - end_red_cost, 0);
                window.draw(line);
            }
            tree_dot.setPosition({static_cast<float>(dots[i][x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(dots[i][y_index] - min[y_index]) * scale_y});
            window.draw(tree_dot);
        }


        //path
        sf::CircleShape path_dot(dot_radius*1.5f);
        path_dot.setOrigin({dot_radius*1.5f, dot_radius*1.5f});
        path_dot.setFillColor(sf::Color::White);
        for (const std::vector<double>& i : path) {
            path_dot.setPosition({static_cast<float>(i[x_index] - min[x_index]) * scale_x, static_cast<float>(window.getSize().y) - static_cast<float>(i[y_index] - min[y_index]) * scale_y});
            window.draw(path_dot);
        }


        window.display();
    }
    void projectDraw(const Informed_RRT_Star &tree) const {}
    void camProjectDraw(const Informed_RRT_Star &tree, double roll, double pitch, double, double yaw) const {}


private:
    sf::RenderWindow &window;
    System &system;
    const PointsToPath* points_to_path;


    std::array<double, 3> xyz_min;
    std::array<double, 3> xyz_max;
    unsigned int connection_steps = 5;
};


#endif //RRT_LIBRARY_RRT_VISUALIZER_H