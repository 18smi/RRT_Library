//add namespace to make final library clean


#ifndef RRT_LIBRARY_RRT_H
#define RRT_LIBRARY_RRT_H


#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <vector>
#include <memory>
#include <optional>
#include <SFML/System/Clock.hpp>
#define root_index UINT_MAX



class SamplingStrategy {
public:
    virtual ~SamplingStrategy() = default;
    [[nodiscard]] virtual std::vector<double> generatePoint(const std::vector<double>&, const std::vector<double>&) = 0;
};
class StochasticSampling final : public SamplingStrategy {
    [[nodiscard]] std::vector<double> generatePoint(const std::vector<double> &min, const std::vector<double> &max) override {
        if (min.size() != max.size()) throw std::invalid_argument("Size Mismatch (StochasticSampling)");
        std::vector<double> new_point(min.size());
        for (unsigned int i = 0; i < min.size(); i++) {
            new_point[i] = min[i] + (rand() / static_cast<double>(RAND_MAX)) * (max[i] - min[i]);
        }
        return new_point;
    }
};
class BiasedSampling final : public SamplingStrategy {
public:
    BiasedSampling(const std::vector<double>& end, const double bias) : end_point(end), bias(bias) {
        if (bias < 0 || bias > 1) throw std::invalid_argument("bias Must Be Between 0 And 1");
    }

    [[nodiscard]] std::vector<double> generatePoint(const std::vector<double>& min, const std::vector<double>& max) override {
        if (min.size() != max.size() || end_point.size() != max.size()) throw std::invalid_argument("Size Mismatch (BiasedSampling)");

        if (rand() / static_cast<double>(RAND_MAX) < bias) return end_point;

        std::vector<double> new_point(min.size());
        for (unsigned int i = 0; i < min.size(); i++) {
            new_point[i] = min[i] + (rand() / static_cast<double>(RAND_MAX)) * (max[i] - min[i]);
        }
        return new_point;
    }

private:
    std::vector<double> end_point;
    double bias;

};
//Sobol sampling
//more


class DistanceStrategy {
public:
    virtual ~DistanceStrategy() = default;
    [[nodiscard]] virtual double getDistance(const std::vector<double>&, const std::vector<double>&, const std::vector<bool>&) const = 0;
};
class EuclidianDistance final : public DistanceStrategy {
public:
    [[nodiscard]] double getDistance(const std::vector<double> &start, const std::vector<double> &end, const std::vector<bool> &wrapping) const override {
        if (start.size() != end.size() || start.size() != wrapping.size()) throw std::invalid_argument("Size Mismatch (DistanceStrategy)");
        double current = 0;
        for (unsigned int i = 0; i < start.size(); i++) {
            current += (start[i]-end[i]) * (start[i]-end[i]);
        }
        return std::sqrt(current);
    }
};
class WeightedEuclidianDistance final : public DistanceStrategy {
public:
    explicit WeightedEuclidianDistance(const std::vector<double> &scalers) : scalers(scalers) {}
    [[nodiscard]] double getDistance(const std::vector<double> &start, const std::vector<double> &end, const std::vector<bool> &wrapping) const override {
        if (start.size() != end.size() || start.size() != wrapping.size() || start.size() != scalers.size()) throw std::invalid_argument("Size Mismatch (EuclidianDistance)");
        double current = 0;
        for (unsigned int i = 0; i < start.size(); i++) {
            current += (start[i]-end[i]) * (start[i]-end[i]) * scalers[i];
        }
        return std::sqrt(current);
    }
private:
    std::vector<double> scalers;
};
class AdditiveDistance final : public DistanceStrategy {
public:
    [[nodiscard]] double getDistance(const std::vector<double> &start, const std::vector<double> &end, const std::vector<bool> &wrapping) const override {
        if (start.size() != end.size() || start.size() != wrapping.size()) throw std::invalid_argument("Size Mismatch (AdditiveDistance)");

        double current = 0;
        for (unsigned int i = 0; i < start.size(); i++) {
            current += fabs(end[i] - start[i]);
        }
        return current;
    }
};
//more


class CostFunction {
public:
    virtual ~CostFunction() = default;
    [[nodiscard]] virtual double calculateCost(const std::vector<double> &start, const std::vector<double> &end) = 0;
};
class DistanceCost final : public CostFunction{
public:
    DistanceCost(DistanceStrategy* distance_strategy, const std::vector<bool> &wrapping) : distance_strategy(distance_strategy), wrapping(wrapping){}

    [[nodiscard]] double calculateCost(const std::vector<double> &start, const std::vector<double> &end) override {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (DistanceCost)");
        return distance_strategy->getDistance(start, end, wrapping);
    }

private:
    std::unique_ptr<DistanceStrategy> distance_strategy;
    std::vector<bool> wrapping;
};



class PointsToPath {
public:
    virtual ~PointsToPath() = default;
    [[nodiscard]] virtual std::vector<double> sample(const std::vector<double>&, const std::vector<double>&, double) const = 0;
};
class LinearPath final : public PointsToPath {
public:
    [[nodiscard]] std::vector<double> sample(const std::vector<double> &start_point, const std::vector<double> &end_point, const double t) const override {
        if (start_point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (LinearPath)");
        if (t < 0 || t > 1) throw std::invalid_argument("t Must Be Between 0 And 1");
        std::vector<double> sampled_point(start_point.size());

        for (unsigned int i = 0; i < start_point.size(); i++) {
            sampled_point[i] = start_point[i] + (end_point[i] - start_point[i]) * t;
        }
        return sampled_point;
    }
};
class AxisSequentialPath final : public PointsToPath {
    public:
    [[nodiscard]] std::vector<double> sample(const std::vector<double> &start, const std::vector<double> &end, const double t) const override {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (LinearPath)");
        if (t < 0 || t > 1) throw std::invalid_argument("t Must Be Between 0 And 1");
        const double portion_size = 1.0 / static_cast<double>(start.size());
        std::vector<double> result(start.size());
        for (unsigned int i = 0; i < start.size(); i++) {
            if (i*portion_size > t) result[i] = start[i];
            else if ((i+1)*portion_size <= t) result[i] = end[i];
            else result[i] = ((end[i] - start[i]) * (t - i*portion_size)) / portion_size + start[i];
        }
        return result;
    }
};
//LinearKinematicPath
//CurvedKinematicPath


struct BoundingBox {
    BoundingBox(const double min_X, const double min_Y, const double min_Z, const double max_X, const double max_Y, const double max_Z) {
        min[0] = min_X;
        min[1] = min_Y;
        min[2] = min_Z;
        max[0] = max_X;
        max[1] = max_Y;
        max[2] = max_Z;
    }
    BoundingBox(const std::array<double, 3> &min, const std::array<double, 3> &max) : min(min), max(max) {}

    [[nodiscard]] std::array<double, 3> getMin() const {
        return min;
    }
    [[nodiscard]] std::array<double, 3> getMax() const {
        return max;
    }

private:
    std::array<double, 3> min;
    std::array<double, 3> max;
};
struct Capsule {
    Capsule(const std::array<double, 3> &position1, const std::array<double, 3> &position2, const double radius) : position1(position1), position2(position2), radius(radius) {}

    [[nodiscard]] std::array<double, 3> getPosition1() const {
        return position1;
    }
    [[nodiscard]] std::array<double, 3> getPosition2() const {
        return position2;
    }
    [[nodiscard]] double getRadius() const {
        return radius;
    }


private:
    std::array<double, 3> position1;
    std::array<double, 3> position2;
    double radius;
};
struct SDF {

};


class StateToGeometry {
public:
    virtual ~StateToGeometry() = default;
    [[nodiscard]] virtual BoundingBox getBoundingBox(const std::vector<double>&) const = 0;//axes aligned
    [[nodiscard]] virtual Capsule getCapsule(const std::vector<double>&) const = 0;
    [[nodiscard]] virtual SDF getSDF(const std::vector<double>&) const = 0;
};
class NoGeometry final : public StateToGeometry {
public:
    [[nodiscard]] BoundingBox getBoundingBox(const std::vector<double> &point) const override {return BoundingBox{0, 0, 0, 0, 0, 0};}
    [[nodiscard]] Capsule getCapsule(const std::vector<double> &point) const override {return Capsule{{0, 0, 0}, {0, 0, 0}, 0};}
    [[nodiscard]] SDF getSDF(const std::vector<double> &point) const override {return SDF{};}
};
class Simple3JointArm final : public StateToGeometry {
public:
    Simple3JointArm(const double length1, const double length2, const double link_thickness) : length1(length1), length2(length2), link_thickness(link_thickness) {

    }


    [[nodiscard]] BoundingBox getBoundingBox(const std::vector<double> &point) const override {
        if (point.size() != 3) throw std::invalid_argument("Point Size Must Equal 3 (Simple3JointArm)");

        const std::array<std::array<double, 3>, 3> joint_positions = getJointPositions(point);

        double x_min = joint_positions[0][0] - link_thickness;
        double x_max = joint_positions[0][0] + link_thickness;
        double y_min = joint_positions[0][1] - link_thickness;
        double y_max = joint_positions[0][1] + link_thickness;
        double z_min = joint_positions[0][2] - link_thickness;
        double z_max = joint_positions[0][2] + link_thickness;

        for (unsigned int i = 1; i < 3; i++) {
            x_min = std::min(joint_positions[i][0] - link_thickness, x_min);
            x_max = std::max(joint_positions[i][0] + link_thickness, x_max);
            y_min = std::min(joint_positions[i][1] - link_thickness, y_min);
            y_max = std::max(joint_positions[i][1] + link_thickness, y_max);
            z_min = std::min(joint_positions[i][2] - link_thickness, z_min);
            z_max = std::max(joint_positions[i][2] + link_thickness, z_max);
        }
        return BoundingBox{x_min, x_max, y_min, y_max, z_min, z_max};
    }
    [[nodiscard]] Capsule getCapsule(const std::vector<double> &point) const override {
        if (point.size() != 3) throw std::invalid_argument("Point Size Must Equal 3 (Simple3JointArm)");

    }
    [[nodiscard]] SDF getSDF(const std::vector<double> &point) const override {
        if (point.size() != 3) throw std::invalid_argument("Point Size Must Equal 3 (Simple3JointArm)");

    }

private:
    double length1;
    double length2;
    double link_thickness;

    [[nodiscard]] std::array<std::array<double, 3>, 3> getJointPositions(const std::vector<double> &point) const {
        if (point.size() != 3) throw std::invalid_argument("Point Size Must Equal 3 (Simple3JointArm)");

        std::array<std::array<double, 3>, 3> result{};

        const double joint1_xy = cos(point[1])*length1;
        const double joint2_xy = joint1_xy + cos(point[2])*length2;

        const double joint1_x = cos(point[0])*joint1_xy;
        const double joint1_y = sin(point[0])*joint1_xy;
        const double joint1_z = sin(point[1])*length1;

        //need to be edited to make 0 = aligned with point[1]
        const double joint2_x = cos(point[0])*joint2_xy;
        const double joint2_y = sin(point[0])*joint2_xy;
        const double joint2_z = joint1_z + sin(point[2])*length2;

        result[0] = {0, 0, 0};
        result[1] = {joint1_x, joint1_y, joint1_z};
        result[2] = {joint2_x, joint2_y, joint2_z};

        return result;
    }
};
//common types (7 dof arm, SE2, SE3, more)



class Constraint {
public:
    virtual ~Constraint() = default;
    [[nodiscard]] virtual bool validConnection(const std::vector<double>&, const std::vector<double>&, double) const = 0;
};
class HyperRectangle final : public Constraint {
    public:
    HyperRectangle(const std::vector<double> &min, const std::vector<double> &max) : min(min), max(max) {
        if (min.size() != max.size()) throw std::invalid_argument("Size Mismatch (HyperRectangle)");
    }


    [[nodiscard]] bool validConnection(const std::vector<double> &start, const std::vector<double> &end, const double margin) const override {
        if (start.size() != end.size() || start.size() != min.size()) throw std::invalid_argument("Size Mismatch (HyperRectangle)");
        double t_enter = 0.0;
        double t_exit  = 1.0;

        for (unsigned int i = 0; i < start.size(); ++i) {
            const double new_min = min[i] - margin;
            const double new_max = max[i] + margin;

            const double direction = end[i] - start[i];

            if (direction == 0) { // parallel
                if (start[i] < new_min || start[i] > new_max) return true;
                continue;
            }

            const double inverse_direction = 1 / direction;
            double t1 = (new_min - start[i]) * inverse_direction;
            double t2 = (new_max - start[i]) * inverse_direction;

            if (t1 > t2) {
                const double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }

            if (t1 > t_enter) t_enter = t1;
            if (t2 < t_exit)  t_exit  = t2;

            if (t_enter > t_exit) return true;

        }
        return t_enter > 1.0 || t_exit < 0.0;
    }


private:
    std::vector<double> min;
    std::vector<double> max;
};
class HyperSphere final : public Constraint {
    public:
    HyperSphere(const double radius, const std::vector<double> &center) : radius(radius), center(center) {}

    [[nodiscard]] bool validConnection(const std::vector<double> &start, const std::vector<double> &end, const double margin) const override {
        if (start.size() != end.size() ||start.size() != center.size()) throw std::invalid_argument("Size Mismatch (HyperSphere)");
        double a = 0;
        double b = 0;
        double c = -(radius + margin)*(radius + margin);

        for (unsigned int i = 0; i < start.size(); ++i) {
            a += (end[i] - start[i]) * (end[i] - start[i]);
            b += (end[i] - start[i]) * (start[i] - center[i]) * 2;
            c += (start[i] - center[i]) * (start[i] - center[i]);
        }
        if (b*b - 4*a*c < 0) return true;

        const double sqrtD = std::sqrt(b*b - 4*a*c);
        const double t1 = (-b - sqrtD) / (2*a);
        const double t2 = (-b + sqrtD) / (2*a);

        return !((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1));
    }


    private:
    double radius;
    std::vector<double> center;
};
//rotated hyper rect
//elips
//convex hull


class Obstacle {
public:
    virtual ~Obstacle() = default;
    [[nodiscard]] virtual bool validBoundingBox(const BoundingBox&, double) const = 0;
    [[nodiscard]] virtual bool validCapsule(const Capsule&) const = 0;
    [[nodiscard]] virtual bool validSDF(const SDF&) const = 0;
};
class Cuboid final : public Obstacle {
public:
    Cuboid(const std::array<double, 3> &center, const std::array<double, 3> &size, const std::array<double, 3> &orientation_deg) {
        this->center = center;
        half_size[0] = size[0] / 2.0;
        half_size[1] = size[1] / 2.0;
        half_size[2] = size[2] / 2.0;
        rotation_matrix = createRotationMatrix(orientation_deg[0], orientation_deg[1], orientation_deg[2]);
        bounding_box = createBoundingBox();
    }


    [[nodiscard]] bool validBoundingBox(const BoundingBox &swept_volume, const double margin) const override {
        const std::array<double, 3> min = swept_volume.getMin();
        const std::array<double, 3> max = swept_volume.getMax();

        if (bounding_box[0][0] + margin > max[0] || bounding_box[1][0] - margin < min[0]) return true;
        if (bounding_box[0][1] + margin > max[1] || bounding_box[1][1] - margin < min[1]) return true;
        if (bounding_box[0][2] + margin > max[2] || bounding_box[1][2] - margin < min[2]) return true;
        return false;
    }
    [[nodiscard]] bool validCapsule(const Capsule &swept_volume) const override{
        return true;
        //correct volume + capsule
    }
    [[nodiscard]] bool validSDF(const SDF &swept_volume) const override{
        return true;
        //SDF check
    }

private:
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
//Sphere (xyz, r)
//pill (xyz, rpy, r, h)
//cone (xyz, rpy, r, h)
//cut cone (xyz, rpy, br, tr, h)
//more (could except points to make complex polygons


struct SafetySettings {
    double constraint_safety_margin;
    double obstacle_safety_margin;
    unsigned int interpolation_steps;
};


class System {
public:
    System(const std::vector<double> &min_bound, const std::vector<double> &max_bound, const std::vector<bool> &wrapping, StateToGeometry* state_to_geometry, PointsToPath* points_to_path, const SafetySettings &safety_settings) : min_bound(min_bound), max_bound(max_bound), wrapping(wrapping), state_to_geometry(state_to_geometry), points_to_path(points_to_path), safety_settings(safety_settings) {
        if (min_bound.size() != max_bound.size() || min_bound.size() != wrapping.size()) throw std::invalid_argument("Size Mismatch (System)");
        if (!state_to_geometry || !points_to_path) throw std::invalid_argument("System was initialised with a nullptr");
    }


    void addObstacle(std::unique_ptr<Obstacle> new_obstacle) {
        if (!new_obstacle) throw std::invalid_argument("New obstacle is nullptr (System)");
        obstacles.push_back(std::move(new_obstacle));
    }
    void addObstacles(std::vector<std::unique_ptr<Obstacle>> &new_obstacles) {
        if (new_obstacles.empty()) throw std::invalid_argument("New obstacles is empty (System)");
        for (auto & new_obstacle : new_obstacles) {
            if (!new_obstacle) throw std::invalid_argument("New obstacle is nullptr (System)");
            obstacles.push_back(std::move(new_obstacle));
        }
    }
    void addConstraint(std::unique_ptr<Constraint> new_constraint) {
        if (!new_constraint) throw std::invalid_argument("New constraint is nullptr (System)");
        constraints.push_back(std::move(new_constraint));
    }
    void addConstraints(std::vector<std::unique_ptr<Constraint>> &new_constraints) {
        if (new_constraints.empty()) throw std::invalid_argument("New constraints is empty (System)");
        for (auto & new_constraint : new_constraints) {
            if (!new_constraint) throw std::invalid_argument("New constraints is nullptr (System)");
            constraints.push_back(std::move(new_constraint));
        }
    }

    [[nodiscard]] std::vector<double> getMinBounds() const {
        return min_bound;
    }
    [[nodiscard]] std::vector<double> getMaxBounds() const {
        return max_bound;
    }
    [[nodiscard]] std::vector<const Constraint*> getConstraints() const {
        std::vector<const Constraint*> result;
        result.reserve(constraints.size());
        for (const auto & constraint : constraints) {
            result.push_back(constraint.get());
        }
        return result;
    }
    [[nodiscard]] std::vector<const Obstacle*> getObstacles() const {
        std::vector<const Obstacle*> result;
        result.reserve(obstacles.size());
        for (const auto & obstacle : obstacles) {
            result.push_back(obstacle.get());
        }
        return result;
    }
    [[nodiscard]] PointsToPath* getPointsToPath() const {
        return points_to_path.get();
    }

    [[nodiscard]] std::vector<bool> getWrapping() const {
        return wrapping;
    }



    [[nodiscard]] bool validSegment(const std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (System::validSegment)");
        for (const auto & constraint : constraints) {
            for (unsigned int i = 0; i < safety_settings.interpolation_steps; ++i) {
                if (!constraint->validConnection(points_to_path->sample(start, end, static_cast<double>(i)/safety_settings.interpolation_steps), points_to_path->sample(start, end, static_cast<double>(i+1)/safety_settings.interpolation_steps), safety_settings.constraint_safety_margin)) return false;
            }
        }
        const BoundingBox bounding_box = createBoundingBox(start, end);
        std::optional<Capsule> capsule;
        std::optional<SDF> sdf;
        for (const auto & obstacle : obstacles) {
            if (!obstacle->validBoundingBox(bounding_box, safety_settings.obstacle_safety_margin)) {
                if (!capsule.has_value()) capsule = createCapsule(start, end);
                if (!obstacle->validCapsule(capsule.value())) {
                    if (!sdf.has_value()) sdf = createSDF(start, end);
                    if (!obstacle->validSDF(sdf.value())) return false;
                }
            }
        }
        return true;
    }

private:
    std::vector<double> min_bound;
    std::vector<double> max_bound;
    std::vector<bool> wrapping;
    std::unique_ptr<StateToGeometry> state_to_geometry;
    std::unique_ptr<PointsToPath> points_to_path;
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    std::vector<std::unique_ptr<Constraint>> constraints;
    SafetySettings safety_settings;


    [[nodiscard]] BoundingBox createBoundingBox(const std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (System::createBoundingBox)");
        const BoundingBox box = state_to_geometry->getBoundingBox(start);
        std::array<double, 3> min = box.getMin();
        std::array<double, 3> max = box.getMax();
        for (unsigned int i = 1; i < safety_settings.interpolation_steps; ++i) {
            BoundingBox new_box = state_to_geometry->getBoundingBox(points_to_path->sample(start, end, static_cast<double>(i)/safety_settings.interpolation_steps));
            min[0] = std::min(min[0], new_box.getMin()[0]);
            min[1] = std::min(min[1], new_box.getMin()[1]);
            min[2] = std::min(min[2], new_box.getMin()[2]);
            max[0] = std::max(max[0], new_box.getMax()[0]);
            max[1] = std::max(max[1], new_box.getMax()[1]);
            max[2] = std::max(max[2], new_box.getMax()[2]);
        }
        return BoundingBox{min, max};
    }
    [[nodiscard]] Capsule createCapsule(const std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (System::createCapsule)");
        return Capsule{{0, 0, 0}, {0, 0, 0}, 0};
    }
    [[nodiscard]] SDF createSDF(const std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (System::createSDF)");
        return SDF{};
    }
};


class RRT {
public:
    RRT(System &system, const std::vector<double> &start, const std::vector<double> &end, const double max_extend, SamplingStrategy* sampling, DistanceStrategy* distance_strategy) : system(system), end_point(end), max_extend(max_extend), sampling(sampling), distance(distance_strategy) {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (RRT)");
        if (!sampling || !distance_strategy) throw std::invalid_argument("Sampling or Distance Strategy missing (RRT)");
        points.push_back(start);
        parent_index.push_back(root_index);
    }

    [[nodiscard]] bool endFound() const {
        return end_found;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree() const {
        return points;
    }
    [[nodiscard]] std::vector<std::vector<double>> getPath() const {
        if (!endFound()) return {};
        std::vector<std::vector<double>> path;
        unsigned int current_index = points.size()-1;
        while (current_index != root_index) {
            path.push_back(points[current_index]);
            current_index = parent_index[current_index];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes() const {
        return parent_index;
    }
    [[nodiscard]] std::vector<double> getEndPoint() const {
        return end_point;
    }


    void step() {
        if (end_found) return;

        for (unsigned int i = 0; i < step_attempts; i++) {
            std::vector<double> new_point = sampling->generatePoint(system.getMinBounds(), system.getMaxBounds());
            const unsigned int closest_index = closestIndex(new_point);
            limitDistance(new_point, points[closest_index]);

            if (!system.validSegment(points[closest_index], new_point)) continue;

            points.push_back(new_point);
            parent_index.push_back(closest_index);
            if (distance->getDistance(new_point, end_point, system.getWrapping()) < max_extend) {
                if (system.validSegment(points[points.size()-1], end_point)) {
                    end_found = true;
                    parent_index.push_back(points.size()-1);
                    points.push_back(end_point);
                }
            }
            break;
        }
    }



private:
    bool end_found = false;
    unsigned int step_attempts = 100;
    std::vector<std::vector<double>> points;
    std::vector<unsigned int> parent_index;


    System &system;
    std::vector<double> end_point;
    double max_extend;
    std::unique_ptr<SamplingStrategy> sampling;
    std::unique_ptr<DistanceStrategy> distance;


    [[nodiscard]] unsigned int closestIndex(const std::vector<double> &point) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT Internal 1)");
        unsigned int closest_index = 0;
        double closest_distance = distance->getDistance(point, points[0], system.getWrapping());
        for (unsigned int i = 1; i < points.size(); i++) {
            if (const double try_distance = distance->getDistance(point, points[i], system.getWrapping()); try_distance < closest_distance) {
                closest_index = i;
                closest_distance = try_distance;
            }
        }
        return closest_index;
    }
    void limitDistance(std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size() || start.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT Internal 2)");
        const double start_distance = distance->getDistance(start, end, system.getWrapping());
        if (start_distance <= max_extend) return;

        for (unsigned int i = 0; i < start.size(); i++)
            start[i] = end[i] + (start[i] - end[i]) * (max_extend / start_distance);

    }
};

class RRT_Star {
public:
    RRT_Star(System &system, const std::vector<double> &start, const std::vector<double> &end, const double max_extend, SamplingStrategy* sampling, DistanceStrategy* distance_strategy, CostFunction* cost) : system(system), end_point(end), max_extend(max_extend), sampling(sampling), distance(distance_strategy), cost(cost) {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (RRT*)");
        if (!sampling || !distance_strategy || !cost) throw std::invalid_argument("Sampling, Distance or Cost Strategy missing (RRT*)");
        points.push_back(start);
        parent_index.push_back(root_index);
        costs.push_back(0);
    }


    [[nodiscard]] bool endFound() const {
        return end_found_index != root_index;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree() const {
        return points;
    }
    [[nodiscard]] std::vector<std::vector<double>> getPath() const {
        if (!endFound()) return {};
        std::vector<std::vector<double>> path;
        unsigned int current_index = end_found_index;
        while (current_index != root_index) {
            path.push_back(points[current_index]);
            current_index = parent_index[current_index];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    [[nodiscard]] std::vector<double> getPathCosts() const {
        if (!endFound()) return {};
        std::vector<double> path_costs;
        unsigned int current_index = end_found_index;
        while (current_index != root_index) {
            path_costs.push_back(costs[current_index]);
            current_index = parent_index[current_index];
        }
        std::reverse(path_costs.begin(), path_costs.end());
        return path_costs;
    }
    [[nodiscard]] std::vector<double> getCosts() const {
        return costs;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes() const {
        return parent_index;
    }
    [[nodiscard]] std::vector<double> getEndPoint() const {
        return end_point;
    }


    void step() {
        for (unsigned int i = 0; i < step_attempts; i++) {
            std::vector<double> new_point = sampling->generatePoint(system.getMinBounds(), system.getMaxBounds());
            const unsigned int closest_index = closestIndex(new_point);
            limitDistance(new_point, points[closest_index]);

            if (!system.validSegment(points[closest_index], new_point)) continue;


            std::vector<unsigned int> neighbour_indexes = findNeighbors(new_point);
            neighbour_indexes.push_back(closest_index);
            unsigned int best_parent = findBestParent(new_point, neighbour_indexes);


            points.push_back(new_point);
            parent_index.push_back(best_parent);
            costs.push_back(costs[best_parent] + cost->calculateCost(new_point, points[best_parent]));


            rewireNetwork(points.size()-1, neighbour_indexes);
            if (distance->getDistance(new_point, end_point, system.getWrapping()) < max_extend && (end_found_index == root_index || cost->calculateCost(new_point, end_point) + costs[points.size()-1] < costs[end_found_index])) {
                if (system.validSegment(points[points.size()-1], end_point)) {
                    parent_index.push_back(points.size()-1);
                    costs.push_back(cost->calculateCost(new_point, end_point) + costs[points.size()-1]);
                    points.push_back(end_point);
                    end_found_index = points.size()-1;
                }
            }
            return;
        }
    }



private:
    unsigned int end_found_index = root_index;
    unsigned int step_attempts = 100;
    std::vector<std::vector<double>> points;
    std::vector<unsigned int> parent_index;
    std::vector<double> costs;


    System &system;
    std::vector<double> end_point;
    double max_extend;
    std::unique_ptr<SamplingStrategy> sampling;
    std::unique_ptr<DistanceStrategy> distance;
    std::unique_ptr<CostFunction> cost;


    [[nodiscard]] unsigned int closestIndex(const std::vector<double> &point) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT* Internal 1)");
        unsigned int closest_index = 0;
        double closest_distance = distance->getDistance(point, points[0], system.getWrapping());
        for (unsigned int i = 1; i < points.size(); i++) {
            if (const double try_distance = distance->getDistance(point, points[i], system.getWrapping()); try_distance < closest_distance) {
                closest_index = i;
                closest_distance = try_distance;
            }
        }
        return closest_index;
    }

    void limitDistance(std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size() || start.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT* Internal 2)");
        const double start_distance = distance->getDistance(start, end, system.getWrapping());
        if (start_distance <= max_extend) return;

        for (unsigned int i = 0; i < start.size(); i++)
            start[i] = end[i] + (start[i] - end[i]) * (max_extend / start_distance);

    }

    [[nodiscard]] std::vector<unsigned int> findNeighbors(const std::vector<double> &point) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT* Internal 3)");
        std::vector<unsigned int> neighbours;

        const double gamma = 2*max_extend;
        double radius = gamma; //gamma * std::sqrt(std::log(points.size()) / static_cast<double>(points.size()));
        radius = std::min(radius, max_extend);


        for (unsigned int i = 0; i < points.size(); i++) {
            if (distance->getDistance(point, points[i], system.getWrapping()) < radius && system.validSegment(point, points[i])) {
                neighbours.push_back(i);
            }
        }
        return neighbours;
    }

    [[nodiscard]] unsigned int findBestParent(const std::vector<double> &point, const std::vector<unsigned int> &neighbour_indexes) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (RRT* Internal 4)");
        if (neighbour_indexes.empty()) throw std::invalid_argument("Empty Input (RRT* Internal 4)");
        unsigned int best_parent = neighbour_indexes[0];
        double current_cost = cost->calculateCost(point, points[neighbour_indexes[0]]) + costs[neighbour_indexes[0]];

        for (unsigned int i = 1; i < neighbour_indexes.size(); i++) {
            if (system.validSegment(point, points[neighbour_indexes[i]])) {
                double posable_cost = cost->calculateCost(point, points[neighbour_indexes[i]]) + costs[neighbour_indexes[i]];
                if (current_cost > posable_cost) {
                    best_parent = neighbour_indexes[i];
                    current_cost = posable_cost;
                }
            }
        }
        return best_parent;
    }

    void rewireNetwork(const unsigned int point_index, const std::vector<unsigned int> &neighbour_indexes) {
        if (point_index >= points.size()) throw std::invalid_argument("Index Error (RRT* Internal 5)");
        if (neighbour_indexes.empty()) throw std::invalid_argument("Empty Input (RRT* Internal 5)");
        for (const unsigned int neighbour_index : neighbour_indexes) {
            if (neighbour_index == point_index) continue;
            if (!system.validSegment(points[point_index], points[neighbour_index])) continue;

            const double potential_cost = cost->calculateCost(points[point_index], points[neighbour_index]) + costs[point_index];
            if (potential_cost < costs[neighbour_index]) {
                parent_index[neighbour_index] = point_index;
                costs[neighbour_index] = potential_cost;
                updateDescendantCosts(neighbour_index);
            }
        }
    }

    void updateDescendantCosts(const unsigned int point_index) {
        if (point_index >= points.size()) throw std::invalid_argument("Index Error (RRT* Internal 6)");
        for (unsigned int i = 0; i < points.size(); i++) {
            if (parent_index[i] == point_index) {
                costs[i] = cost->calculateCost(points[point_index], points[i]) + costs[point_index];
                updateDescendantCosts(i);
            }
        }
    }
};

class Bi_RRT {
public:
    Bi_RRT(System &system, const std::vector<double> &start, const std::vector<double> &end, const double max_extend, SamplingStrategy* sampling, DistanceStrategy* distance_strategy) : system(system), max_extend(max_extend), sampling(sampling), distance(distance_strategy) {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (BI_RRT)");
        if (!sampling || !distance_strategy) throw std::invalid_argument("Sampling or Distance Strategy missing (BI_RRT)");
        points1.push_back(start);
        parent_index1.push_back(root_index);
        points2.push_back(end);
        parent_index2.push_back(root_index);
    }

    [[nodiscard]] bool endFound() const {
        return start_link != root_index;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree1() const {
        return points1;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree2() const {
        return points2;
    }
    [[nodiscard]] std::vector<std::vector<double>> getPath() const {
        if (!endFound()) return {};
        std::vector<std::vector<double>> path;

        unsigned int current_index = start_link;
        while (current_index != root_index) {
            path.push_back(points1[current_index]);
            current_index = parent_index1[current_index];
        }
        std::reverse(path.begin(), path.end());

        current_index = end_link;
        while (current_index != root_index) {
            path.push_back(points2[current_index]);
            current_index = parent_index2[current_index];
        }

        return path;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes1() const {
        return parent_index1;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes2() const {
        return parent_index2;
    }
    [[nodiscard]] unsigned int getStartLink() const {
        return start_link;
    }
    [[nodiscard]] unsigned int getEndLink() const {
        return end_link;
    }


    void step() {
        if (start_link != root_index) return;

        for (unsigned int i = 0; i < step_attempts; i++) {
            std::vector<double> new_point = sampling->generatePoint(system.getMinBounds(), system.getMaxBounds());
            const unsigned int closest_index = closestActiveIndex(new_point);
            limitDistance(new_point, (active_tree1? points1 : points2)[closest_index]);

            if (active_tree1) {
                if (!system.validSegment(points1[closest_index], new_point)) continue;
            }
            else {
                if (!system.validSegment(new_point, points2[closest_index])) continue;
            }


            (active_tree1? points1 : points2).push_back(new_point);
            (active_tree1? parent_index1 : parent_index2).push_back(closest_index);

            active_tree1 = !active_tree1;
            const unsigned int closest_other_index = closestActiveIndex(new_point);
            if (active_tree1) {
                if (system.validSegment(points1[closest_other_index], new_point)) {
                    if (distance->getDistance(points1[closest_other_index], new_point, system.getWrapping()) < max_extend) {
                        start_link = closest_other_index;
                        end_link = closest_index;
                    }
                }
            }
            else {
                if (system.validSegment(new_point, points2[closest_other_index])) {
                    if (distance->getDistance(new_point, points2[closest_other_index], system.getWrapping()) < max_extend) {
                        start_link = closest_index;
                        end_link = closest_other_index;
                    }
                }
            }
            break;
        }
    }

private:
    unsigned int start_link = root_index;
    unsigned int end_link = root_index;
    unsigned int step_attempts = 100;
    std::vector<std::vector<double>> points1;
    std::vector<std::vector<double>> points2;
    std::vector<unsigned int> parent_index1;
    std::vector<unsigned int> parent_index2;
    bool active_tree1 = true;


    System &system;
    double max_extend;
    std::unique_ptr<SamplingStrategy> sampling;
    std::unique_ptr<DistanceStrategy> distance;


    [[nodiscard]] unsigned int closestActiveIndex(const std::vector<double> &point) const {
        if (point.size() != points1[0].size()) throw std::invalid_argument("Size Mismatch (BI_RRT Internal 1)");
        unsigned int closest_index = 0;

        if (active_tree1) {
            double closest_distance = distance->getDistance(point, points1[0], system.getWrapping());
            for (unsigned int i = 1; i < points1.size(); i++) {
                if (const double try_distance = distance->getDistance(point, points1[i], system.getWrapping()); try_distance < closest_distance) {
                    closest_index = i;
                    closest_distance = try_distance;
                }
            }
        }
        else {
            double closest_distance = distance->getDistance(points2[0], point, system.getWrapping());
            for (unsigned int i = 1; i < points2.size(); i++) {
                if (const double try_distance = distance->getDistance(points2[i], point, system.getWrapping()); try_distance < closest_distance) {
                    closest_index = i;
                    closest_distance = try_distance;
                }
            }
        }

        return closest_index;
    }
    void limitDistance(std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size() || start.size() != points1[0].size()) throw std::invalid_argument("Size Mismatch (BI_RRT Internal 2)");

        const double start_distance = active_tree1? distance->getDistance(start, end, system.getWrapping()) : distance->getDistance(end, start, system.getWrapping());
        if (start_distance <= max_extend) return;

        for (unsigned int i = 0; i < start.size(); i++) start[i] = end[i] + (start[i] - end[i]) * (max_extend / start_distance);
    }
};

class RRT_Connect {
public:
    RRT_Connect(System &system, const std::vector<double> &start, const std::vector<double> &end, const double max_extend, SamplingStrategy* sampling, DistanceStrategy* distance_strategy) : system(system), max_extend(max_extend), sampling(sampling), distance(distance_strategy) {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (RRT_Connect)");
        if (!sampling || !distance_strategy) throw std::invalid_argument("Sampling or Distance Strategy missing (RRT_Connect)");
        points1.push_back(start);
        parent_index1.push_back(root_index);
        points2.push_back(end);
        parent_index2.push_back(root_index);
    }

    [[nodiscard]] bool endFound() const {
        return start_link != root_index;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree1() const {
        return points1;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree2() const {
        return points2;
    }
    [[nodiscard]] std::vector<std::vector<double>> getPath() const {
        if (!endFound()) return {};
        std::vector<std::vector<double>> path;

        unsigned int current_index = start_link;
        while (current_index != root_index) {
            path.push_back(points1[current_index]);
            current_index = parent_index1[current_index];
        }
        std::reverse(path.begin(), path.end());

        current_index = end_link;
        while (current_index != root_index) {
            path.push_back(points2[current_index]);
            current_index = parent_index2[current_index];
        }

        return path;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes1() const {
        return parent_index1;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes2() const {
        return parent_index2;
    }
    [[nodiscard]] unsigned int getStartLink() const {
        return start_link;
    }
    [[nodiscard]] unsigned int getEndLink() const {
        return end_link;
    }


    void step() {
        if (start_link != root_index) return;

        for (unsigned int i = 0; i < step_attempts; i++) {
            std::vector<double> new_point = sampling->generatePoint(system.getMinBounds(), system.getMaxBounds());
            const unsigned int closest_index = closestActiveIndex(new_point);
            limitDistance(new_point, (active_tree1? points1 : points2)[closest_index]);

            if (active_tree1) {
                if (!system.validSegment(points1[closest_index], new_point)) continue;
            }
            else {
                if (!system.validSegment(new_point, points2[closest_index])) continue;
            }

            (active_tree1? points1 : points2).push_back(new_point);
            (active_tree1? parent_index1 : parent_index2).push_back(closest_index);


            active_tree1 = !active_tree1;
            unsigned int closest_other_index = closestActiveIndex(new_point);
            while (true) {//target point = new_point
                std::vector<double> extend = new_point;
                limitDistance(extend, (active_tree1? points1 : points2)[closest_other_index]);

                if (active_tree1) {
                    if (!system.validSegment(extend, points1[closest_other_index])) break;
                }
                else {
                    if (!system.validSegment(points2[closest_other_index], extend)) break;
                }

                if (new_point == extend) {
                    start_link = points1.size()-1;
                    end_link = points2.size()-1;
                    break;
                }

                (active_tree1? points1 : points2).push_back(extend);
                (active_tree1? parent_index1 : parent_index2).push_back(closest_other_index);

                closest_other_index = (active_tree1? points1 : points2).size()-1;
            }

            break;
        }
    }

private:
    unsigned int start_link = root_index;
    unsigned int end_link = root_index;
    unsigned int step_attempts = 100;
    std::vector<std::vector<double>> points1;
    std::vector<std::vector<double>> points2;
    std::vector<unsigned int> parent_index1;
    std::vector<unsigned int> parent_index2;
    bool active_tree1 = true;


    System &system;
    double max_extend;
    std::unique_ptr<SamplingStrategy> sampling;
    std::unique_ptr<DistanceStrategy> distance;


    [[nodiscard]] unsigned int closestActiveIndex(const std::vector<double> &point) const {
        if (point.size() != points1[0].size()) throw std::invalid_argument("Size Mismatch (RRT_Connect Internal 1)");
        unsigned int closest_index = 0;
        if (active_tree1) {
            double closest_distance = distance->getDistance(point, points1[0], system.getWrapping());
            for (unsigned int i = 1; i < points1.size(); i++) {
                if (const double try_distance = distance->getDistance(point, points1[i], system.getWrapping()); try_distance < closest_distance) {
                    closest_index = i;
                    closest_distance = try_distance;
                }
            }
        }
        else {
            double closest_distance = distance->getDistance(points2[0], point, system.getWrapping());
            for (unsigned int i = 1; i < points2.size(); i++) {
                if (const double try_distance = distance->getDistance(points2[i], point, system.getWrapping()); try_distance < closest_distance) {
                    closest_index = i;
                    closest_distance = try_distance;
                }
            }
        }
        return closest_index;
    }

    void limitDistance(std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size() || start.size() != points1[0].size()) throw std::invalid_argument("Size Mismatch (RRT_Connect Internal 2)");

        const double start_distance = active_tree1? distance->getDistance(start, end, system.getWrapping()) : distance->getDistance(end, start, system.getWrapping());
        if (start_distance <= max_extend) return;

        for (unsigned int i = 0; i < start.size(); i++) start[i] = end[i] + (start[i] - end[i]) * (max_extend / start_distance);

    }
};

class Informed_RRT_Star {
public:
    Informed_RRT_Star(System &system, const std::vector<double> &start, const std::vector<double> &end, const double max_extend, SamplingStrategy* sampling, DistanceStrategy* distance_strategy, CostFunction* cost) : system(system), end_point(end), max_extend(max_extend), sampling(sampling), distance(distance_strategy), cost(cost) {
        if (start.size() != end.size()) throw std::invalid_argument("Size Mismatch (Informed RRT*)");
        if (!sampling || !distance_strategy || !cost) throw std::invalid_argument("Sampling, Distance or Cost Strategy missing (Informed RRT*)");
        points.push_back(start);
        parent_index.push_back(root_index);
        costs.push_back(0);

        start_end_center.resize(end_point.size());
        for (unsigned int i = 0; i < end_point.size(); i++) {
            start_end_center[i] = (points[0][i] + end_point[i]) / 2.0;
            straight_distance += (end_point[i] - points[0][i]) * (end_point[i] - points[0][i]);
        }
        straight_distance = std::sqrt(straight_distance);


        std::vector<double> start_end_direction(end_point.size());
        double direction_length_squared = 0.0;

        for (unsigned int i = 0; i < end_point.size(); i++) {
            start_end_direction[i] = end_point[i] - points[0][i];
            direction_length_squared += start_end_direction[i] * start_end_direction[i];
        }

        const double length = std::sqrt(direction_length_squared);
        rotation_matrix.resize(end_point.size());
        for (unsigned int i = 0; i < end_point.size(); i++) {
            rotation_matrix[i] = std::vector<double>(end_point.size(), 0.0);
            rotation_matrix[i][i] = 1.0;
            rotation_matrix[0][i] = start_end_direction[i] / length;// [0] is the major axis
        }

        //Modified Gram–Schmidt
        for (unsigned int i = 1; i < end_point.size(); i++) {
            for (unsigned int j = 0; j < i; j++) {
                double dot_product = 0.0;
                for (unsigned int k = 0; k < end_point.size(); k++) {
                    dot_product += rotation_matrix[i][k] * rotation_matrix[j][k];
                }
                for (unsigned int k = 0; k < end_point.size(); k++) {
                    rotation_matrix[i][k] -= dot_product * rotation_matrix[j][k];
                }
            }


            double length_sq = 0.0;
            for (unsigned int j = 0; j < end_point.size(); j++) {
                length_sq += rotation_matrix[i][j] * rotation_matrix[i][j];
            }

            const double length_k = std::sqrt(length_sq);
            for (unsigned int j = 0; j < end_point.size(); j++) {
                rotation_matrix[i][j] /= length_k;
            }
        }
    }

    [[nodiscard]] bool endFound() const {
        return end_found_index != root_index;
    }
    [[nodiscard]] std::vector<std::vector<double>> getTree() const {
        return points;
    }
    [[nodiscard]] std::vector<std::vector<double>> getPath() const {
        if (!endFound()) return {};
        std::vector<std::vector<double>> path;
        unsigned int current_index = end_found_index;
        while (current_index != root_index) {
            path.push_back(points[current_index]);
            current_index = parent_index[current_index];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    [[nodiscard]] std::vector<double> getPathCosts() const {
        if (!endFound()) return {};
        std::vector<double> path_costs;
        unsigned int current_index = end_found_index;
        while (current_index != root_index) {
            path_costs.push_back(costs[current_index]);
            current_index = parent_index[current_index];
        }
        std::reverse(path_costs.begin(), path_costs.end());
        return path_costs;
    }
    [[nodiscard]] std::vector<double> getCosts() const {
        return costs;
    }
    [[nodiscard]] std::vector<unsigned int> getParentIndexes() const {
        return parent_index;
    }
    [[nodiscard]] std::vector<double> getEndPoint() const {
        return end_point;
    }


    void step() {
        for (unsigned int i = 0; i < step_attempts; i++) {
            std::vector<double> new_point(end_point.size());
            if (end_found_index != root_index) {
                new_point = informedSampling();
                std::vector<double> min = system.getMinBounds();
                std::vector<double> max = system.getMaxBounds();
                bool outside = false;
                for (unsigned int j = 0; j < end_point.size(); j++) {
                    if (new_point[j] < min[j] || new_point[j] > max[j]) {
                        outside = true;
                        break;
                    }
                }
                if (outside) continue;
            }
            else {
                new_point = sampling->generatePoint(system.getMinBounds(), system.getMaxBounds());
            }


            const unsigned int closest_index = closestIndex(new_point);
            limitDistance(new_point, points[closest_index]);

            if (!system.validSegment(points[closest_index], new_point)) continue;


            std::vector<unsigned int> neighbour_indexes = findNeighbors(new_point);
            neighbour_indexes.push_back(closest_index);
            const unsigned int best_parent = findBestParent(new_point, neighbour_indexes);


            points.push_back(new_point);
            parent_index.push_back(best_parent);
            costs.push_back(costs[best_parent] + cost->calculateCost(new_point, points[best_parent]));


            rewireNetwork(points.size()-1, neighbour_indexes);
            if (distance->getDistance(new_point, end_point, system.getWrapping()) < max_extend && (end_found_index == root_index || cost->calculateCost(new_point, end_point) + costs[points.size()-1] < costs[end_found_index])) {
                if (system.validSegment(points[points.size()-1], end_point)) {
                    parent_index.push_back(points.size()-1);
                    costs.push_back(cost->calculateCost(new_point, end_point) + costs[points.size()-1]);
                    points.push_back(end_point);
                    end_found_index = points.size()-1;
                }
            }
            return;
        }
    }



private:
    unsigned int end_found_index = root_index;
    unsigned int step_attempts = 100;
    std::vector<std::vector<double>> points;
    std::vector<unsigned int> parent_index;
    std::vector<double> costs;

    std::vector<double> start_end_center;
    double straight_distance = 0;
    std::vector<std::vector<double>> rotation_matrix;


    System &system;
    std::vector<double> end_point;
    double max_extend;
    std::unique_ptr<SamplingStrategy> sampling;
    std::unique_ptr<DistanceStrategy> distance;
    std::unique_ptr<CostFunction> cost;


    [[nodiscard]] unsigned int closestIndex(const std::vector<double> &point) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (Informed_RRT* Internal 1)");
        unsigned int closest_index = 0;
        double closest_distance = distance->getDistance(point, points[0], system.getWrapping());
        for (unsigned int i = 1; i < points.size(); i++) {
            if (const double try_distance = distance->getDistance(point, points[i], system.getWrapping()); try_distance < closest_distance) {
                closest_index = i;
                closest_distance = try_distance;
            }
        }
        return closest_index;
    }

    void limitDistance(std::vector<double> &start, const std::vector<double> &end) const {
        if (start.size() != end.size() || start.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (Informed_RRT* Internal 2)");
        const double start_distance = distance->getDistance(start, end, system.getWrapping());
        if (start_distance <= max_extend) return;

        for (unsigned int i = 0; i < start.size(); i++)
            start[i] = end[i] + (start[i] - end[i]) * (max_extend / start_distance);

    }

    [[nodiscard]] std::vector<unsigned int> findNeighbors(const std::vector<double> &point) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (Informed_RRT* Internal 3)");
        std::vector<unsigned int> neighbours;

        const double gamma = 2*max_extend;
        double radius = gamma; //gamma * std::sqrt(std::log(points.size()) / static_cast<double>(points.size()));
        radius = std::min(radius, max_extend);


        for (unsigned int i = 0; i < points.size(); i++) {
            if (distance->getDistance(point, points[i], system.getWrapping()) < radius && system.validSegment(point, points[i])) {
                neighbours.push_back(i);
            }
        }
        return neighbours;
    }

    [[nodiscard]] unsigned int findBestParent(const std::vector<double> &point, const std::vector<unsigned int> &neighbour_indexes) const {
        if (point.size() != end_point.size()) throw std::invalid_argument("Size Mismatch (Informed_RRT* Internal 4)");
        if (neighbour_indexes.empty()) throw std::invalid_argument("Empty Input (Informed_RRT* Internal 4)");
        unsigned int best_parent = neighbour_indexes[0];
        double current_cost = cost->calculateCost(point, points[neighbour_indexes[0]]) + costs[neighbour_indexes[0]];

        for (unsigned int i = 1; i < neighbour_indexes.size(); i++) {
            if (system.validSegment(point, points[neighbour_indexes[i]])) {
                double posable_cost = cost->calculateCost(point, points[neighbour_indexes[i]]) + costs[neighbour_indexes[i]];
                if (current_cost > posable_cost) {
                    best_parent = neighbour_indexes[i];
                    current_cost = posable_cost;
                }
            }
        }
        return best_parent;
    }

    void rewireNetwork(const unsigned int point_index, const std::vector<unsigned int> &neighbour_indexes) {
        if (point_index >= points.size()) throw std::invalid_argument("Index Error (RRT* Internal 5)");
        if (neighbour_indexes.empty()) throw std::invalid_argument("Empty Input (RRT* Internal 5)");
        for (const unsigned int neighbour_index : neighbour_indexes) {
            if (neighbour_index == point_index) continue;
            if (!system.validSegment(points[point_index], points[neighbour_index])) continue;

            const double potential_cost = cost->calculateCost(points[point_index], points[neighbour_index]) + costs[point_index];
            if (potential_cost < costs[neighbour_index]) {
                parent_index[neighbour_index] = point_index;
                costs[neighbour_index] = potential_cost;
                updateDescendantCosts(neighbour_index);
            }
        }
    }

    void updateDescendantCosts(const unsigned int point_index) {
        if (point_index >= points.size()) throw std::invalid_argument("Index Error (Informed_RRT* Internal 6)");
        for (unsigned int i = 0; i < points.size(); i++) {
            if (parent_index[i] == point_index) {
                costs[i] = cost->calculateCost(points[point_index], points[i]) + costs[point_index];
                updateDescendantCosts(i);
            }
        }
    }

    [[nodiscard]] std::vector<double> informedSampling() const {
        std::vector<double> result = unitHyperSphereSample();

        const double major_axis = costs[end_found_index] / 2.0;
        const double minor_axis = std::sqrt(costs[end_found_index] * costs[end_found_index] - straight_distance * straight_distance) / 2.0;

        result[0] *= major_axis;
        for (unsigned int i = 1; i < end_point.size(); i++) result[i] *= minor_axis;


        std::vector<double> rotated(end_point.size());
        for (unsigned int i = 0; i < end_point.size(); i++) {
            rotated[i] = 0.0;
            for (unsigned int k = 0; k < end_point.size(); k++) {
                rotated[i] += rotation_matrix[k][i] * result[k];
            }
        }

        for (unsigned int i = 0; i < end_point.size(); i++) rotated[i] += start_end_center[i];
        return rotated;
    }

    [[nodiscard]] std::vector<double> unitHyperSphereSample() const {
        std::vector<double> result(end_point.size());
        const double radius = std::pow(rand() / static_cast<double>(RAND_MAX), 1.0 / static_cast<double>(end_point.size()));

        std::vector<double> direction(end_point.size());
        double length_squared = 0.0;
        for (unsigned int i = 0; i < end_point.size(); i++) {
            direction[i] = (rand() / static_cast<double>(RAND_MAX)) * 2.0 - 1.0;
            length_squared += direction[i] * direction[i];
        }
        for (unsigned int i = 0; i < end_point.size(); i++) {
            result[i] = radius * direction[i] / std::sqrt(length_squared);
        }

        return result;
    }
};




#endif //RRT_LIBRARY_RRT_H