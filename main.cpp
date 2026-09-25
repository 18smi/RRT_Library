#include <iostream>
#include "RRT.h"
#include "RRT_Visualizer.h"
#include "Path_Refinement.h"


int main() {
    sf::RenderWindow window(sf::VideoMode({500, 500}), "RRT display");//, sf::Style::Fullscreen

    System sys({0, 0, 0}, {360, 360, 360}, {true, false, true}, std::make_unique<Simple3JointArm>(30, 20, 2), std::make_unique<LinearPath>(), {.constraint_safety_margin = 0, .obstacle_safety_margin = 2, .interpolation_steps = 10});
    sys.addConstraint(std::make_unique<DrawableHyperRectangle>(std::vector<double>{0, 0, 170}, std::vector<double>{360, 360, 190}));// stops the arm hitting itself
    sys.addObstacle(std::make_unique<DrawableCuboid>(std::array<double, 3>{0, 0, 0}, std::array<double, 3>{100, 100, 100}, std::array<double, 3>{0, 0, 0}));// platform the arm is on

    //workspace obstacles

    std::vector<double> start_point = Simple3JointArm::PositionToPoint(std::array<double, 3>{0, 20, 30}, 30, 20);
    std::vector<double> end_point = Simple3JointArm::PositionToPoint(std::array<double, 3>{10, 0.5, -30}, 30, 20);
    for (unsigned int i = 0; i < 3; i++) {
        start_point[i] *= 360 / std::numbers::pi;
        end_point[i] *= 360 / std::numbers::pi;
        start_point[i] = fmod(start_point[i] + 360,  360);
        end_point[i] = fmod(end_point[i] + 360,  360);
        std::cout << start_point[i]  << ", " << end_point[i] << std::endl;
    }// converts radians into degrees

    RRT alg(sys, start_point, end_point, 0.1, std::make_unique<BiasedSampling>(end_point, 0.1) , std::make_unique<WeightedEuclidianDistance>(std::vector<double>{1, 2, 2}));
    RRT_Visualiser vis(window, sys);

    vis.setGeometricBounds({-50, -50, 0}, {50, 50, 50});


    Simple3JointArmPositionFromPoint position_from_point(30, 20);
    sf::Clock clock;
    while (window.isOpen()) {
        while (std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            if (auto key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape) {
                    window.close();
                }
            }
        }


        if (clock.getElapsedTime().asMilliseconds() > 10) {
            vis.camProjectDraw(alg, position_from_point, 0, 0, 0);
            //vis.drawFlat(alg, 1, 2);
            alg.step();
            clock.restart();
        }
    }

    std::vector<std::vector<double>> path = alg.getPath();
    for (auto & i : path) {
        std::cout << "(" << i[0];
        for (int j = 1; j < i.size(); j++) {
            std::cout << ", " << i[j];
        }
        std::cout << ")\n";
    }


    std::cout << "\n\nSmoothed Path\n";
    const std::vector<std::vector<double>> smoothed = refine::greedyShortcutting(path, sys);
    for (auto & i : smoothed) {
        std::cout << "(" << i[0];
        for (int j = 1; j < i.size(); j++) {
            std::cout << ", " << i[j];
        }
        std::cout << ")\n";
    }

    return 0;
}