#include <iostream>
#include "RRT.h"
#include "RRT_Visualizer.h"
#include "Path_Refinement.h"



int main() {
    sf::RenderWindow window(sf::VideoMode({500, 500}), "RRT display");//, sf::Style::Fullscreen

    System sys({0, 0, -0.9*std::numbers::pi}, {std::numbers::pi, 2*std::numbers::pi, 0.9*std::numbers::pi}, {false, false, false}, new Simple3JointArm(30, 20, 2), new LinearPath, {.constraint_safety_margin = 0, .obstacle_safety_margin = 2, .interpolation_steps = 10});
    //sys.addConstraint



    RRT alg(sys, {0, 0.5*std::numbers::pi, 1.5*std::numbers::pi}, {0.5*std::numbers::pi, 0.2*std::numbers::pi, 1.8*std::numbers::pi}, (1.0/90)*std::numbers::pi, new StochasticSampling, new WeightedEuclidianDistance({1, 2, 2}));
    RRT_Visualiser vis(window, sys);

    vis.setGeometricBounds({-50, -50, 0}, {50, 50, 50});


    Simple3JointArmEffector position_from_point(30, 20);
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
            //vis.drawFlat(alg, 0, 1);
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