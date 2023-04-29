#include <iostream>
#include "GraphSearch.hpp"
int main() {

    GraphSearch::Coordinate mapSize = {9,9};
    GraphSearch::coordinateSet obstacle = {{3, 3}, {4, 4}};
    GraphSearch::Coordinate start = {2, 2};
    GraphSearch::Coordinate target = {9, 9};
    GraphSearch::AStar plan;
    plan.setMapSize(mapSize);
    plan.setObstacle(obstacle);

    GraphSearch::coordinateSet path = plan.getPath(start, target);

    for (auto coor: path) {
        std::cout << coor.x << "," << coor.y << std::endl;
    }

    GraphSearch::ThetaStar plan2;
    plan2.setMapSize(mapSize);
    plan2.setObstacle(obstacle);
    GraphSearch::coordinateSet path2 = plan2.getPath(start, target);
    for (auto coor2: path2) {
        std::cout << coor2.x << "," << coor2.y << std::endl;

    }
    return 0;
}
