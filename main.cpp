#include <iostream>
#include "GraphSearch.hpp"
int main() {

    GraphSearch::Coordinate mapSize = {9,9};
    GraphSearch::coordinateSet obstacle = {{3, 3}, {4, 4}};
    GraphSearch::Coordinate start = {2, 2};
    GraphSearch::Coordinate target = {9, 9};


    // AStar
    GraphSearch::AStar plan;        // 实例化对象
    plan.setMapSize(mapSize);   // 设置地图大小
    plan.setObstacle(obstacle);     //添加障碍物列表
    GraphSearch::coordinateSet path = plan.getPath(start, target);      // 获取路径
    std::cout<<"AStar:"<<std::endl;
    for (auto coor: path) {
        std::cout << coor.x << "," << coor.y << std::endl;
    }
    // Theta Star
    GraphSearch::ThetaStar plan2;
    plan2.setMapSize(mapSize);
    plan2.setObstacle(obstacle);
    GraphSearch::coordinateSet path2 = plan2.getPath(start, target);
    std::cout<<"ThetaStar:"<<std::endl;
    for (auto coor2: path2) {
        std::cout << coor2.x << "," << coor2.y << std::endl;
    }
    return 0;
}
