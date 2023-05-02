//
// Created by rocky on 2023/4/28.
//
#include <algorithm>
#include <cmath>
#include <utility>
#include "GraphSearch.hpp"


GraphSearch::Plan::Plan() {
    direction = {{1, 0}, {0, 1}, {-1, 0}, {0, -1},
                 {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

    obstacle.clear();


}

void GraphSearch::Plan::setMapSize(GraphSearch::Coordinate coordinate_) {
    mapSize = coordinate_;

}

void GraphSearch::Plan::setObstacle(GraphSearch::coordinateSet newObstacle_) {
    obstacle = std::move(newObstacle_);

}

bool GraphSearch::Plan::isCollision(GraphSearch::Coordinate coordinate_) {
    return (coordinate_.x < 0 || coordinate_.x > mapSize.x ||
            coordinate_.y < 0 || coordinate_.y > mapSize.y ||
            std::find(obstacle.begin(), obstacle.end(), coordinate_)!=obstacle.end());
}

GraphSearch::Node *
GraphSearch::Plan::findNodeInSet(GraphSearch::nodeSet &nodeSet_, GraphSearch::Coordinate coordinate_) {
    for (auto node: nodeSet_) {
        if (node->coordinate == coordinate_)
            return node;
    }
    return nullptr;
}

double GraphSearch::Plan::getH(GraphSearch::Coordinate current_, GraphSearch::Coordinate target_) {

    Coordinate delta = {abs(current_.x - target_.x), abs(current_.y - target_.y)};

    return 10*sqrt(pow(delta.x, 2) + pow(delta.y, 2));
}

GraphSearch::coordinateSet GraphSearch::Plan::getPath(GraphSearch::Coordinate start_, GraphSearch::Coordinate target_) {

    openSet.clear();
    closedSet.clear();

    target = target_;
    openSet.push_back(new Node(start_));
    while (!openSet.empty())
    {
        auto currentIt = openSet.begin();
        current = *currentIt;

        // get the best
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
           auto node = *it;
           if (node->getF() < current->getF())
           {
               current = node;
               currentIt = it;
           }
        }
        // break
        if (current->coordinate == target)
            break;

        //
        closedSet.push_back(current);
        openSet.erase(currentIt);

        // update open set
        updateOpenSet();


    }
    // output
    coordinateSet path;

    while (current != nullptr)
    {
        path.push_back(current->coordinate);
        current = current->parent;
    }
    return path;
}

void GraphSearch::AStar::updateOpenSet() {
    for (uint i = 0; i < 8; ++i) {
        // around
        Coordinate newCoordinate(current->coordinate + direction.at(i));
        if (isCollision(newCoordinate) || findNodeInSet(closedSet, newCoordinate))
            continue;
        double totalCost = current->g + ((i < 4) ? 10 : 14);

        Node* successor = findNodeInSet(openSet, newCoordinate);
        if (successor == nullptr)
        {
            successor = new Node(newCoordinate, current);
            successor->g = totalCost;
            successor->h = getH(newCoordinate, target);
            openSet.push_back(successor);

        }else if (totalCost < successor->g)
        {
            successor->parent = current;
            successor->g = totalCost;
        }




    }
}

void GraphSearch::ThetaStar::updateOpenSet() {
    for (uint i = 0; i < 8; ++i)
    {
        Coordinate newCoordinate(current->coordinate + direction.at(i));
        if (isCollision((newCoordinate)) || findNodeInSet(closedSet, newCoordinate))
            continue;
        // 删除对角范围内
        if(i > 3)
        {
            if (std::find(obstacle.begin(), obstacle.end(), Coordinate({current->coordinate.x, newCoordinate.y}))!=obstacle.end())
                continue;
            if (std::find(obstacle.begin(), obstacle.end(), Coordinate({newCoordinate.x, current->coordinate.x}))!=obstacle.end())
                continue;
        }

        Node* successor = findNodeInSet(openSet, newCoordinate);
        if (successor == nullptr)
        {
            successor = new Node(newCoordinate, current);
            successor->h = getH(newCoordinate, target);
            successor->g = current->g + ((i < 4)? 10:14);
            openSet.push_back(successor);
        } else if (current->g + ((i < 4)? 10:14) < successor->g)
        {
            successor->parent = current;
            successor->g = current->g + ((i < 4)? 10:14);
        }
        // 根据视线确定父节点
        if (current->coordinate != closedSet.at(0)->coordinate)
            if(lineOfSight(successor->coordinate, current->parent->coordinate))
            {
                successor->g = getH(successor->coordinate, current->parent->coordinate);
                successor->parent = current->parent;
            }

    }

}

bool GraphSearch::ThetaStar::lineOfSight(GraphSearch::Coordinate current_, GraphSearch::Coordinate target_) {
    int dx = abs(target_.x - current_.x);
    int dy = abs(target_.y - current_.y);
    int x = current_.x;
    int y = current_.y;

    int x_inc = (target_.x > current_.x) ? 1 : -1;
    int y_inc = (target_.y > current_.y) ? 1 : -1;
    int error = dx - dy;

    dx *= 2;
    dy *= 2;

    while (x != target_.x || y != target_.y)
    {
        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
        if (std::find(obstacle.begin(), obstacle.end(), Coordinate({x, y})) != obstacle.end())
            return false;
    }

    return true;
}

void GraphSearch::SafeA::updateOpenSet() {
    for (uint i = 0; i < 8; ++i) {
        // around
        Coordinate newCoordinate(current->coordinate + direction.at(i));
        if (isCollision(newCoordinate) || findNodeInSet(closedSet, newCoordinate))
            continue;
        if(i > 3)
        {
            if (std::find(obstacle.begin(), obstacle.end(), Coordinate({current->coordinate.x, newCoordinate.y}))!=obstacle.end())
                continue;
            if (std::find(obstacle.begin(), obstacle.end(), Coordinate({current->coordinate.y, newCoordinate.x}))!=obstacle.end())
                continue;
        }
        double totalCost = current->g + ((i < 4) ? 10 : 14);

        Node *successor = findNodeInSet(openSet, newCoordinate);
        if (successor == nullptr) {
            successor = new Node(newCoordinate, current);
            successor->g = totalCost;
            successor->h = getH(newCoordinate, target);
            openSet.push_back(successor);

        } else if (totalCost < successor->g) {
            successor->parent = current;
            successor->g = totalCost;
        }


    }
}