//
// Created by rocky on 2023/4/28.
//

#ifndef GRAPHSEARCH_GRAPHSEARCH_HPP
#define GRAPHSEARCH_GRAPHSEARCH_HPP
#include <vector>

namespace GraphSearch {
// struct
    using uint = unsigned int;

    struct Coordinate{
        int x, y;
        bool operator == (const Coordinate coordinate_) const
        {
            if (x == coordinate_.x && y == coordinate_.y)
                return true;
            else
                return false;
        }
        bool operator != (const Coordinate coordinate_) const
        {
            if (x == coordinate_.x && y == coordinate_.y)
                return false;
            else
                return true;
        }
        Coordinate operator + (const Coordinate coordinate_) const
        {
            return {x + coordinate_.x, y + coordinate_.y};
        }
    };

    struct Node{
        Coordinate coordinate;
        Node *parent;
        double g, h;

        explicit Node(Coordinate coordinate_, Node* parent_ = nullptr)
        {
            coordinate = coordinate_;
            parent = parent_;
            g = 0;
        }
        double getF()
        {
            return g + h;
        }
    };

    using coordinateSet = std::vector<Coordinate>;
    using nodeSet = std::vector<Node*>;
// base
    class Plan{
    private:
        Coordinate mapSize{};
    protected:
        Coordinate target{};
        coordinateSet obstacle, direction;
        Node * current{};
        nodeSet openSet, closedSet;

        bool isCollision (Coordinate coordinate_);
        static Node* findNodeInSet(nodeSet &nodeSet_, Coordinate coordinate_);
        static double getH(Coordinate current_, Coordinate target_);
        virtual void updateOpenSet() = 0;

    public:
         Plan();
         void setMapSize(Coordinate coordinate_);
         void setObstacle(coordinateSet newObstacle_);
         coordinateSet getPath(Coordinate start_, Coordinate target_);

    };
// AStar
    class AStar: public Plan
    {
    protected:
         void updateOpenSet() override;
    public:

    };
// ThetaStar
    class ThetaStar: public Plan
    {
    private:
        bool lineOfSight(Coordinate current_, Coordinate target_);
    public:
        static bool lineOfSightOpen(Coordinate current_, Coordinate target_, GraphSearch::coordinateSet obstacle_);
    protected:
        void updateOpenSet() override;
    };
// SafeA
    class SafeA : public Plan
    {
    private:
    protected:
        void updateOpenSet() override;
    };

}


#endif //GRAPHSEARCH_GRAPHSEARCH_HPP
