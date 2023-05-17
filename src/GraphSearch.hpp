//
// Created by rocky on 2023/4/28.
//

#ifndef GRAPHSEARCH_GRAPHSEARCH_HPP
#define GRAPHSEARCH_GRAPHSEARCH_HPP
#include <vector>

namespace GraphSearch {


    using uint = unsigned int;
    /**
     * @brief 用于存储坐标点的结构体，覆写了+，+，！=运算符
     */
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
    /**
     * @brief 用于存储节点的结构体，包含g，h值，以及父节点
     */
    struct Node{
        Coordinate coordinate{};
        Node *parent;
        double g, h;
        /**
         * @brief 需要使用坐标点构造实例，默认父节点为空，g,h为0
         * @param coordinate_
         * @param parent_
         */
        explicit Node(Coordinate coordinate_, Node* parent_ = nullptr)
        {
            coordinate = coordinate_;
            parent = parent_;
            g = 0;
            h = 0;
        }
        double getF() const
        {
            return g + h;
        }
    };
    /**
     * @brief 存储栅格图结构
     */
     struct GridMap
     {
         /**
          * @brief 将char[]类型的数据转换为map，char[]中1为true
          * @param mapList_ unsigned char[] 一维向量存储的代价图
          * @param x_ 横坐标边界
          * @param y_ 纵坐标边界
          */
         GridMap(const unsigned char* mapList_, size_t x_, size_t y_ ) {
             size_t i = 0;
             std::vector<bool> tmp(y_, false);
             while (i++ < x_)
             {
                 for (uint j = 0; j < y_; ++j)
                 {
                     if ( *(mapList_ + (i-1)*y_ + j) == 1)
                         tmp[j] = true;
                     else
                         tmp[j] = false;
                 }
                map.push_back(tmp);
             }
         }
         bool isCollision(size_t x_, size_t y_) {
             return map.at(x_ - 1).at(y_ -1);
         }
         Coordinate getMapSize() {
             Coordinate mapSize = {static_cast<int>(map.size()), static_cast<int>(map.at(0).size())};
             return mapSize;
         }
         std::vector<std::vector<bool>> getMap() {
             return map;
         }

     private:
         std::vector<std::vector<bool>> map;
     };
    // 存储一系列坐标点
    using coordinateSet = std::vector<Coordinate>;
    // 存储一系列节点
    using nodeSet = std::vector<Node*>;
   /**
    * @brief 图搜索规划类的基类，其中更新开集列表为纯虚函数，因此需要继承并重写后才能使用
    */
    class Plan{
    private:
        Coordinate mapSize{};
    protected:
        Coordinate target{};
        coordinateSet obstacle, direction;
        Node * current{};
        nodeSet openSet, closedSet;

        virtual bool isCollision (Coordinate coordinate_);
        static Node* findNodeInSet(nodeSet &nodeSet_, Coordinate coordinate_);
        static double getH(Coordinate current_, Coordinate target_);
        virtual void updateOpenSet() = 0;
    public:
         Plan();
         void setMapSize(Coordinate coordinate_);

        virtual void setObstacle(coordinateSet newObstacle_);
         coordinateSet getPath(Coordinate start_, Coordinate target_);

    };
    /**
     * @brief 使用栅格图存储障碍物并进行图搜索的算法的基类
     */
    class PlanGrid : public Plan
    {
    private:
        Coordinate mapSize{};
        coordinateSet obstacle;
        void setMapSize(){};
        void setObstacle(coordinateSet newObstacle_) override{ };
    protected:
        GridMap *map;
        bool isCollision (Coordinate coordinate_) override;
    public:
        PlanGrid();
        void setMap(GridMap* map_);
    };

    /**
     * @brief A*算法的实现
     */
    class AStar: public Plan
    {
    protected:
         void updateOpenSet() override;
    public:

    };
    /**
     * @brief A*算法的实现-基于栅格图存储障碍物
     */
     class AStarGrid: public PlanGrid
     {
     protected:
         void updateOpenSet() override;
     };
    /**
     * @brief theta*算法的实现
     */
    class ThetaStar: public Plan
    {
    private:
        bool lineOfSight(Coordinate current_, Coordinate target_);
    public:
        static bool lineOfSightOpen(Coordinate current_, Coordinate target_, GraphSearch::coordinateSet obstacle_);
    protected:
        void updateOpenSet() override;
    };
    /**
     * @brief theta*算法的实现-基于栅格图存储的障碍物
     */
    class ThetaStarGrid: public PlanGrid
    {
    private:
        bool lineOfSight(Coordinate current_, Coordinate target_);
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
