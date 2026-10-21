#ifndef KDTREE_H_
#define KDTREE_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

class KdTree3D
{
public:
    using Point = std::array<float, 3>;

    void insert(const Point& point, int id)
    {
        std::unique_ptr<Node>* branch = &root_;
        std::size_t depth = 0;

        while (*branch)
        {
            const std::size_t axis = depth % dimensions;
            if (point[axis] < (*branch)->point[axis])
            {
                branch = &((*branch)->left);
            }
            else
            {
                branch = &((*branch)->right);
            }
            ++depth;
        }

        branch->reset(new Node(point, id));
    }

    std::vector<int> search(const Point& target, float distanceTolerance) const
    {
        std::vector<int> ids;
        searchNode(root_.get(), target, distanceTolerance, 0, ids);
        return ids;
    }

private:
    static const std::size_t dimensions = 3;

    struct Node
    {
        Node(const Point& setPoint, int setId)
            : point(setPoint), id(setId)
        {
        }

        Point point;
        int id;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    static void searchNode(
        const Node* node,
        const Point& target,
        float distanceTolerance,
        std::size_t depth,
        std::vector<int>& ids)
    {
        if (!node)
        {
            return;
        }

        bool insideSearchBox = true;
        float squaredDistance = 0.0f;
        for (std::size_t axis = 0; axis < dimensions; ++axis)
        {
            const float difference = node->point[axis] - target[axis];
            insideSearchBox = insideSearchBox &&
                std::fabs(difference) <= distanceTolerance;
            squaredDistance += difference * difference;
        }

        if (insideSearchBox &&
            squaredDistance <= distanceTolerance * distanceTolerance)
        {
            ids.push_back(node->id);
        }

        const std::size_t axis = depth % dimensions;
        if (target[axis] - distanceTolerance <= node->point[axis])
        {
            searchNode(
                node->left.get(), target, distanceTolerance, depth + 1, ids);
        }
        if (target[axis] + distanceTolerance >= node->point[axis])
        {
            searchNode(
                node->right.get(), target, distanceTolerance, depth + 1, ids);
        }
    }

    std::unique_ptr<Node> root_;
};

#endif  // KDTREE_H_
