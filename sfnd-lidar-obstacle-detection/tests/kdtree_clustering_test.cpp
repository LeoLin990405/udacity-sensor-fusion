#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "euclidean_clustering.h"

namespace
{
std::vector<int> sorted(std::vector<int> values)
{
    std::sort(values.begin(), values.end());
    return values;
}

bool require(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "KD-tree test failed: " << message << std::endl;
    }
    return condition;
}
}

int main()
{
    KdTree3D tree;
    const std::vector<KdTree3D::Point> points = {
        {{0.0f, 0.0f, 0.0f}},
        {{0.5f, 0.5f, 0.5f}},
        {{0.8f, 0.8f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}},
        {{-0.5f, -0.5f, -0.5f}},
        {{5.0f, 5.0f, 5.0f}},
        {{5.4f, 5.0f, 5.0f}}
    };

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        tree.insert(points[index], static_cast<int>(index));
    }

    bool valid = true;
    valid &= require(
        sorted(tree.search({{0.0f, 0.0f, 0.0f}}, 1.0f)) ==
            std::vector<int>({0, 1, 3, 4}),
        "3D radius search returned incorrect neighbors");
    valid &= require(
        tree.search({{10.0f, 10.0f, 10.0f}}, 0.5f).empty(),
        "an empty search region returned a neighbor");

    std::vector<std::vector<int>> clusters =
        euclideanCluster3D(points, tree, 1.25f);
    for (std::vector<int>& cluster : clusters)
    {
        std::sort(cluster.begin(), cluster.end());
    }
    std::sort(clusters.begin(), clusters.end());

    valid &= require(clusters.size() == 2,
                     "Euclidean clustering returned the wrong cluster count");
    valid &= require(
        clusters.size() >= 1 &&
            clusters[0] == std::vector<int>({0, 1, 2, 3, 4}),
        "the first Euclidean cluster has incorrect members");
    valid &= require(
        clusters.size() >= 2 && clusters[1] == std::vector<int>({5, 6}),
        "the second Euclidean cluster has incorrect members");

    if (!valid)
    {
        return 1;
    }

    std::cout << "KD-tree and Euclidean clustering tests passed" << std::endl;
    return 0;
}
