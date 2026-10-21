#ifndef EUCLIDEAN_CLUSTERING_H_
#define EUCLIDEAN_CLUSTERING_H_

#include <cstddef>
#include <stack>
#include <vector>

#include "kdtree.h"

inline std::vector<std::vector<int>> euclideanCluster3D(
    const std::vector<KdTree3D::Point>& points,
    const KdTree3D& tree,
    float distanceTolerance)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (std::size_t seed = 0; seed < points.size(); ++seed)
    {
        if (processed[seed])
        {
            continue;
        }

        std::vector<int> cluster;
        std::stack<int> pending;
        pending.push(static_cast<int>(seed));
        processed[seed] = true;

        while (!pending.empty())
        {
            const int current = pending.top();
            pending.pop();
            cluster.push_back(current);

            const std::vector<int> nearby =
                tree.search(points[current], distanceTolerance);
            for (int neighbor : nearby)
            {
                if (!processed[neighbor])
                {
                    processed[neighbor] = true;
                    pending.push(neighbor);
                }
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}

#endif  // EUCLIDEAN_CLUSTERING_H_
