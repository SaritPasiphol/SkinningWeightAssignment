#ifndef WEIGHT_OPTIMIZATION_H
#define WEIGHT_OPTIMIZATION_H

#include <vector>
#include <glm/glm.hpp>

// Forward declarations
struct Vertex;
struct Bone;
struct Edge;

namespace WeightOptimization {
    // Prune weights to top N bones per vertex with simple normalization
    void pruneWeights(
        std::vector<Vertex>& vertices,
        const std::vector<Bone>& skeleton,
        int maxBonesPerVertex = 4
    );
    
    // Smooth weights with neighbor vertices (constrained)
    void smoothWeights(
        std::vector<Vertex>& vertices,
        const std::vector<std::vector<Edge>>& adjacency,
        const std::vector<Bone>& skeleton,
        int iterations = 2
    );
}

#endif // WEIGHT_OPTIMIZATION_H
