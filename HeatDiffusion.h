#ifndef HEATDIFFUSION_H
#define HEATDIFFUSION_H

#include <vector>
#include <glm/glm.hpp>

// Forward declarations
struct Vertex;
struct Bone;
struct Edge;

class HeatDiffusion {
public:
    // Compute bone weights using Heat Diffusion method
    static void computeBoneWeights(
        std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices,
        const std::vector<std::vector<Edge>>& adjacency,
        std::vector<Bone>& skeleton,
        float timeStep = 0.01f
    );

private:
    // Helper: Build cotangent Laplacian matrix (stored as adjacency list for simplicity)
    static std::vector<std::vector<std::pair<int, float>>> buildCotangentLaplacian(
        const std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices
    );

    // Helper: Solve heat diffusion equation using iterative method
    static std::vector<float> solveHeatDiffusion(
        const std::vector<Vertex>& vertices,
        const std::vector<std::vector<std::pair<int, float>>>& laplacian,
        int sourceVertex,
        float timeStep,
        int iterations = 100
    );

    // Helper: Compute cotangent of angle at vertex
    static float cotangent(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
};

#endif // HEATDIFFUSION_H
