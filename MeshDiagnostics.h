#ifndef MESHDIAGNOSTICS_H
#define MESHDIAGNOSTICS_H

#include <vector>
#include <map>
#include <set>
#include <glm/glm.hpp>

struct Vertex;

struct MeshQuality {
    bool isManifold;           // True if all edges connect exactly 2 triangles
    bool hasBoundaryEdges;     // True if mesh has holes (edges with only 1 triangle)
    bool hasSelfIntersections; // True if triangles intersect each other
    int nonManifoldEdges;      // Count of edges with != 2 triangles
    int boundaryEdges;         // Count of edges with only 1 triangle
    int selfIntersectingPairs; // Count of intersecting triangle pairs
    
    void print() const;
};

class MeshDiagnostics {
public:
    // Main diagnostic function
    static MeshQuality analyzeMesh(
        const std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices,
        bool checkSelfIntersections = true  // Can be slow for large meshes
    );

private:
    // Helper: Check if edge connects exactly 2 triangles (manifold)
    static std::map<std::pair<int, int>, int> buildEdgeMap(
        const std::vector<unsigned int>& indices
    );
    
    // Helper: Check if two triangles intersect
    static bool trianglesIntersect(
        const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
        const glm::vec3& u0, const glm::vec3& u1, const glm::vec3& u2
    );
    
    // Helper: Check if a ray intersects a triangle
    static bool rayIntersectsTriangle(
        const glm::vec3& rayOrigin, const glm::vec3& rayDir,
        const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
        float& t
    );
};

#endif // MESHDIAGNOSTICS_H
