#include "MeshDiagnostics.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// Need Vertex definition
struct Vertex {
    glm::vec3 originalPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    std::vector<float> boneWeights;
};

void MeshQuality::print() const {
    std::cout << "\n=== MESH QUALITY REPORT ===" << std::endl;
    std::cout << "Manifold: " << (isManifold ? "YES" : "NO") << std::endl;
    std::cout << "Has Boundary Edges (Holes): " << (hasBoundaryEdges ? "YES" : "NO") << std::endl;
    
    if (nonManifoldEdges > 0) {
        std::cout << "  ⚠ Non-manifold edges: " << nonManifoldEdges << std::endl;
    }
    if (boundaryEdges > 0) {
        std::cout << "  ⚠ Boundary edges (holes): " << boundaryEdges << std::endl;
    }
    if (hasSelfIntersections) {
        std::cout << "Self-intersections: YES (" << selfIntersectingPairs << " pairs)" << std::endl;
    } else {
        std::cout << "Self-intersections: NO" << std::endl;
    }
    
    if (isManifold && !hasBoundaryEdges && !hasSelfIntersections) {
        std::cout << "✓ Mesh is CLEAN - Surface-based heat diffusion OK" << std::endl;
    } else {
        std::cout << "✗ Mesh has DEFECTS - Consider using Voxel-based method" << std::endl;
    }
    std::cout << "========================\n" << std::endl;
}

MeshQuality MeshDiagnostics::analyzeMesh(
    const std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices,
    bool checkSelfIntersections
) {
    std::cout << "Analyzing mesh topology..." << std::endl;
    
    MeshQuality quality;
    quality.isManifold = true;
    quality.hasBoundaryEdges = false;
    quality.hasSelfIntersections = false;
    quality.nonManifoldEdges = 0;
    quality.boundaryEdges = 0;
    quality.selfIntersectingPairs = 0;
    
    // Step 1: Check manifoldness and boundary edges
    auto edgeMap = buildEdgeMap(indices);
    
    for (const auto& pair : edgeMap) {
        int count = pair.second;
        if (count == 1) {
            quality.boundaryEdges++;
            quality.hasBoundaryEdges = true;
        } else if (count > 2) {
            quality.nonManifoldEdges++;
            quality.isManifold = false;
        }
    }
    
    std::cout << "  Total unique edges: " << edgeMap.size() << std::endl;
    std::cout << "  Boundary edges: " << quality.boundaryEdges << std::endl;
    std::cout << "  Non-manifold edges: " << quality.nonManifoldEdges << std::endl;
    
    // Step 2: Check for self-intersections (expensive!)
    if (checkSelfIntersections) {
        std::cout << "  Checking self-intersections (this may take a while)..." << std::endl;
        int numTriangles = indices.size() / 3;
        
        // Only check a reasonable number for large meshes
        int maxChecks = 100000;
        int checksPerformed = 0;
        
        for (int i = 0; i < numTriangles && checksPerformed < maxChecks; ++i) {
            glm::vec3 v0 = vertices[indices[i * 3 + 0]].originalPosition;
            glm::vec3 v1 = vertices[indices[i * 3 + 1]].originalPosition;
            glm::vec3 v2 = vertices[indices[i * 3 + 2]].originalPosition;
            
            // Only check non-adjacent triangles
            for (int j = i + 2; j < numTriangles && checksPerformed < maxChecks; ++j) {
                glm::vec3 u0 = vertices[indices[j * 3 + 0]].originalPosition;
                glm::vec3 u1 = vertices[indices[j * 3 + 1]].originalPosition;
                glm::vec3 u2 = vertices[indices[j * 3 + 2]].originalPosition;
                
                // Skip if triangles share vertices
                std::set<int> verts = {
                    (int)indices[i * 3 + 0], (int)indices[i * 3 + 1], (int)indices[i * 3 + 2],
                    (int)indices[j * 3 + 0], (int)indices[j * 3 + 1], (int)indices[j * 3 + 2]
                };
                if (verts.size() < 6) continue; // Shared vertex
                
                if (trianglesIntersect(v0, v1, v2, u0, u1, u2)) {
                    quality.selfIntersectingPairs++;
                    quality.hasSelfIntersections = true;
                }
                checksPerformed++;
            }
        }
        
        if (checksPerformed >= maxChecks) {
            std::cout << "  (Stopped after " << maxChecks << " checks)" << std::endl;
        }
    }
    
    return quality;
}

std::map<std::pair<int, int>, int> MeshDiagnostics::buildEdgeMap(
    const std::vector<unsigned int>& indices
) {
    std::map<std::pair<int, int>, int> edgeCount;
    
    for (size_t i = 0; i < indices.size(); i += 3) {
        int v[3] = { (int)indices[i], (int)indices[i + 1], (int)indices[i + 2] };
        
        // For each edge in the triangle
        for (int j = 0; j < 3; ++j) {
            int a = v[j];
            int b = v[(j + 1) % 3];
            
            // Always store edge with smaller index first
            if (a > b) std::swap(a, b);
            
            edgeCount[{a, b}]++;
        }
    }
    
    return edgeCount;
}

bool MeshDiagnostics::trianglesIntersect(
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
    const glm::vec3& u0, const glm::vec3& u1, const glm::vec3& u2
) {
    // Simplified test: Check if any edge of triangle 1 intersects triangle 2
    // This is not a complete triangle-triangle intersection test but catches most cases
    
    // Test edges of first triangle against second triangle
    glm::vec3 edges1[3] = {
        v1 - v0, v2 - v1, v0 - v2
    };
    glm::vec3 starts1[3] = { v0, v1, v2 };
    
    for (int i = 0; i < 3; ++i) {
        float t;
        if (rayIntersectsTriangle(starts1[i], glm::normalize(edges1[i]), u0, u1, u2, t)) {
            if (t >= 0 && t <= glm::length(edges1[i])) {
                return true;
            }
        }
    }
    
    // Test edges of second triangle against first triangle
    glm::vec3 edges2[3] = {
        u1 - u0, u2 - u1, u0 - u2
    };
    glm::vec3 starts2[3] = { u0, u1, u2 };
    
    for (int i = 0; i < 3; ++i) {
        float t;
        if (rayIntersectsTriangle(starts2[i], glm::normalize(edges2[i]), v0, v1, v2, t)) {
            if (t >= 0 && t <= glm::length(edges2[i])) {
                return true;
            }
        }
    }
    
    return false;
}

bool MeshDiagnostics::rayIntersectsTriangle(
    const glm::vec3& rayOrigin, const glm::vec3& rayDir,
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
    float& t
) {
    // Möller–Trumbore intersection algorithm
    const float EPSILON = 0.0000001f;
    
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(rayDir, edge2);
    float a = glm::dot(edge1, h);
    
    if (a > -EPSILON && a < EPSILON)
        return false; // Ray is parallel to triangle
    
    float f = 1.0f / a;
    glm::vec3 s = rayOrigin - v0;
    float u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f)
        return false;
    
    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(rayDir, q);
    
    if (v < 0.0f || u + v > 1.0f)
        return false;
    
    t = f * glm::dot(edge2, q);
    
    return t > EPSILON;
}
