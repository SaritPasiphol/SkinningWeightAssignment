#include "HeatDiffusion.h"
#include <iostream>
#include <limits>
#include <cmath>
#include <algorithm>

// Assuming these structures exist in main.cpp
// We need them declared here for compilation
struct Vertex {
    glm::vec3 originalPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    std::vector<float> boneWeights;
};

struct Edge { 
    int target; 
    float weight; 
};

struct Bone {
    std::string name;
    glm::vec3 bindPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    int seedVertex;
};

float HeatDiffusion::cotangent(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    // Compute cotangent of angle at v2
    glm::vec3 e1 = v1 - v2;
    glm::vec3 e2 = v3 - v2;
    
    float dot = glm::dot(e1, e2);
    float cross_length = glm::length(glm::cross(e1, e2));
    
    if (cross_length < 1e-8f) return 0.0f;
    return dot / cross_length;
}

std::vector<std::vector<std::pair<int, float>>> HeatDiffusion::buildCotangentLaplacian(
    const std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices
) {
    int n = vertices.size();
    std::vector<std::vector<std::pair<int, float>>> laplacian(n);
    std::vector<float> diagonal(n, 0.0f);

    // Process each triangle
    for (size_t i = 0; i < indices.size(); i += 3) {
        int idx[3] = { (int)indices[i], (int)indices[i+1], (int)indices[i+2] };
        glm::vec3 pos[3] = {
            vertices[idx[0]].originalPosition,
            vertices[idx[1]].originalPosition,
            vertices[idx[2]].originalPosition
        };

        // For each edge in the triangle
        for (int j = 0; j < 3; ++j) {
            int v1 = idx[j];
            int v2 = idx[(j+1) % 3];
            int v_opposite = idx[(j+2) % 3];

            // Cotangent of angle at opposite vertex
            float cot = cotangent(pos[j], pos[(j+2)%3], pos[(j+1)%3]);
            float weight = 0.5f * cot;

            // Add to Laplacian (symmetric)
            laplacian[v1].push_back({v2, -weight});
            laplacian[v2].push_back({v1, -weight});
            diagonal[v1] += weight;
            diagonal[v2] += weight;
        }
    }

    // Add diagonal entries
    for (int i = 0; i < n; ++i) {
        laplacian[i].push_back({i, diagonal[i]});
    }

    return laplacian;
}

std::vector<float> HeatDiffusion::solveHeatDiffusion(
    const std::vector<Vertex>& vertices,
    const std::vector<std::vector<std::pair<int, float>>>& laplacian,
    int sourceVertex,
    float timeStep,
    int iterations
) {
    int n = vertices.size();
    std::vector<float> heat(n, 0.0f);
    std::vector<float> newHeat(n, 0.0f);
    
    // Initialize with heat at source
    heat[sourceVertex] = 1.0f;

    // Improved iterative solver using Gauss-Seidel (more stable than Jacobi)
    // Solving: (I + t*L)u = u_0
    const float dampingFactor = 0.5f; // SOR parameter for stability
    
    for (int iter = 0; iter < iterations; ++iter) {
        float maxChange = 0.0f;
        
        for (int i = 0; i < n; ++i) {
            float sum = 0.0f;
            float diag = 0.0f;

            for (const auto& [j, weight] : laplacian[i]) {
                if (i == j) {
                    diag = weight;
                } else {
                    // Use already updated values (Gauss-Seidel)
                    sum += weight * heat[j];
                }
            }

            // Solve: (1 + t*diag) * u_new[i] = u_old[i] - t*sum
            float oldValue = heat[i];
            float newValue;
            
            if (i == sourceVertex) {
                // Keep source fixed
                newValue = 1.0f;
            } else {
                if (diag > 1e-8f) {
                    newValue = (oldValue - timeStep * sum) / (1.0f + timeStep * diag);
                } else {
                    newValue = oldValue;
                }
                // Apply damping for stability (SOR method)
                newValue = oldValue + dampingFactor * (newValue - oldValue);
            }
            
            heat[i] = newValue;
            maxChange = std::max(maxChange, std::abs(newValue - oldValue));
        }
        
        // Early exit if converged
        if (maxChange < 1e-6f) {
            break;
        }
    }

    return heat;
}

void HeatDiffusion::computeBoneWeights(
    std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices,
    const std::vector<std::vector<Edge>>& adjacency,
    std::vector<Bone>& skeleton,
    float timeStep
) {
    if (skeleton.empty()) return;

    std::cout << "Computing Heat Diffusion Weights..." << std::endl;

    // 1. Find seed vertex for each bone (closest vertex to bone position)
    for (auto& bone : skeleton) {
        float minDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for (int i = 0; i < vertices.size(); ++i) {
            float d = glm::distance(bone.bindPosition, vertices[i].originalPosition);
            if (d < minDist) {
                minDist = d;
                bestIdx = i;
            }
        }
        bone.seedVertex = bestIdx;
    }

    // 2. Build cotangent Laplacian
    std::cout << "  Building Laplacian..." << std::endl;
    auto laplacian = buildCotangentLaplacian(vertices, indices);

    // 3. Calculate average edge length for better time step
    float avgEdgeLength = 0.0f;
    int edgeCount = 0;
    for (const auto& edges : adjacency) {
        for (const auto& e : edges) {
            avgEdgeLength += e.weight;
            edgeCount++;
        }
    }
    if (edgeCount > 0) avgEdgeLength /= edgeCount;
    
    // Use MUCH larger time step to allow heat to spread across entire mesh
    float adaptiveTimeStep = avgEdgeLength * avgEdgeLength * 50.0f;
    std::cout << "  Average edge length: " << avgEdgeLength << ", Time step: " << adaptiveTimeStep << std::endl;

    // 4. Solve heat diffusion from each bone with more iterations
    std::vector<std::vector<float>> allHeatMaps;
    for (int b = 0; b < skeleton.size(); ++b) {
        std::cout << "  Solving heat for bone " << (b+1) << "/" << skeleton.size() << std::endl;
        auto heatMap = solveHeatDiffusion(vertices, laplacian, skeleton[b].seedVertex, adaptiveTimeStep, 1000);
        allHeatMaps.push_back(heatMap);
    }

    // 5. Normalize heat maps and apply inverse power weighting like Geodesic
    // First, find max heat for each bone to normalize
    std::vector<float> maxHeats(skeleton.size(), 0.0f);
    for (int b = 0; b < skeleton.size(); ++b) {
        for (float h : allHeatMaps[b]) {
            maxHeats[b] = std::max(maxHeats[b], h);
        }
    }
    
    for (int i = 0; i < vertices.size(); ++i) {
        vertices[i].boneWeights.clear();
        
        // Normalize heat values and convert to distances
        std::vector<float> distances;
        for (int b = 0; b < skeleton.size(); ++b) {
            float normalizedHeat = allHeatMaps[b][i] / (maxHeats[b] + 1e-8f);
            // Convert normalized heat [0,1] to distance: high heat = small distance
            // Use logarithmic scale to prevent extreme values
            float dist = -std::log(normalizedHeat + 1e-8f);
            distances.push_back(dist);
        }
        
        // Apply same weighting formula as Geodesic: w = 1 / (dist^4 + epsilon)
        std::vector<float> tempWeights;
        float totalInverseDist = 0.0f;
        for (const auto& dist : distances) {
            float w = 1.0f / (std::pow(dist, 4.0f) + 0.0001f);
            tempWeights.push_back(w);
            totalInverseDist += w;
        }
        
        // Normalize
        glm::vec3 debugColor(0.0f);
        for (int b = 0; b < skeleton.size(); ++b) {
            float normW = tempWeights[b] / totalInverseDist;
            vertices[i].boneWeights.push_back(normW);
            debugColor += skeleton[b].color * normW;
        }
        vertices[i].color = debugColor;
    }

    std::cout << "Heat Diffusion Weights Calculated." << std::endl;
}
