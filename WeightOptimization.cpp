#include "WeightOptimization.h"
#include <algorithm>
#include <iostream>
#include <string>

// Structures (matching main.cpp)
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

namespace WeightOptimization {

// Helper: Simple normalization (preserves weight ratios)
static std::vector<float> normalizeWeights(
    const std::vector<int>& selectedBones,
    const std::vector<float>& originalWeights
) {
    int n = selectedBones.size();
    if (n == 0) return {};
    
    std::vector<float> weights;
    float sum = 0.0f;
    
    for (int bone : selectedBones) {
        float w = (bone < originalWeights.size()) ? originalWeights[bone] : 0.0f;
        weights.push_back(w);
        sum += w;
    }
    
    // Normalize to sum = 1
    if (sum > 0.0001f) {
        for (float& w : weights) {
            w /= sum;
        }
    } else {
        // All weights were zero, distribute equally
        for (float& w : weights) {
            w = 1.0f / n;
        }
    }
    
    return weights;
}

void pruneWeights(
    std::vector<Vertex>& vertices,
    const std::vector<Bone>& skeleton,
    int maxBonesPerVertex
) {
    std::cout << "Pruning weights to max " << maxBonesPerVertex << " bones per vertex..." << std::endl;
    
    int totalBonesBefore = 0;
    int totalBonesAfter = 0;
    
    for (auto& v : vertices) {
        // Store original weights
        std::vector<float> originalWeights = v.boneWeights;
        
        // Count non-zero weights before
        for (float w : v.boneWeights) {
            if (w > 0.0001f) totalBonesBefore++;
        }
        
        // Create pairs of (boneIndex, weight)
        std::vector<std::pair<int, float>> boneWeightPairs;
        for (int i = 0; i < v.boneWeights.size(); ++i) {
            if (v.boneWeights[i] > 0.0001f) {
                boneWeightPairs.push_back({i, v.boneWeights[i]});
            }
        }
        
        // Sort by weight descending
        std::sort(boneWeightPairs.begin(), boneWeightPairs.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        
        // Keep only top N bones
        std::vector<int> selectedBones;
        for (int i = 0; i < std::min(maxBonesPerVertex, (int)boneWeightPairs.size()); ++i) {
            selectedBones.push_back(boneWeightPairs[i].first);
        }
        
        // Zero out all weights
        std::fill(v.boneWeights.begin(), v.boneWeights.end(), 0.0f);
        
        // Renormalize selected bones
        if (selectedBones.size() > 0) {
            auto newWeights = normalizeWeights(selectedBones, originalWeights);
            
            for (int i = 0; i < selectedBones.size(); ++i) {
                v.boneWeights[selectedBones[i]] = newWeights[i];
            }
        }
        
        // Recalculate color based on pruned weights
        glm::vec3 debugColor(0.0f);
        for (int b = 0; b < skeleton.size(); ++b) {
            debugColor += skeleton[b].color * v.boneWeights[b];
        }
        v.color = debugColor;
        
        // Count non-zero weights after
        for (float w : v.boneWeights) {
            if (w > 0.0001f) totalBonesAfter++;
        }
    }
    
    float avgBefore = (float)totalBonesBefore / vertices.size();
    float avgAfter = (float)totalBonesAfter / vertices.size();
    std::cout << "  Average bones per vertex: " << avgBefore << " -> " << avgAfter << std::endl;
    std::cout << "  Memory reduction: " << (100.0f * (1.0f - avgAfter/avgBefore)) << "%" << std::endl;
}

void smoothWeights(
    std::vector<Vertex>& vertices,
    const std::vector<std::vector<Edge>>& adjacency,
    const std::vector<Bone>& skeleton,
    int iterations
) {
    std::cout << "Smoothing weights (" << iterations << " iterations)..." << std::endl;
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Store smoothed weights
        std::vector<std::vector<float>> smoothedWeights(vertices.size());
        
        for (int i = 0; i < vertices.size(); ++i) {
            // Start with current weights
            smoothedWeights[i] = vertices[i].boneWeights;
            
            // Count neighbors and accumulate weights
            int neighborCount = 0;
            std::vector<float> accumulated(skeleton.size(), 0.0f);
            
            // Add neighbor weights
            for (const auto& edge : adjacency[i]) {
                int neighbor = edge.target;
                for (int b = 0; b < skeleton.size(); ++b) {
                    // Only accumulate if BOTH vertex and neighbor have this bone
                    if (vertices[i].boneWeights[b] > 0.0001f && 
                        vertices[neighbor].boneWeights[b] > 0.0001f) {
                        accumulated[b] += vertices[neighbor].boneWeights[b];
                    }
                }
                neighborCount++;
            }
            
            if (neighborCount > 0) {
                // Blend: 70% original + 30% neighbors (constrained)
                float alpha = 0.3f;
                for (int b = 0; b < skeleton.size(); ++b) {
                    if (vertices[i].boneWeights[b] > 0.0001f) {
                        float neighborAvg = accumulated[b] / neighborCount;
                        smoothedWeights[i][b] = (1.0f - alpha) * vertices[i].boneWeights[b] + 
                                                alpha * neighborAvg;
                    }
                }
                
                // Renormalize
                float sum = 0.0f;
                for (float w : smoothedWeights[i]) sum += w;
                if (sum > 0.0001f) {
                    for (float& w : smoothedWeights[i]) w /= sum;
                }
            }
        }
        
        // Apply smoothed weights
        for (int i = 0; i < vertices.size(); ++i) {
            vertices[i].boneWeights = smoothedWeights[i];
            
            // Update colors
            glm::vec3 debugColor(0.0f);
            for (int b = 0; b < skeleton.size(); ++b) {
                debugColor += skeleton[b].color * vertices[i].boneWeights[b];
            }
            vertices[i].color = debugColor;
        }
    }
    
    std::cout << "  Smoothing complete." << std::endl;
}

} // namespace WeightOptimization
