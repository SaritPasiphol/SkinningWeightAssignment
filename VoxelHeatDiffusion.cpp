#include "VoxelHeatDiffusion.h"
#include <iostream>
#include <queue>
#include <algorithm>
#include <cmath>

struct Vertex {
    glm::vec3 originalPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    std::vector<float> boneWeights;
};

struct Bone {
    std::string name;
    glm::vec3 bindPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    int seedVertex;
};

VoxelGrid VoxelHeatDiffusion::createVoxelGrid(
    const std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices,
    int resolution
) {
    // Find bounding box
    glm::vec3 minBound(1e9f), maxBound(-1e9f);
    for (const auto& v : vertices) {
        minBound = glm::min(minBound, v.originalPosition);
        maxBound = glm::max(maxBound, v.originalPosition);
    }
    
    // Add padding
    glm::vec3 size = maxBound - minBound;
    float maxSize = std::max(std::max(size.x, size.y), size.z);
    float padding = maxSize * 0.1f;
    minBound -= glm::vec3(padding);
    maxBound += glm::vec3(padding);
    
    // Calculate voxel size
    size = maxBound - minBound;
    maxSize = std::max(std::max(size.x, size.y), size.z);
    float voxelSize = maxSize / resolution;
    
    // Calculate grid dimensions
    int resX = (int)std::ceil(size.x / voxelSize);
    int resY = (int)std::ceil(size.y / voxelSize);
    int resZ = (int)std::ceil(size.z / voxelSize);
    
    std::cout << "  Voxel grid: " << resX << "x" << resY << "x" << resZ 
              << " (" << (resX*resY*resZ) << " voxels)" << std::endl;
    std::cout << "  Voxel size: " << voxelSize << std::endl;
    
    return VoxelGrid(resX, resY, resZ, minBound, voxelSize);
}

bool VoxelHeatDiffusion::rayIntersectsTriangle(
    const glm::vec3& rayOrigin, const glm::vec3& rayDir,
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2
) {
    const float EPSILON = 0.0000001f;
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(rayDir, edge2);
    float a = glm::dot(edge1, h);
    
    if (a > -EPSILON && a < EPSILON) return false;
    
    float f = 1.0f / a;
    glm::vec3 s = rayOrigin - v0;
    float u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) return false;
    
    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(rayDir, q);
    
    if (v < 0.0f || u + v > 1.0f) return false;
    
    float t = f * glm::dot(edge2, q);
    return t > EPSILON;
}

void VoxelHeatDiffusion::voxelizeMesh(
    VoxelGrid& grid,
    const std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices
) {
    std::cout << "  Voxelizing mesh..." << std::endl;
    
    // Conservative voxelization: mark voxel as solid if triangle overlaps it
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = vertices[indices[i]].originalPosition;
        glm::vec3 v1 = vertices[indices[i+1]].originalPosition;
        glm::vec3 v2 = vertices[indices[i+2]].originalPosition;
        
        // Find bounding box of triangle in voxel space
        glm::vec3 triMin = glm::min(glm::min(v0, v1), v2);
        glm::vec3 triMax = glm::max(glm::max(v0, v1), v2);
        
        glm::ivec3 voxelMin = grid.getVoxelCoords(triMin);
        glm::ivec3 voxelMax = grid.getVoxelCoords(triMax);
        
        // Clamp to grid bounds
        voxelMin = glm::max(voxelMin, glm::ivec3(0));
        voxelMax = glm::min(voxelMax, glm::ivec3(grid.resX-1, grid.resY-1, grid.resZ-1));
        
        // Test each voxel in bounding box
        for (int z = voxelMin.z; z <= voxelMax.z; ++z) {
            for (int y = voxelMin.y; y <= voxelMax.y; ++y) {
                for (int x = voxelMin.x; x <= voxelMax.x; ++x) {
                    glm::vec3 voxelCenter = grid.getVoxelCenter(x, y, z);
                    
                    // Simple test: if voxel center is close to triangle, mark as solid
                    glm::vec3 closest = v0; // Simplified: just check distance to vertices
                    float dist = glm::distance(voxelCenter, v0);
                    dist = std::min(dist, glm::distance(voxelCenter, v1));
                    dist = std::min(dist, glm::distance(voxelCenter, v2));
                    
                    if (dist < grid.voxelSize * 1.5f) {
                        int idx = grid.getIndex(x, y, z);
                        if (idx >= 0) grid.solidVoxels[idx] = true;
                    }
                }
            }
        }
    }
}

void VoxelHeatDiffusion::floodFillExterior(VoxelGrid& grid) {
    std::cout << "  Flood filling exterior..." << std::endl;
    
    std::vector<bool> exterior(grid.solidVoxels.size(), false);
    std::queue<glm::ivec3> q;
    
    // Start from all boundary voxels
    for (int z = 0; z < grid.resZ; ++z) {
        for (int y = 0; y < grid.resY; ++y) {
            for (int x = 0; x < grid.resX; ++x) {
                // Only start from boundary
                if (x == 0 || x == grid.resX-1 || y == 0 || y == grid.resY-1 || 
                    z == 0 || z == grid.resZ-1) {
                    int idx = grid.getIndex(x, y, z);
                    if (idx >= 0 && !grid.solidVoxels[idx] && !exterior[idx]) {
                        exterior[idx] = true;
                        q.push(glm::ivec3(x, y, z));
                    }
                }
            }
        }
    }
    
    // Flood fill from boundaries
    int dirs[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    while (!q.empty()) {
        glm::ivec3 v = q.front();
        q.pop();
        
        for (int d = 0; d < 6; ++d) {
            int nx = v.x + dirs[d][0];
            int ny = v.y + dirs[d][1];
            int nz = v.z + dirs[d][2];
            
            int idx = grid.getIndex(nx, ny, nz);
            if (idx >= 0 && !grid.solidVoxels[idx] && !exterior[idx]) {
                exterior[idx] = true;
                q.push(glm::ivec3(nx, ny, nz));
            }
        }
    }
    
    // Mark interior voxels as solid
    int interiorCount = 0;
    for (int i = 0; i < grid.solidVoxels.size(); ++i) {
        if (!exterior[i] && !grid.solidVoxels[i]) {
            grid.solidVoxels[i] = true;
            interiorCount++;
        }
    }
    
    int totalSolid = 0;
    for (bool s : grid.solidVoxels) if (s) totalSolid++;
    
    std::cout << "  Solid voxels: " << totalSolid << " (" 
              << (100.0f * totalSolid / grid.solidVoxels.size()) << "%)" << std::endl;
}

std::vector<float> VoxelHeatDiffusion::solveVoxelHeatDiffusion(
    const VoxelGrid& grid,
    const glm::ivec3& sourceVoxel,
    float timeStep,
    int iterations
) {
    int totalVoxels = grid.resX * grid.resY * grid.resZ;
    std::vector<float> heat(totalVoxels, 0.0f);
    
    // Initialize source
    int sourceIdx = grid.getIndex(sourceVoxel.x, sourceVoxel.y, sourceVoxel.z);
    if (sourceIdx >= 0 && grid.solidVoxels[sourceIdx]) {
        heat[sourceIdx] = 1.0f;
    }
    
    // 6-connected diffusion on grid (much simpler than mesh!)
    int dirs[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<float> newHeat = heat;
        
        for (int z = 0; z < grid.resZ; ++z) {
            for (int y = 0; y < grid.resY; ++y) {
                for (int x = 0; x < grid.resX; ++x) {
                    int idx = grid.getIndex(x, y, z);
                    if (idx < 0 || !grid.solidVoxels[idx]) continue;
                    
                    // Skip source
                    if (idx == sourceIdx) continue;
                    
                    // Diffusion from neighbors
                    float laplacian = 0.0f;
                    int neighborCount = 0;
                    
                    for (int d = 0; d < 6; ++d) {
                        int nx = x + dirs[d][0];
                        int ny = y + dirs[d][1];
                        int nz = z + dirs[d][2];
                        
                        if (grid.isSolid(nx, ny, nz)) {
                            int nidx = grid.getIndex(nx, ny, nz);
                            laplacian += heat[nidx];
                            neighborCount++;
                        }
                    }
                    
                    if (neighborCount > 0) {
                        laplacian = laplacian / neighborCount - heat[idx];
                        newHeat[idx] = heat[idx] + timeStep * laplacian;
                        newHeat[idx] = std::max(0.0f, std::min(1.0f, newHeat[idx]));
                    }
                }
            }
        }
        
        heat = newHeat;
    }
    
    return heat;
}

float VoxelHeatDiffusion::interpolateVoxelToVertex(
    const VoxelGrid& grid,
    const std::vector<float>& voxelValues,
    const glm::vec3& vertexPos
) {
    // Trilinear interpolation
    glm::vec3 local = (vertexPos - grid.origin) / grid.voxelSize;
    
    int x0 = (int)std::floor(local.x);
    int y0 = (int)std::floor(local.y);
    int z0 = (int)std::floor(local.z);
    
    // Clamp to grid
    x0 = std::max(0, std::min(grid.resX - 2, x0));
    y0 = std::max(0, std::min(grid.resY - 2, y0));
    z0 = std::max(0, std::min(grid.resZ - 2, z0));
    
    float fx = local.x - x0;
    float fy = local.y - y0;
    float fz = local.z - z0;
    
    // Get 8 corner values
    float c000 = 0.0f, c001 = 0.0f, c010 = 0.0f, c011 = 0.0f;
    float c100 = 0.0f, c101 = 0.0f, c110 = 0.0f, c111 = 0.0f;
    
    int idx;
    idx = grid.getIndex(x0, y0, z0); if (idx >= 0 && grid.solidVoxels[idx]) c000 = voxelValues[idx];
    idx = grid.getIndex(x0, y0, z0+1); if (idx >= 0 && grid.solidVoxels[idx]) c001 = voxelValues[idx];
    idx = grid.getIndex(x0, y0+1, z0); if (idx >= 0 && grid.solidVoxels[idx]) c010 = voxelValues[idx];
    idx = grid.getIndex(x0, y0+1, z0+1); if (idx >= 0 && grid.solidVoxels[idx]) c011 = voxelValues[idx];
    idx = grid.getIndex(x0+1, y0, z0); if (idx >= 0 && grid.solidVoxels[idx]) c100 = voxelValues[idx];
    idx = grid.getIndex(x0+1, y0, z0+1); if (idx >= 0 && grid.solidVoxels[idx]) c101 = voxelValues[idx];
    idx = grid.getIndex(x0+1, y0+1, z0); if (idx >= 0 && grid.solidVoxels[idx]) c110 = voxelValues[idx];
    idx = grid.getIndex(x0+1, y0+1, z0+1); if (idx >= 0 && grid.solidVoxels[idx]) c111 = voxelValues[idx];
    
    // Trilinear interpolation
    float c00 = c000 * (1 - fx) + c100 * fx;
    float c01 = c001 * (1 - fx) + c101 * fx;
    float c10 = c010 * (1 - fx) + c110 * fx;
    float c11 = c011 * (1 - fx) + c111 * fx;
    
    float c0 = c00 * (1 - fy) + c10 * fy;
    float c1 = c01 * (1 - fy) + c11 * fy;
    
    return c0 * (1 - fz) + c1 * fz;
}

void VoxelHeatDiffusion::computeBoneWeights(
    std::vector<Vertex>& vertices,
    const std::vector<unsigned int>& indices,
    std::vector<Bone>& skeleton,
    int voxelResolution
) {
    if (skeleton.empty()) return;
    
    std::cout << "\n=== VOXEL-BASED HEAT DIFFUSION ===" << std::endl;
    std::cout << "Resolution: " << voxelResolution << "³" << std::endl;
    
    // 1. Create voxel grid
    VoxelGrid grid = createVoxelGrid(vertices, indices, voxelResolution);
    
    // 2. Voxelize mesh
    voxelizeMesh(grid, vertices, indices);
    
    // 3. Fill interior
    floodFillExterior(grid);
    
    // 4. Find seed voxels for each bone
    for (auto& bone : skeleton) {
        glm::ivec3 voxel = grid.getVoxelCoords(bone.bindPosition);
        // Ensure it's within grid and solid
        voxel = glm::clamp(voxel, glm::ivec3(0), glm::ivec3(grid.resX-1, grid.resY-1, grid.resZ-1));
        
        // Find nearest solid voxel if not solid
        int idx = grid.getIndex(voxel.x, voxel.y, voxel.z);
        if (idx < 0 || !grid.solidVoxels[idx]) {
            float minDist = 1e9f;
            glm::ivec3 bestVoxel = voxel;
            
            for (int z = 0; z < grid.resZ; ++z) {
                for (int y = 0; y < grid.resY; ++y) {
                    for (int x = 0; x < grid.resX; ++x) {
                        if (grid.isSolid(x, y, z)) {
                            glm::vec3 center = grid.getVoxelCenter(x, y, z);
                            float dist = glm::distance(bone.bindPosition, center);
                            if (dist < minDist) {
                                minDist = dist;
                                bestVoxel = glm::ivec3(x, y, z);
                            }
                        }
                    }
                }
            }
            voxel = bestVoxel;
        }
        
        bone.seedVertex = grid.getIndex(voxel.x, voxel.y, voxel.z);
        std::cout << "  Bone '" << bone.name << "' -> voxel (" << voxel.x << "," << voxel.y << "," << voxel.z << ")" << std::endl;
    }
    
    // 5. Solve heat diffusion for each bone on voxel grid
    // Adaptive parameters based on grid size
    glm::vec3 gridSize(grid.resX, grid.resY, grid.resZ);
    float avgGridDim = (gridSize.x + gridSize.y + gridSize.z) / 3.0f;
    
    // Careful balance: we want heat to spread locally but not globally
    // Lower timeStep = slower spread = more localized
    // Fewer iterations = less total diffusion
    float timeStep = 0.06f;
    int iterations = std::min(250, std::max(100, (int)(avgGridDim * 8)));
    
    std::cout << "  Diffusion params: timeStep=" << timeStep << ", iterations=" << iterations << std::endl;
    
    std::vector<std::vector<float>> allHeatMaps;
    for (int b = 0; b < skeleton.size(); ++b) {
        std::cout << "  Solving voxel heat for bone " << (b+1) << "/" << skeleton.size() << std::endl;
        
        glm::ivec3 seedVoxel = grid.getVoxelCoords(skeleton[b].bindPosition);
        auto heatMap = solveVoxelHeatDiffusion(grid, seedVoxel, timeStep, iterations);
        allHeatMaps.push_back(heatMap);
    }
    
    // 6. Interpolate voxel weights to mesh vertices
    std::cout << "  Interpolating weights to vertices..." << std::endl;
    for (int i = 0; i < vertices.size(); ++i) {
        vertices[i].boneWeights.clear();
        
        // Sample heat from all bones at this vertex position
        std::vector<float> heats;
        for (int b = 0; b < skeleton.size(); ++b) {
            heats.push_back(interpolateVoxelToVertex(grid, allHeatMaps[b], vertices[i].originalPosition));
        }
        
        // Convert heat to weights with balanced inverse distance formula
        std::vector<float> tempWeights;
        float totalWeight = 0.0f;
        for (float h : heats) {
            // Clamp heat values to valid range
            h = std::max(1e-6f, std::min(1.0f, h));
            
            // Convert heat to distance-like metric (high heat = low distance)
            // Use moderate power for good locality without over-sharpening
            float dist = -std::log(h + 1e-6f);
            float w = 1.0f / (std::pow(dist, 4.0f) + 1e-6f);
            tempWeights.push_back(w);
            totalWeight += w;
        }
        
        // Normalize and assign
        glm::vec3 debugColor(0.0f);
        for (int b = 0; b < skeleton.size(); ++b) {
            float normW = tempWeights[b] / (totalWeight + 1e-8f);
            vertices[i].boneWeights.push_back(normW);
            debugColor += skeleton[b].color * normW;
        }
        vertices[i].color = debugColor;
    }
    
    std::cout << "Voxel-based weights calculated!\n" << std::endl;
}
