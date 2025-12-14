#ifndef VOXELHEATDIFFUSION_H
#define VOXELHEATDIFFUSION_H

#include <vector>
#include <glm/glm.hpp>

struct Vertex;
struct Bone;

class VoxelGrid {
public:
    int resX, resY, resZ;
    glm::vec3 origin;
    float voxelSize;
    std::vector<bool> solidVoxels;  // true if voxel is inside mesh
    
    VoxelGrid(int rx, int ry, int rz, glm::vec3 org, float size)
        : resX(rx), resY(ry), resZ(rz), origin(org), voxelSize(size) {
        solidVoxels.resize(rx * ry * rz, false);
    }
    
    int getIndex(int x, int y, int z) const {
        if (x < 0 || x >= resX || y < 0 || y >= resY || z < 0 || z >= resZ)
            return -1;
        return x + y * resX + z * resX * resY;
    }
    
    bool isSolid(int x, int y, int z) const {
        int idx = getIndex(x, y, z);
        return idx >= 0 && solidVoxels[idx];
    }
    
    glm::vec3 getVoxelCenter(int x, int y, int z) const {
        return origin + glm::vec3(
            (x + 0.5f) * voxelSize,
            (y + 0.5f) * voxelSize,
            (z + 0.5f) * voxelSize
        );
    }
    
    glm::ivec3 getVoxelCoords(const glm::vec3& worldPos) const {
        glm::vec3 local = (worldPos - origin) / voxelSize;
        return glm::ivec3(
            (int)std::floor(local.x),
            (int)std::floor(local.y),
            (int)std::floor(local.z)
        );
    }
};

class VoxelHeatDiffusion {
public:
    // Main function: compute bone weights using voxel-based method
    static void computeBoneWeights(
        std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices,
        std::vector<Bone>& skeleton,
        int voxelResolution = 32
    );

private:
    // Create voxel grid from mesh
    static VoxelGrid createVoxelGrid(
        const std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices,
        int resolution
    );
    
    // Mark voxels as solid (inside mesh) using conservative rasterization
    static void voxelizeMesh(
        VoxelGrid& grid,
        const std::vector<Vertex>& vertices,
        const std::vector<unsigned int>& indices
    );
    
    // Flood fill from outside to mark exterior voxels
    static void floodFillExterior(VoxelGrid& grid);
    
    // Solve heat diffusion on voxel grid from source point
    static std::vector<float> solveVoxelHeatDiffusion(
        const VoxelGrid& grid,
        const glm::ivec3& sourceVoxel,
        float timeStep,
        int iterations
    );
    
    // Interpolate voxel values to vertex
    static float interpolateVoxelToVertex(
        const VoxelGrid& grid,
        const std::vector<float>& voxelValues,
        const glm::vec3& vertexPos
    );
    
    // Check if ray intersects triangle
    static bool rayIntersectsTriangle(
        const glm::vec3& rayOrigin, const glm::vec3& rayDir,
        const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2
    );
};

#endif // VOXELHEATDIFFUSION_H
