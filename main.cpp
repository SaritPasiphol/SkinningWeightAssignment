#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <queue>
#include <limits>
#include <algorithm>

#include "HeatDiffusion.h"
#include "MeshDiagnostics.h"
#include "VoxelHeatDiffusion.h"

// --- Structures ---
struct Vertex {
    glm::vec3 originalPosition;
    glm::vec3 currentPosition;
    glm::vec3 color;
    std::vector<float> boneWeights;
};

struct Edge { int target; float weight; };

struct Bone {
    std::string name;
    glm::vec3 bindPosition;
    glm::vec3 currentPosition;
    glm::vec3 color; 
    int seedVertex;
};

// --- Global Data ---
std::vector<Vertex> vertices;
std::vector<unsigned int> indices;
std::vector<std::vector<Edge>> adjacency;
std::vector<Bone> skeleton;

// --- Interaction Globals ---
int selectedBoneIndex = 0;
bool isDragging = false;
float lastX = 400, lastY = 300;
// Camera
float camYaw   = -90.0f; 
float camPitch =  0.0f; 
float camRadius = 8.0f;
// Weight calculation method
bool useHeatDiffusion = false; // Toggle: false = Geodesic, true = Heat Diffusion
bool useVoxelMethod = false;   // Toggle: false = Surface, true = Voxel-based
GLFWwindow* globalWindow = nullptr; // For updating window title 

// --- Shaders ---
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 vColor;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 overrideColor;
uniform bool useOverride;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    vColor = useOverride ? overrideColor : aColor;
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

// --- Loaders ---
bool loadOBJ(const char* path) {
    std::cout << "Loading Mesh: " << path << " ... ";
    std::ifstream file(path);
    if (!file.is_open()) return false;
    
    vertices.clear(); indices.clear();
    std::vector<glm::vec3> temp_positions;
    std::string line;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string prefix; ss >> prefix;
        if (prefix == "v") {
            glm::vec3 pos; ss >> pos.x >> pos.y >> pos.z;
            temp_positions.push_back(pos);
            vertices.push_back({ pos, pos, glm::vec3(0.5f), {} });
        }
        else if (prefix == "f") {
            std::vector<int> faceIndices;
            std::string segment;
            while (ss >> segment) {
                size_t slashPos = segment.find('/');
                if (slashPos != std::string::npos) segment = segment.substr(0, slashPos);
                faceIndices.push_back(std::stoi(segment) - 1);
            }
            for (size_t i = 1; i < faceIndices.size() - 1; ++i) {
                indices.push_back(faceIndices[0]);
                indices.push_back(faceIndices[i]);
                indices.push_back(faceIndices[i + 1]);
            }
        }
    }

    // Auto-Center & Scale
    if (!temp_positions.empty()) {
        glm::vec3 minBounds(1e9), maxBounds(-1e9);
        for (const auto& p : temp_positions) {
            minBounds = glm::min(minBounds, p);
            maxBounds = glm::max(maxBounds, p);
        }
        glm::vec3 center = (minBounds + maxBounds) * 0.5f;
        float maxDim = glm::max(glm::max(maxBounds.x - minBounds.x, maxBounds.y - minBounds.y), maxBounds.z - minBounds.z);
        float scale = 4.0f / maxDim; 

        for (auto& v : vertices) {
            v.originalPosition = (v.originalPosition - center) * scale;
            v.currentPosition = v.originalPosition; 
        }
        std::cout << "Auto-Centered." << std::endl;
    }
    return true;
}

bool loadSkeleton(const char* path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    skeleton.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        bool hasText = false; for (char c : line) if (!isspace(c)) hasText = true;
        if (!hasText) continue;
        std::stringstream ss(line);
        Bone b;
        if (!(ss >> b.name >> b.bindPosition.x >> b.bindPosition.y >> b.bindPosition.z)) continue;
        b.currentPosition = b.bindPosition;
        if (!(ss >> b.color.r >> b.color.g >> b.color.b)) b.color = glm::vec3(1.0f);
        skeleton.push_back(b);
    }
    return true;
}

// --- Algorithms ---
void buildGraph() {
    adjacency.clear(); adjacency.resize(vertices.size());
    for (size_t i = 0; i < indices.size(); i += 3) {
        int idx[3] = { (int)indices[i], (int)indices[i + 1], (int)indices[i + 2] };
        for (int j = 0; j < 3; ++j) {
            int u = idx[j], v = idx[(j + 1) % 3];
            float dist = glm::distance(vertices[u].originalPosition, vertices[v].originalPosition);
            adjacency[u].push_back({ v, dist }); adjacency[v].push_back({ u, dist });
        }
    }
}

std::vector<float> computeGeodesicMap(int startNode) {
    int n = vertices.size();
    std::vector<float> dist(n, std::numeric_limits<float>::max());
    using PII = std::pair<float, int>;
    std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;
    dist[startNode] = 0.0f; pq.push({ 0.0f, startNode });
    while (!pq.empty()) {
        float d = pq.top().first; int u = pq.top().second; pq.pop();
        if (d > dist[u]) continue;
        for (auto& edge : adjacency[u]) {
            if (dist[u] + edge.weight < dist[edge.target]) {
                dist[edge.target] = dist[u] + edge.weight;
                pq.push({ dist[edge.target], edge.target });
            }
        }
    }
    return dist;
}

void computeBoneWeights() {
    if (skeleton.empty()) return;
    // Update window title to show current mode
    if (globalWindow) {
        std::string title = "[ ] Select | WASD Move | Mode: ";
        if (useVoxelMethod) {
            title += "VOXEL (J) | H:Surface";
        } else if (useHeatDiffusion) {
            title += "SURFACE-HEAT (H) | J:Voxel";
        } else {
            title += "GEODESIC | H:Heat | J:Voxel";
        }
        glfwSetWindowTitle(globalWindow, title.c_str());
    }
    
    
    if (useVoxelMethod) {
        // Use Voxel-based Heat Diffusion (robust to defects!)
        VoxelHeatDiffusion::computeBoneWeights(vertices, indices, skeleton, 32);
    } else if (useHeatDiffusion) {
        // Use Surface Heat Diffusion method
        HeatDiffusion::computeBoneWeights(vertices, indices, adjacency, skeleton);
    } else {
        // Use Geodesic method (original)
        std::cout << "Computing Geodesic Weights..." << std::endl;
        for (auto& bone : skeleton) {
            float minDst = std::numeric_limits<float>::max();
            int bestIdx = -1;
            for (int i = 0; i < vertices.size(); ++i) {
                float d = glm::distance(bone.bindPosition, vertices[i].originalPosition);
                if (d < minDst) { minDst = d; bestIdx = i; }
            }
            bone.seedVertex = bestIdx;
        }
        std::vector<std::vector<float>> allDistances;
        for (auto& bone : skeleton) allDistances.push_back(computeGeodesicMap(bone.seedVertex));
        for (int i = 0; i < vertices.size(); ++i) {
            float totalInverseDist = 0.0f;
            std::vector<float> tempWeights;
            for (const auto& dMap : allDistances) {
                float w = 1.0f / (pow(dMap[i], 4.0f) + 0.0001f); 
                tempWeights.push_back(w); totalInverseDist += w;
            }
            glm::vec3 debugColor(0.0f);
            for (int b = 0; b < skeleton.size(); ++b) {
                float normW = tempWeights[b] / totalInverseDist;
                vertices[i].boneWeights.push_back(normW);
                debugColor += skeleton[b].color * normW;
            }
            vertices[i].color = debugColor;
        }
        std::cout << "Weights Calculated." << std::endl;
    }
}

void updateSkinning() {
    for (auto& v : vertices) {
        glm::vec3 newPos(0.0f);
        for (int b = 0; b < skeleton.size(); ++b) {
            glm::vec3 offset = skeleton[b].currentPosition - skeleton[b].bindPosition;
            newPos += offset * v.boneWeights[b];
        }
        v.currentPosition = v.originalPosition + newPos;
    }
}

// --- Input Callbacks ---
void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);
    
    // Toggle weight calculation method with H key (Surface Heat vs Geodesic)
    static bool hKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS && !hKeyPressed) {
        useHeatDiffusion = !useHeatDiffusion;
        useVoxelMethod = false; // Disable voxel when toggling H
        std::cout << "Switching to " << (useHeatDiffusion ? "Surface Heat Diffusion" : "Geodesic") << " method..." << std::endl;
        // Recalculate weights
        for (auto& v : vertices) { v.boneWeights.clear(); }
        computeBoneWeights();
        hKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_H) == GLFW_RELEASE) {
        hKeyPressed = false;
    }
    
    // Toggle voxel-based method with J key (Robust for broken meshes!)
    static bool jKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS && !jKeyPressed) {
        useVoxelMethod = !useVoxelMethod;
        std::cout << "\n========================================" << std::endl;
        std::cout << "Switching to " << (useVoxelMethod ? "VOXEL-BASED" : (useHeatDiffusion ? "Surface Heat" : "Geodesic")) << " method..." << std::endl;
        std::cout << "========================================\n" << std::endl;
        // Recalculate weights
        for (auto& v : vertices) { v.boneWeights.clear(); }
        computeBoneWeights();
        jKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_RELEASE) {
        jKeyPressed = false;
    }
    
    // Cycle Selection with Brackets
    static bool bracketPressed = false;
    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS && !bracketPressed) {
        selectedBoneIndex = (selectedBoneIndex + 1) % skeleton.size();
        bracketPressed = true;
    }
    else if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS && !bracketPressed) {
        selectedBoneIndex = (selectedBoneIndex - 1 + skeleton.size()) % skeleton.size();
        bracketPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_RELEASE && 
        glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_RELEASE) {
        bracketPressed = false;
    }

    // Move Bones
    float speed = 0.02f;
    if (!skeleton.empty()) {
        if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.y += speed;
        if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.y -= speed;
        if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.x -= speed;
        if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.x += speed;
        if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.z += speed;
        if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) skeleton[selectedBoneIndex].currentPosition.z -= speed;
        if(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) for(auto& b : skeleton) b.currentPosition = b.bindPosition;
    }
}

// Mouse callbacks (Same as before)
void mouse_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) isDragging = (action == GLFW_PRESS);
}
void cursor_callback(GLFWwindow* window, double xpos, double ypos) {
    if (isDragging) {
        float sensitivity = 0.3f;
        camYaw   += (xpos - lastX) * sensitivity;
        camPitch += (lastY - ypos) * sensitivity; 
        if(camPitch > 89.0f) camPitch = 89.0f;
        if(camPitch < -89.0f) camPitch = -89.0f;
    }
    lastX = xpos; lastY = ypos;
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    camRadius -= (float)yoffset * 1.0f;
    if (camRadius < 1.0f) camRadius = 1.0f;
}

// --- Main ---
int main(int argc, char** argv) {
    std::string objPath = (argc > 1) ? argv[1] : "cat.obj";
    std::string skelPath = (argc > 2) ? argv[2] : "cat_skeleton_16.txt"; // Default to 16 bone file

    if (!loadOBJ(objPath.c_str())) {
        std::cout << "ERROR: Failed to load mesh: " << objPath << std::endl;
        return -1;
    }
    if (!loadSkeleton(skelPath.c_str())) {
        std::cout << "ERROR: Failed to load skeleton: " << skelPath << std::endl;
        return -1;
    }
    buildGraph();
    
    // Diagnose mesh quality
    MeshQuality meshQuality = MeshDiagnostics::analyzeMesh(vertices, indices, true);
    meshQuality.print();
    
    // Auto-switch to voxel method if mesh has defects
    if (!meshQuality.isManifold || meshQuality.hasBoundaryEdges) {
        std::cout << "⚠ Mesh defects detected! Consider using voxel-based heat diffusion." << std::endl;
        // TODO: In future, automatically use voxel method here
    }
    
    computeBoneWeights();

    std::cout << "Initializing GLFW..." << std::endl;
    if (!glfwInit()) {
        std::cout << "ERROR: Failed to initialize GLFW!" << std::endl;
        return -1;
    }
    std::cout << "Creating window..." << std::endl;
    GLFWwindow* window = glfwCreateWindow(1024, 768, "[ ] Select | WASD Move | H:Heat | J:Voxel", NULL, NULL);
    if (!window) {
        std::cout << "ERROR: Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return -1;
    }
    std::cout << "Window created successfully!" << std::endl;
    globalWindow = window; // Store for title updates
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glfwSetCursorPosCallback(window, cursor_callback);
    glfwSetMouseButtonCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    GLuint vs = glCreateShader(GL_VERTEX_SHADER); glShaderSource(vs, 1, &vertexShaderSource, NULL); glCompileShader(vs);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER); glShaderSource(fs, 1, &fragmentShaderSource, NULL); glCompileShader(fs);
    GLuint program = glCreateProgram(); glAttachShader(program, vs); glAttachShader(program, fs); glLinkProgram(program);

    GLuint VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO); glGenBuffers(1, &VBO); glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // BONE RENDERING BUFFERS
    GLuint boneVAO, boneVBO;
    glGenVertexArrays(1, &boneVAO); glGenBuffers(1, &boneVBO);
    // Note: We upload data every frame in the loop

    GLint modelLoc = glGetUniformLocation(program, "model");
    GLint viewLoc = glGetUniformLocation(program, "view");
    GLint projLoc = glGetUniformLocation(program, "projection");
    GLint colorLoc = glGetUniformLocation(program, "overrideColor");
    GLint useLoc = glGetUniformLocation(program, "useOverride");

    glEnable(GL_DEPTH_TEST);
    // Allow changing point size for bones
    glEnable(GL_PROGRAM_POINT_SIZE); 

    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        updateSkinning();

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program);

        // Camera
        glm::vec3 camPos;
        camPos.x = cos(glm::radians(camYaw)) * cos(glm::radians(camPitch)) * camRadius;
        camPos.y = sin(glm::radians(camPitch)) * camRadius;
        camPos.z = sin(glm::radians(camYaw)) * cos(glm::radians(camPitch)) * camRadius;
        glm::mat4 view = glm::lookAt(camPos, glm::vec3(0,0,0), glm::vec3(0,1,0));
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), 1024.0f/768.0f, 0.1f, 100.0f);
        
        // Model Matrix (Rotate cat to stand up)
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1, 0, 0)); 

        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));

        // 1. Draw Mesh
        glBindVertexArray(VAO); // Bind Mesh VAO
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, currentPosition)); glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color)); glEnableVertexAttribArray(1);

        glUniform1i(useLoc, false);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        // 2. Draw Wireframe
        glUniform1i(useLoc, true);
        glUniform3f(colorLoc, 0.0f, 0.0f, 0.0f); 
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE); glPolygonOffset(-1.0, -1.0);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        glDisable(GL_POLYGON_OFFSET_LINE);

        // 3. DRAW SKELETON (POINTS)
        // Prepare Bone Data
        std::vector<float> boneData;
        for(int i=0; i<skeleton.size(); ++i) {
            glm::vec3 p = skeleton[i].currentPosition;
            boneData.push_back(p.x); boneData.push_back(p.y); boneData.push_back(p.z);
            // Color logic: Selected = White, Others = Bone Color
            if (i == selectedBoneIndex) { boneData.push_back(0.0f); boneData.push_back(0.0f); boneData.push_back(0.0f); }
            else { boneData.push_back(skeleton[i].color.r); boneData.push_back(skeleton[i].color.g); boneData.push_back(skeleton[i].color.b); }
        }

        glBindVertexArray(boneVAO);
        glBindBuffer(GL_ARRAY_BUFFER, boneVBO);
        glBufferData(GL_ARRAY_BUFFER, boneData.size() * sizeof(float), boneData.data(), GL_DYNAMIC_DRAW);
        // Pos
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0); glEnableVertexAttribArray(0);
        // Color
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float))); glEnableVertexAttribArray(1);

        glDisable(GL_DEPTH_TEST); // Draw ON TOP of mesh
        glPointSize(15.0f); // BIG DOTS
        glUniform1i(useLoc, false); // Use vertex colors we just uploaded
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawArrays(GL_POINTS, 0, skeleton.size());
        glEnable(GL_DEPTH_TEST);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}