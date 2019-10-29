#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // position stored as 4 floats (4th component is supposed to be 1.0)
    Vector4f position;
    // color stored as 4 unsigned char
    Vector4uc color;
};

float distance(Vector4f v1, Vector4f v2) {
    return sqrtf(
            pow(v1.x() - v2.x(), 2) + // x component
            pow(v1.y() - v2.y(), 2) + // y component
            pow(v1.z() - v2.z(), 2)   // z component
    );
}

bool WriteMesh(Vertex *vertices, unsigned int width, unsigned int height, const std::string &filename) {
    float edgeThreshold = 0.01f; // 1cm

    // TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
    // - have a look at the "off_sample.off" file to see how to store the vertices and triangles
    // - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
    // - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
    // - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
    // - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
    // - only write triangles with valid vertices and an edge length smaller then edgeThreshold

    // TODO: Get number of vertices
    // All vertices
    unsigned int nVertices = width * height;

    // TODO: Determine number of valid faces
    unsigned nFaces = 0;

    /*
     *  Calcuate all valid faces according to above conditions
     *  v0_____v1
     *   |    /|
     *   |  /  |
     *   |/    |
     *  v3-----v4
     */
    std::vector<std::vector<unsigned int>> validFaces;
    for (unsigned int i = 0; i < width * height; ++i) {
        if ((i + 1) % width == 0) continue; // Skip last column
        if ((int) (i / width) == height - 1) continue; // Skip last row

        Vector4f v0 = vertices[i].position;
        Vector4f v1 = vertices[i + 1].position;
        Vector4f v2 = vertices[i + 1 + width].position;
        Vector4f v3 = vertices[i + width].position;

        bool face1 = v0.x() != MINF && v1.x() != MINF && v3.x() != MINF;
        // Check face 1 v0, v1, v3
        if (face1) face1 = distance(v0, v1) < edgeThreshold;
        if (face1) face1 = distance(v1, v3) < edgeThreshold;
        if (face1) face1 = distance(v3, v0) < edgeThreshold;

        bool face2 = v1.x() != MINF && v2.x() != MINF && v3.x() != MINF;
        // Check face 2 v1, v2, v3
        if (face2) face2 = distance(v1, v2) < edgeThreshold;
        if (face2) face2 = distance(v2, v3) < edgeThreshold;
        if (face2) face2 = distance(v3, v1) < edgeThreshold;

        if (face1) { // Store face 1 if it is valid
            std::vector<unsigned int> vertexSet{i, i + width, i + 1};
            validFaces.push_back(vertexSet);
            nFaces++;
        }
        if (face2) { // Store face 2 if it is valid
            std::vector<unsigned int> vertexSet{i + 1, i + width, i + 1 + width};
            validFaces.push_back(vertexSet);
            nFaces++;
        }
    }

    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    // TODO: save vertices
    for (unsigned int i = 0; i < width * height; ++i) {
        Vertex v = vertices[i];
        if (v.position.x() != MINF) {
            // X Y Z R G B A
            outFile << v.position.x() << " " << v.position.y() << " " << v.position.z() << " ";
            outFile << (int) v.color.x() << " " << (int) v.color.y() << " ";
            outFile << (int) v.color.z() << " " << (int) v.color.w() << std::endl;
        } else outFile << "0 0 0 0 0 0 0" << std::endl;
    }

    // TODO: save valid faces
    for (std::vector<unsigned int> v : validFaces) outFile << "3 " << v[0] << " " << v[1] << " " << v[2] << std::endl;

    // close file
    outFile.close();

    return true;
}

int main() {
    // Make sure this path points to the data folder
    std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    // convert video to meshes
    while (sensor.ProcessNextFrame()) {
        // get ptr to the current depth frame
        // depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
        float *depthMap = sensor.GetDepth();
        // get ptr to the current color frame
        // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
        BYTE *colorMap = sensor.GetColorRGBX();

        // get depth intrinsics
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        float fX = depthIntrinsics(0, 0);
        float fY = depthIntrinsics(1, 1);
        float cX = depthIntrinsics(0, 2);
        float cY = depthIntrinsics(1, 2);

        // compute inverse depth extrinsics
        Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

        Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

        // TODO 1: back-projection
        // write result to the vertices array below, keep pixel ordering!
        // if the depth value at idx is invalid (MINF) write the following values to the vertices array
        // vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
        // vertices[idx].color = Vector4uc(0,0,0,0);
        // otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
        Vertex *vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

        for (unsigned int idx = 0; idx < sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight(); ++idx) {
            float depth = depthMap[idx];
            // Skip if depth is not valid
            if (depth == MINF) {
                vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[idx].color = Vector4uc(0, 0, 0, 0);
                continue;
            }

            unsigned int ux = idx % sensor.GetDepthImageWidth();
            unsigned int uy = idx / (unsigned int) sensor.GetDepthImageWidth();
            const float x = depth * ((float) ux - cX) / fX;
            const float y = depth * ((float) uy - cY) / fY;

            // Calculate pCamera with intrinsic values and x, y, depth
            Vector4f pCamera = Vector4f(x, y, depth, 1.0);
            // MCamera^-1 * E^-1 * pCamera
            Vector4f pWorld = trajectoryInv * depthExtrinsicsInv * pCamera;

            vertices[idx].position = pWorld;
            vertices[idx].color = Vector4uc(
                    colorMap[4 * idx],
                    colorMap[4 * idx + 1],
                    colorMap[4 * idx + 2],
                    colorMap[4 * idx + 3]
            );
        }

        // write mesh file
        std::stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str())) {
            std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            return -1;
        }

        // free mem
        delete[] vertices;
    }

    return 0;
}
