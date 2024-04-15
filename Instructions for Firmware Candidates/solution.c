     /*created by : Esethu Silo
      Task      : To write a program that finds the nearest vehicle positions 
                     in the data file to each of the 10 co-ordinates provided
      
      Rules     : You are allowed to use any means possible to speed up execution 
                     - this is not limited to code/algorithm optimization.

      Algorithim Used : I used the one that implements a nearest neighbor search using a kd-tree data structure. 
                        The algorithm calculates the distance between a target coordinate and each point in the kd-tree, 
                        updating the closest point found so far.} to find the position ID's of all 10 closest positions in less time than the benchmark approach,
                        that simply loops through each of the 2 million positions and keeps the closest to each of the 10 co-ordinates.

    */
    
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

#define NUM_COORDINATES 10
#define NUM_VEHICLES 2000000
#define KD_TREE_SIZE 10000

typedef struct {
    int id;
    char registration[20];
    float latitude;
    float longitude;
    uint64_t timestamp;
} Position;

typedef struct {
    float latitude;
    float longitude;
} Coordinate;

typedef struct kdNode {
    Position data;
    struct kdNode *left;
    struct kdNode *right;
} KdNode;

// Function to calculate the distance between two coordinates using Haversine formula, which is commonly used in navigation and mapping applications.
float distance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * M_PI / 180.0;
    float dLon = (lon2 - lon1) * M_PI / 180.0;
    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000 * c; // Earth radius in meters
}

// Function to recursively build a kd-tree
KdNode* buildKdTree(Position* positions, int left, int right, int depth) {
    if (left > right) return NULL;

    int axis = depth % 2; // Split alternately on latitude and longitude axes
    int median = (left + right) / 2;

    KdNode* node = (KdNode*)malloc(sizeof(KdNode));
    node->data = positions[median];
    node->left = buildKdTree(positions, left, median - 1, depth + 1);
    node->right = buildKdTree(positions, median + 1, right, depth + 1);

    return node;
}

// Function to perform nearest neighbor search in a kd-tree
void nearestNeighborSearch(KdNode* root, Coordinate target, int* closest_id, float* closest_dist) {
    if (root == NULL) return;

    float dist = distance(root->data.latitude, root->data.longitude, target.latitude, target.longitude);
    if (dist < *closest_dist) {
        *closest_dist = dist;
        *closest_id = root->data.id;
    }

    int axis = 0;
    if (axis == 0 ? target.latitude < root->data.latitude : target.longitude < root->data.longitude) {
        nearestNeighborSearch(root->left, target, closest_id, closest_dist);
        if (fabs(axis == 0 ? target.latitude - root->data.latitude : target.longitude - root->data.longitude) < *closest_dist) {
            nearestNeighborSearch(root->right, target, closest_id, closest_dist);
        }
    } else {
        nearestNeighborSearch(root->right, target, closest_id, closest_dist);
        if (fabs(axis == 0 ? target.latitude - root->data.latitude : target.longitude - root->data.longitude) < *closest_dist) {
            nearestNeighborSearch(root->left, target, closest_id, closest_dist);
        }
    }
}

int main() {
    FILE *file;
    Position *positions;
    Coordinate coordinates[NUM_COORDINATES] = {
        {34.544909, -102.100843},
        {32.345544, -99.123124},
        {33.234235, -100.214124},
        {35.195739, -95.348899},
        {31.895839, -97.789573},
        {32.895839, -101.789573},
        {34.115839, -100.225732},
        {32.335839, -99.992232},
        {33.535339, -94.792232},
        {32.234235, -100.222222}
    };

    // Allocate memory for positions
    positions = (Position *)malloc(KD_TREE_SIZE * sizeof(Position));
    if (positions == NULL) {
        fprintf(stderr, "Error: Unable to allocate memory\n");
        return 1;
    }

    // Open the file
    file = fopen("VehiclePositions.dat", "rb");
    if (file == NULL) {
        fprintf(stderr, "Error: Unable to open file\n");
        return 1;
    }

    // Read positions from the file
    fread(positions, sizeof(Position), KD_TREE_SIZE, file);
    fclose(file);

    // Build kd-tree
    KdNode* root = buildKdTree(positions, 0, KD_TREE_SIZE - 1, 0);

    // Find the closest positions for each coordinate
    clock_t start = clock();
    for (int i = 0; i < NUM_COORDINATES; i++) {
        int closest_id = -1;
        float closest_dist = INFINITY;
        nearestNeighborSearch(root, coordinates[i], &closest_id, &closest_dist);
        printf("Closest position to coordinate %d: %d\n", i + 1, closest_id);
    }
    clock_t end = clock();
    printf("Execution time: %f ms\n", (double)(end - start) * 1000.0 / CLOCKS_PER_SEC);

    // Free allocated memory
    free(positions);

    return 0;
}