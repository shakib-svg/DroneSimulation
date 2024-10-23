#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#define MAX_LINE_LENGTH 100
#define MAX_DRONES 100
#define MAX_NEIGHBORS 10
#define Vmax 50
#define POSITION_THRESHOLD 0.1
#define MIN_DISTANCE 2.0
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"

typedef struct {
    int id;
    float x, y, z;
    float coverage;
    float vx, vy, vz;
    float communication_range;
    float camera_resolution;
    int neighbors[MAX_NEIGHBORS];
    int neighbor_count;
    int is_active;
} Drone;

typedef struct {
    float xMin, xMax, yMin, yMax;
    float width, height;
} Image;

int is_position_occupied(float x, float y, float z, int moving_drone_id);
void voronoi_adapt_all_drones(int moved_drone_id, Image* img1);
void avoid_collisions();
float random_float(float min, float max);
float calculate_distance(Drone* d1, Drone* d2);
float go_to(float x, float y, float z, int id, Image* img1);
float* speed(float vx, float vy, float vz, int id, float delta_t, Image* img1);
float calculate_coverage(float altitude);
Image spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res, Image Im1);
void capture(const char* path16, const char* path15, const char* path14);

#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
} BMPHeader;

typedef struct {
    uint32_t biSize;
    int32_t biWidth;
    int32_t biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t biXPelsPerMeter;
    int32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BMPInfoHeader;
#pragma pack(pop)

Drone drones[MAX_DRONES];
int drone_count = 0;

float calculate_coverage(float altitude) {
   return pow(2 * altitude * tan(0.05), 2);
}

int within_communication_range(Drone* d1, Drone* d2) {
    float dx = d1->x - d2->x;
    float dy = d1->y - d2->y;
    float dz = d1->z - d2->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= d1->communication_range;
}

void update_neighbors() {
    for (int i = 0; i < drone_count; i++) {
        drones[i].neighbor_count = 0;
        for (int j = 0; j < drone_count; j++) {
            if ((i != j) && (within_communication_range(&drones[i], &drones[j]))) {
                drones[i].neighbors[drones[i].neighbor_count] = drones[j].id;
                drones[i].neighbor_count++;
            }
        }
    }
}

Image spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res, Image Im1) {
    Image* img1 = &Im1;
    img1->xMax = xmax;
    img1->xMin = xmin;
    img1->yMax = ymax;
    img1->yMin = ymin;
    float total_area = (xmax - xmin) * (ymax - ymin);
    float c = sqrt(total_area / num_drones);
    float h = c / (2 * tan(0.05));
    float grid_size = c;
    int drones_per_row = (int)ceil((xmax - xmin) / grid_size);
    int drones_per_column = (int)ceil((ymax - ymin) / grid_size);
    int drone_id = 0;
    for (int i = 0; i < drones_per_row && drone_id < num_drones; i++) {
        for (int j = 0; j < drones_per_column && drone_id < num_drones; j++) {
            float x = xmin + i * grid_size + grid_size / 2;
            float y;
            if (y > ymax) {
                y = ymax - grid_size / 2;
            } else {
                y = ymin + j * grid_size + grid_size / 2;
            }
            float z = h;
            drones[drone_id].id = drone_id + 1;
            drones[drone_id].x = x;
            drones[drone_id].y = y;
            drones[drone_id].z = z;
            drones[drone_id].coverage = c * c;
            drones[drone_id].vx = 0;
            drones[drone_id].vy = 0;
            drones[drone_id].vz = 0;
            drones[drone_id].communication_range = comm_range;
            drones[drone_id].camera_resolution = cam_res;
            drones[drone_id].is_active = 1;
            drone_id++;
        }
    }
    drone_count = drone_id;
    update_neighbors();
    return *img1;
}

void adjust_neighbors_for_destroyed_drone(int destroyed_drone_id) {
    Drone destroyed_drone = drones[destroyed_drone_id];
    for (int i = 0; i < drone_count; i++) {
        if ((drones[i].is_active) && (drones[i].neighbor_count > 0)) {
            for (int j = 0; j < drones[i].neighbor_count; j++) {
                if ((drones[i].neighbors[j]) == (destroyed_drone.id)) {
                    float new_coverage = drones[i].coverage + destroyed_drone.coverage;
                    drones[i].z = sqrt(new_coverage) / (2 * tan(0.05));
                    drones[i].coverage = calculate_coverage(drones[i].z);
                    printf("Drone %d adapted its position to (%.2f, %.2f, %.2f) and coverage area to %.2f m^2 to cover for destroyed Drone %d.\n",
                        drones[i].id, drones[i].x, drones[i].y, drones[i].z, drones[i].coverage, destroyed_drone.id);
                }
            }
        }
    }
}

void check_and_adapt() {
    for (int i = 0; i < drone_count; i++) {
        if (!drones[i].is_active) {
            printf("Drone %d is destroyed, adapting neighbors...\n", drones[i].id);
            adjust_neighbors_for_destroyed_drone(i);
            update_neighbors();
        }
    }
}

void print_drone_infos() {
    for (int i = 0; i < drone_count; i++) {
        printf("Drone %d:\n", drones[i].id);
        printf("  Position (x=%.3f, y=%.3f, z=%.3f)\n", drones[i].x, drones[i].y, drones[i].z);
        printf("  Coverage Area = %.3f m^2\n", drones[i].coverage);
        printf("  Velocity (vx=%.3f, vy=%.3f, vz=%.3f)\n", drones[i].vx, drones[i].vy, drones[i].vz);
        printf("  Communication Range = %.3f meters\n", drones[i].communication_range);
        printf("  Camera Resolution = %.3f pixels\n", drones[i].camera_resolution);
        printf("  Active Status = %s\n", drones[i].is_active ? "Active" : "Destroyed");
        printf("\n");
    }
}

void destruc(int id) {
    Drone* d = &drones[id];
    d->is_active = 0;
}

void altitude_to_color(float altitude, uint8_t* r, uint8_t* g, uint8_t* b) {
    if (altitude < 20) {
        *r = 255; *g = 0; *b = 0;
    } else if (altitude < 40) {
        *r = 255; *g = 165; *b = 0;
    } else if (altitude < 60) {
        *r = 255; *g = 255; *b = 0;
    } else if (altitude < 80) {
        *r = 0; *g = 255; *b = 0;
    } else {
        *r = 0; *g = 0; *b = 255;
    }
}

int is_position_occupied(float x, float y, float z, int moving_drone_id) {
    for (int i = 0; i < drone_count; i++) {
        if ((i != moving_drone_id) && (drones[i].is_active)) {
            float dx = x - drones[i].x;
            float dy = y - drones[i].y;
            float dz = z - drones[i].z;
            if (sqrt(dx * dx + dy * dy + dz * dz) < POSITION_THRESHOLD) {
                return i;
            }
        }
    }
    return -1;
}

void calculate_centroid(float* centroid_x, float* centroid_y, float* centroid_z) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int active_drones = 0;
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].is_active) {
            sum_x += drones[i].x;
            sum_y += drones[i].y;
            sum_z += drones[i].z;
            active_drones++;
        }
    }
    if (active_drones > 0) {
        *centroid_x = sum_x / active_drones;
        *centroid_y = sum_y / active_drones;
        *centroid_z = sum_z / active_drones;
    }
}

float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

void normalize_vector(float* vx, float* vy, float* vz) {
    float magnitude = sqrt((*vx) * (*vx) + (*vy) * (*vy) + (*vz) * (*vz));
    if (magnitude != 0) {
        *vx /= magnitude;
        *vy /= magnitude;
        *vz /= magnitude;
    }
}

float calculate_distance(Drone* d1, Drone* d2) {
    return sqrt(pow(d1->x - d2->x, 2) + pow(d1->y - d2->y, 2) + pow(d1->z - d2->z, 2));
}

void avoid_overlap(Drone* d) {
    for (int j = 0; j < drone_count; j++) {
        if (d != &drones[j] && drones[j].is_active) {
            float distance = calculate_distance(d, &drones[j]);
            if (distance < MIN_DISTANCE) {
                float repulsion_strength = MIN_DISTANCE - distance;
                float repulsion_x = d->x - drones[j].x;
                float repulsion_y = d->y - drones[j].y;
                float repulsion_z = d->z - drones[j].z;
                normalize_vector(&repulsion_x, &repulsion_y, &repulsion_z);
                d->x += repulsion_x * repulsion_strength;
                d->y += repulsion_y * repulsion_strength;
                d->z += repulsion_z * repulsion_strength;
            }
        }
    }
}

void voronoi_adapt_all_drones(int moved_drone_id, Image* img1) {
    printf("Voronoi-based intelligent adaptation after drone %d moved...\n", moved_drone_id);
    float centroid_x, centroid_y, centroid_z;
    calculate_centroid(&centroid_x, &centroid_y, &centroid_z);
    for (int i = 0; i < drone_count; i++) {
        if ((i != moved_drone_id - 1) && drones[i].is_active) {
            Drone* d = &drones[i];
            float vx = d->x - centroid_x;
            float vy = d->y - centroid_y;
            float vz = d->z - centroid_z;
            normalize_vector(&vx, &vy, &vz);
            float offset_distance = random_float(1.0, 3.0);
            d->x += vx * offset_distance;
            d->y += vy * offset_distance;
            d->z += vz * offset_distance;
            if (d->x < img1->xMin) d->x = img1->xMin;
            if (d->x > img1->xMax) d->x = img1->xMax;
            if (d->y < img1->yMin) d->y = img1->yMin;
            if (d->y > img1->yMax) d->y = img1->yMax;
            if (d->z < 0) d->z = 0;
            avoid_overlap(d);
            printf("Drone %d adjusted to new position (%.2f, %.2f, %.2f) using Voronoi partitioning.\n",
                   d->id, d->x, d->y, d->z);
        }
    }
    update_neighbors();
}

void avoid_collisions() {
    for (int i = 0; i < drone_count; i++) {
        for (int j = i + 1; j < drone_count; j++) {
            if ((drones[i].is_active) && (drones[j].is_active)) {
                float distance = calculate_distance(&drones[i], &drones[j]);
                if (distance < POSITION_THRESHOLD) {
                    float dx = drones[i].x - drones[j].x;
                    float dy = drones[i].y - drones[j].y;
                    float dz = drones[i].z - drones[j].z;
                    float magnitude = sqrt(dx * dx + dy * dy + dz * dz);
                    dx /= magnitude;
                    dy /= magnitude;
                    dz /= magnitude;
                    drones[i].x += dx * 0.5;
                    drones[i].y += dy * 0.5;
                    drones[i].z += dz * 0.5;
                    drones[j].x -= dx * 0.5;
                    drones[j].y -= dy * 0.5;
                    drones[j].z -= dz * 0.5;
                    printf("Collision avoided between Drone %d and Drone %d.\n", drones[i].id, drones[j].id);
                }
            }
        }
    }
}

typedef struct Command {
    char type[MAX_LINE_LENGTH];
    char args[MAX_LINE_LENGTH];
    struct Command* next;
} Command;

Command* add_command(Command* head, const char* type, const char* args) {
    Command* new_command = (Command*)malloc(sizeof(Command));
    strcpy(new_command->type, type);
    strcpy(new_command->args, args);
    new_command->next = head;
    return new_command;
}

void free_commands(Command* head) {
    Command* current = head;
    while (current != NULL) {
        Command* temp = current;
        current = current->next;
        free(temp);
    }
}

void execute_commands(Command* head, float t_max, Image* img1) {
    Command* current = head;
    while (current != NULL) {
        if ((strcmp(current->type, "goto")) == 0) {
            int id;
            float x, y, z;
            sscanf(current->args, "%d %f %f %f", &id, &x, &y, &z);
        } else if ((strcmp(current->type, "speed")) == 0) {
            int id;
            float vx, vy, vz, delta_t = 0;
            sscanf(current->args, "%d %f %f %f", &id, &vx, &vy, &vz);
            char* delta_t_str = strchr(current->args, ';');
            if (delta_t_str != NULL) {
                sscanf(delta_t_str, "%f", &delta_t);
            } else {
                delta_t = t_max;
            }
            float* new_coords = speed(vx, vy, vz, id, delta_t, img1);
            if (new_coords) {
                printf("Drone %d moved to new position: x = %f, y = %f, z = %f\n", id, new_coords[0], new_coords[1], new_coords[2]);
                free(new_coords);
            }
        }
        current = current->next;
    }
}

float find_global_tmax(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return -1;
    }
    char line[MAX_LINE_LENGTH];
    float global_t_max = 0.0;
    while (fgets(line, sizeof(line), file)) {
        char type[MAX_LINE_LENGTH];
        char args[MAX_LINE_LENGTH];
        if ((sscanf(line, "%s %[^\n]", type, args)) == 2) {
            if ((strcmp(type, "goto")) == 0) {
                int id;
                float x, y, z;
                sscanf(args, "%d %f %f %f", &id, &x, &y, &z);
                float dx = x - drones[id].x;
                float dy = y - drones[id].y;
                float dz = z - drones[id].z;
                float distance = sqrt(dx * dx + dy * dy + dz * dz);
                float time_taken = distance / Vmax;
                if (time_taken > global_t_max) {
                    global_t_max = time_taken;
                }
            }
        }
    }
    fclose(file);
    return global_t_max;
}

void process_file(const char* filename, Image* img1) {
    float t_max = find_global_tmax(filename);
    if (t_max < 0) {
        printf("Error in finding global t_max.\n");
        return;
    }
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return;
    }
    char line[MAX_LINE_LENGTH];
    Command* command_queue = NULL;
    while (fgets(line, sizeof(line), file)) {
        char type[MAX_LINE_LENGTH];
        char args[MAX_LINE_LENGTH];
        if ((sscanf(line, "%s %[^\n]", type, args)) == 2) {
            if (((strcmp(type, "goto")) == 0) || ((strcmp(type, "speed")) == 0)) {
                command_queue = add_command(command_queue, type, args);
            } else if ((strcmp(type, "step")) == 0) {
                printf("Executing step...\n");
                execute_commands(command_queue, t_max, img1);
                free_commands(command_queue);
                command_queue = NULL;
            }
        }
    }
    if (command_queue != NULL) {
        printf("Executing remaining commands...\n");
        execute_commands(command_queue, t_max, img1);
        free_commands(command_queue);
    }
    fclose(file);
}

float go_to(float x, float y, float z, int id, Image* img1) {
    if ((id < 0) || (id >= drone_count) || (!drones[id].is_active)) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return -1;
    }
    if ((x < img1->xMin) || (x > img1->xMax) || (y < img1->yMin) || (y > img1->yMax) || z < 0) {
        printf("Error: Destination is out of bounds.\n");
        return -1;
    }
    Drone* d = &drones[id - 1];
    int occupied_by_id = is_position_occupied(x, y, z, id);
    if (occupied_by_id != -1) {
        printf("Destination is occupied by drone %d. Adjusting all drones intelligently...\n", occupied_by_id);
        voronoi_adapt_all_drones(id, img1);
    }
    float dx = x - d->x;
    float dy = y - d->y;
    float dz = z - d->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    float time_taken = distance / Vmax;
    d->x = x;
    d->y = y;
    d->z = z;
    voronoi_adapt_all_drones(id, img1);
    avoid_collisions();
    return time_taken;
}

float* speed(float vx, float vy, float vz, int id, float delta_t, Image* img1) {
    if ((id < 0) || (id >= drone_count) || (!drones[id].is_active)) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return NULL;
    }
    Drone* d = &drones[id - 1];
    float speed_magnitude = sqrt(vx * vx + vy * vy + vz * vz);
    if (speed_magnitude > Vmax) {
        float scale_factor = Vmax / speed_magnitude;
        vx *= scale_factor;
        vy *= scale_factor;
        vz *= scale_factor;
    }
    float xnew = vx * delta_t + d->x;
    float ynew = vy * delta_t + d->y;
    float znew = vz * delta_t + d->z;
    if ((xnew < img1->xMin) || (xnew > img1->xMax) || (ynew < img1->yMin) || (ynew > img1->yMax) || znew < 0) {
        printf("Error: New position out of bounds.\n");
        return NULL;
    }
    int occupied_by_id = is_position_occupied(xnew, ynew, znew, id);
    if (occupied_by_id != -1) {
        printf("Position is occupied by drone %d. Adjusting all drones intelligently...\n", occupied_by_id);
        voronoi_adapt_all_drones(id, img1);
    }
    d->x = xnew;
    d->y = ynew;
    d->z = znew;
    float* newco = (float*)malloc(3 * sizeof(float));
    if (newco == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }
    newco[0] = xnew;
    newco[1] = ynew;
    newco[2] = znew;
    voronoi_adapt_all_drones(id, img1);
    avoid_collisions();
    return newco;
}

void zoom_out(float z, int id, Image* img1) {
    Drone* d = &drones[id];
    go_to(d->x, d->y, z, id, img1);
}

void stretch(float x, float y, int id, Image* img1) {
    Drone* d = &drones[id];
    go_to(x, y, d->z, id, img1);
}

unsigned char* readZooms(const char* filename, int* width, int* height) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error: Could not open file %s for reading\n", filename);
        return NULL;
    }
    if (fscanf(file, "%d %d", width, height) != 2) {
        fprintf(stderr, "Error: Could not read image dimensions\n");
        fclose(file);
        return NULL;
    }
    unsigned char* pixelData = (unsigned char*)malloc((*width) * (*height) * 3);
    if (pixelData == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for pixel data\n");
        fclose(file);
        return NULL;
    }
    for (int y = 0; y < *height; y++) {
        for (int x = 0; x < *width; x++) {
            int r, g, b;
            if (fscanf(file, "%d %d %d", &r, &g, &b) != 3) {
                fprintf(stderr, "Error: Could not read pixel data at (%d, %d)\n", x, y);
                free(pixelData);
                fclose(file);
                return NULL;
            }
            pixelData[(y * (*width) + x) * 3 + 0] = (unsigned char)r;
            pixelData[(y * (*width) + x) * 3 + 1] = (unsigned char)g;
            pixelData[(y * (*width) + x) * 3 + 2] = (unsigned char)b;
        }
    }
    fclose(file);
    return pixelData;
}

unsigned char* createImageWithSubregion(unsigned char* pixelData, int mainWidth, int mainHeight, int startLine, int startCol, int numLines, int numCols) {
    unsigned char* newImage = (unsigned char*)malloc(mainWidth * mainHeight * 3);
    if (newImage == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for new image\n");
        return NULL;
    }
    for (int i = 0; i < mainWidth * mainHeight * 3; i++) {
        newImage[i] = 0;
    }
    for (int y = 0; y < numLines; y++) {
        for (int x = 0; x < numCols; x++) {
            if ((startLine + y) < mainHeight && (startCol + x) < mainWidth) {
                newImage[((startLine + y) * mainWidth + (startCol + x)) * 3 + 0] = pixelData[((startLine + y) * mainWidth + (startCol + x)) * 3 + 0];
                newImage[((startLine + y) * mainWidth + (startCol + x)) * 3 + 1] = pixelData[((startLine + y) * mainWidth + (startCol + x)) * 3 + 1];
                newImage[((startLine + y) * mainWidth + (startCol + x)) * 3 + 2] = pixelData[((startLine + y) * mainWidth + (startCol + x)) * 3 + 2];
            }
        }
    }
    return newImage;
}

void saveImageAsPNG(const char* filename, unsigned char* pixelData, int width, int height) {
    if (stbi_write_png(filename, width, height, 3, pixelData, width * 3)) {
        printf("Image saved to: %s\n", filename);
    } else {
        fprintf(stderr, "Error: Could not save image to %s\n", filename);
    }
}

int widthh = 2365 ;
int heightt = 1524;


void copySubregion(unsigned char* srcData, unsigned char* targetData, int imgWidth, int imgHeight, int startLine, int startCol, int subWidth, int subHeight) {
    for (int y = 0; y < subHeight; y++) {
        for (int x = 0; x < subWidth; x++) {
            if ((startLine + y) < imgHeight && (startCol + x) < imgWidth) {
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 0] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 0];
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 1] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 1];
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 2] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 2];
            }
        }
    }
}

int assignStandardValue(int value) {
    int standard1 = 700;
    int standard2 = 2000;
    int standard3 = 3300;
    int diff1 = abs(value - standard1);
    int diff2 = abs(value - standard2);
    int diff3 = abs(value - standard3);
    if (diff1 <= diff2 && diff1 <= diff3) {
        return standard1;
    } else if (diff2 <= diff1 && diff2 <= diff3) {
        return standard2;
    } else {
        return standard3;
    }
}

void capture(const char* path16, const char* path15, const char* path14) {
    unsigned char* newImage;
    unsigned char* zoom14 = readZooms(path14, &widthh, &heightt);
    unsigned char* zoom15 = readZooms(path15, &widthh, &heightt);
    unsigned char* zoom16 = readZooms(path16, &widthh, &heightt);
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].is_active) {
            float c = sqrt(calculate_coverage(drones[i].z));
            int y_dr_n = (int)(drones[i].y - c / 2);
            int x_dr_n = (int)(drones[i].x - c / 2);
            int b = assignStandardValue(drones[i].z);
            printf("c= %f, y_dr_n= %d,  x_dr_n= %d, b= %d\n", c, y_dr_n, x_dr_n, b);
            if (b == 700) {
                printf("700 \n");
                newImage = createImageWithSubregion(zoom16, widthh, heightt, x_dr_n, y_dr_n, (int)c, (int)c);
            }
            if (b == 2000) {
                printf("2000 \n");
                newImage = createImageWithSubregion(zoom15, widthh, heightt, x_dr_n, y_dr_n, (int)c, (int)c);
            }
            if (b == 3300) {
                printf("3300 \n");
                newImage = createImageWithSubregion(zoom14, widthh, heightt, x_dr_n, y_dr_n, (int)c, (int)c);
            }
            if (newImage == NULL) {
                fprintf(stderr, "Error: Failed to create subregion for drone %d\n", i);
                continue;
            }
            saveImageAsPNG("subregion_image.png", newImage, widthh, heightt);
            char* outputsuregionname = "./subregions/subregion_image.png";
            saveImageAsPNG(outputsuregionname, newImage, widthh, heightt);
            const char* originalImageFilename = "./subregions/subregion_image.png";
            const char* targetImageFilename = "./subregions/map30000resized.png";
            const char* outputImageFilename = "./subregions/map30000resized.png";
            int mainWidth, mainHeight, origChannels;
            unsigned char* originalData = stbi_load(originalImageFilename, &mainWidth, &mainHeight, &origChannels, 3);
            int targetWidth, targetHeight, targetChannels;
            unsigned char* targetData = stbi_load(targetImageFilename, &targetWidth, &targetHeight, &targetChannels, 3);
            copySubregion(originalData, targetData, mainWidth, mainHeight, x_dr_n, y_dr_n, (int)c, (int)c);
            if (stbi_write_png(outputImageFilename, targetWidth, targetHeight, 3, targetData, targetWidth * 3)) {
                printf("Modified image saved to: %s\n", outputImageFilename);
            } else {
                fprintf(stderr, "Error: Could not save output image\n");
            }
            free(newImage);
            printf("drone: %d", i);
        }
    }
}

int main() {
    int num_drones;
    Image Im1;
    float xMin, yMin, xMax, yMax, comm_range, cam_res;
    Image* img1;
    img1 = &Im1;
    printf("Enter the number of drones: ");
    scanf("%d", &num_drones);
    printf("Enter the area dimensions (xMin yMin xMax yMax): ");
    scanf("%f %f %f %f", &xMin, &yMin, &xMax, &yMax);
    printf("Enter the communication range: ");
    scanf("%f", &comm_range);
    printf("Enter the camera resolution: ");
    scanf("%f", &cam_res);
    spread_drones(num_drones, xMin, yMin, xMax, yMax, comm_range, cam_res, Im1);
    print_drone_infos();
    const char* inputFilename12000 = "./12000.txt";
    const char* inputFilename20000 = "./20000.txt";
    const char* inputFilename30000 = "./30000.txt";
    drones[6].is_active = 0;
    print_drone_infos();
    capture(inputFilename12000, inputFilename20000, inputFilename30000);
}  
