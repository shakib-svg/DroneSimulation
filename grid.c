#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GRID_ROWS 10
#define GRID_COLS 15
#define MAX_DRONES 50

typedef struct {
    int id;
    int x, y;     // Position dans la grille
    int is_active;
} Drone;

Drone drones[MAX_DRONES];
int grid[GRID_ROWS][GRID_COLS];

void init_grid() {
    // Initialiser la grille à zéro (aucun drone)
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            grid[i][j] = 0;
        }
    }
}

void print_grid() {
    // Affichage de la grille avec les ID des drones
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            if (grid[i][j] == 0)
                printf(" 0 ");
            else
                printf("%2d ", grid[i][j]);
        }
        printf("\n");
    }
}

void init_drone(int id, int x, int y) {
    drones[id - 1].id = id;
    drones[id - 1].x = x;
    drones[id - 1].y = y;
    drones[id - 1].is_active = 1;
    grid[x][y] = id;  // Mettre le drone dans la grille
}

void move_drone(int id, int new_x, int new_y) {
    if (drones[id - 1].is_active) {
        grid[drones[id - 1].x][drones[id - 1].y] = 0;  // Enlever l'ancienne position
        drones[id - 1].x = new_x;
        drones[id - 1].y = new_y;
        grid[new_x][new_y] = id;  // Mettre la nouvelle position
    }
}

void destroy_drone(int id) {
    if (drones[id - 1].is_active) {
        grid[drones[id - 1].x][drones[id - 1].y] = 0;
        drones[id - 1].is_active = 0;
        printf("Drone %d détruit.\n", id);
    }
}

void execute_command(const char* command) {
    char cmd_type[10];
    int id, x, y;
    
    sscanf(command, "%s", cmd_type);

    if (strcmp(cmd_type, "init") == 0) {
        sscanf(command, "%*s %d %d %d", &id, &x, &y);
        init_drone(id, x, y);
        printf("Drone %d initialisé à la position (%d, %d).\n", id, x, y);
    }
    else if (strcmp(cmd_type, "compas") == 0) {
        sscanf(command, "%*s %d %d %d", &id, &x, &y);
        move_drone(id, x, y);
        printf("Drone %d déplacé à la position (%d, %d).\n", id, x, y);
    }
    else if (strcmp(cmd_type, "destroy") == 0) {
        sscanf(command, "%*s %d", &id);
        destroy_drone(id);
    }
    else if (strcmp(cmd_type, "wait") == 0) {
        int time;
        sscanf(command, "%*s %d", &time);
        printf("Attente de %d secondes.\n", time);
    }
    else if (strcmp(cmd_type, "capture") == 0) {
        printf("Capture d'image en cours...\n");
    }
    else if (strcmp(cmd_type, "intruder") == 0) {
        printf("Intrus détecté !\n");
    }
}
void process_file(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Erreur lors de l'ouverture du fichier");
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        execute_command(line);
        print_grid();
    }
    
    fclose(file);
}
int main() {
    init_grid();  // Initialiser la grille à zéro
    
    const char* inputFilename = "commands.txt";
    process_file(inputFilename);  // Traiter les commandes à partir du fichier

    return 0;
}
