#include <Arduino.h>

// Hardware Definitions
#define IR_FRONT_PIN 32
#define IR_LEFT_PIN 33
#define IR_RIGHT_PIN 34
#define MOTOR_L1 25
#define MOTOR_L2 26
#define MOTOR_R1 27
#define MOTOR_R2 14
#define ENC_L_A 18
#define ENC_L_B 19
#define ENC_R_A 16
#define ENC_R_B 17

// Constants
const int CELL_DISTANCE = 3000; // Encoder ticks per cell (adjust based on testing)
const int TURN_TICKS = 500;    // Encoder ticks for 90° turn
const int IR_THRESHOLD = 2000; // ADC value for wall detection

// Global State
typedef enum { NORTH, EAST, SOUTH, WEST } Heading;
typedef enum { FORWARD, LEFT, RIGHT, IDLE } Action;
typedef enum { TO_CENTER, TO_START, FINAL_SPRINT } ExplorationPhase;

typedef struct {
    bool north, east, south, west;
} WallGrid;

static bool initialized = false;
static WallGrid walls[16][16];
static int distances[16][16];
static bool visited[16][16];
static bool traveled[16][16];
static volatile long encL = 0, encR = 0;
static int x = 0, y = 0;
static Heading dir = NORTH;
static ExplorationPhase phase = TO_CENTER;
static int loop_counter = 0, last_x = -1, last_y = -1;

// Motor Control Functions
void setMotorLeft(int speed) {
    digitalWrite(MOTOR_L1, speed > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_L2, speed <= 0 ? HIGH : LOW);
}

void setMotorRight(int speed) {
    digitalWrite(MOTOR_R1, speed > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R2, speed <= 0 ? HIGH : LOW);
}

void stopMotors() {
    setMotorLeft(0);
    setMotorRight(0);
}

// Encoder ISRs
void IRAM_ATTR encLISR() { encL++; }
void IRAM_ATTR encRISR() { encR++; }

// Movement Functions (Blocking)
void moveForwardCell() {
    long target = encL + CELL_DISTANCE;
    setMotorLeft(255);
    setMotorRight(255);
    while (encL < target || encR < target) {
        if (encL >= target) setMotorLeft(0);
        if (encR >= target) setMotorRight(0);
        delay(1);
    }
    stopMotors();
}

void turnLeft90() {
    long target = encL + TURN_TICKS;
    setMotorLeft(-150);
    setMotorRight(150);
    while (encL < target) delay(1);
    stopMotors();
}

void turnRight90() {
    long target = encR + TURN_TICKS;
    setMotorLeft(150);
    setMotorRight(-150);
    while (encR < target) delay(1);
    stopMotors();
}

// Sensor Functions
bool wallFront() { return analogRead(IR_FRONT_PIN) > IR_THRESHOLD; }
bool wallLeft()  { return analogRead(IR_LEFT_PIN)  > IR_THRESHOLD; }
bool wallRight() { return analogRead(IR_RIGHT_PIN) > IR_THRESHOLD; }

void updateWalls() {
    switch (dir) {
        case NORTH:
            walls[x][y].north = wallFront();
            walls[x][y].west  = wallLeft();
            walls[x][y].east  = wallRight();
            if (y < 15) walls[x][y+1].south = walls[x][y].north;
            if (x > 0)  walls[x-1][y].east  = walls[x][y].west;
            if (x < 15) walls[x+1][y].west  = walls[x][y].east;
            break;
        case EAST:
            walls[x][y].east  = wallFront();
            walls[x][y].north = wallLeft();
            walls[x][y].south = wallRight();
            if (x < 15) walls[x+1][y].west  = walls[x][y].east;
            if (y < 15) walls[x][y+1].south = walls[x][y].north;
            if (y > 0)  walls[x][y-1].north = walls[x][y].south;
            break;
        case SOUTH:
            walls[x][y].south = wallFront();
            walls[x][y].east  = wallLeft();
            walls[x][y].west  = wallRight();
            if (y > 0)  walls[x][y-1].north = walls[x][y].south;
            if (x < 15) walls[x+1][y].west  = walls[x][y].east;
            if (x > 0)  walls[x-1][y].east  = walls[x][y].west;
            break;
        case WEST:
            walls[x][y].west  = wallFront();
            walls[x][y].south = wallLeft();
            walls[x][y].north = wallRight();
            if (x > 0)  walls[x-1][y].east  = walls[x][y].west;
            if (y > 0)  walls[x][y-1].north = walls[x][y].south;
            if (y < 15) walls[x][y+1].south = walls[x][y].north;
            break;
    }
}

// Floodfill Algorithm (Unchanged)
void run_floodfill(int goals[][2], int num_goals) {
    int queue[256][2], front = 0, rear = 0;
    for (int i = 0; i < 16; i++) 
        for (int j = 0; j < 16; j++) 
            distances[i][j] = 999;

    for (int i = 0; i < num_goals; i++) {
        int gx = goals[i][0], gy = goals[i][1];
        distances[gx][gy] = 0;
        queue[rear][0] = gx; queue[rear][1] = gy; rear++;
    }

    while (front < rear) {
        int cx = queue[front][0], cy = queue[front][1], current_dist = distances[cx][cy];
        front++;
        int directions[4][2] = {{0,1}, {1,0}, {0,-1}, {-1,0}};
        for (int d = 0; d < 4; d++) {
            int nx = cx + directions[d][0], ny = cy + directions[d][1];
            if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16) continue;
            bool wall_exists = false;
            switch (d) {
                case 0: wall_exists = walls[cx][cy].north; break;
                case 1: wall_exists = walls[cx][cy].east;  break;
                case 2: wall_exists = walls[cx][cy].south; break;
                case 3: wall_exists = walls[cx][cy].west;  break;
            }
            if (!wall_exists && (current_dist + 1 < distances[nx][ny])) {
                distances[nx][ny] = current_dist + 1;
                queue[rear][0] = nx; queue[rear][1] = ny; rear++;
            }
        }
    }
}

// Core Solver Function (Modified)
Action floodFill() {
    if (!initialized) {
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                visited[i][j] = traveled[i][j] = false;
                walls[i][j] = {
                    .north = (j == 15),
                    .east  = (i == 15),
                    .south = (j == 0),
                    .west  = (i == 0)
                };
            }
        }
        traveled[0][0] = true;
        int center_goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
        run_floodfill(center_goals, 4);
        initialized = true;
        phase = TO_CENTER;
    }

    traveled[x][y] = true;
    
    // Loop detection
    if (x == last_x && y == last_y) {
        if (++loop_counter > 5) {
            dir = (Heading)((dir + 2) % 4);
            return RIGHT; // Start 180° turn
        }
    } else {
        last_x = x; last_y = y; loop_counter = 0;
    }

    // Phase transitions
    if (phase == TO_CENTER && (x==7||x==8) && (y==7||y==8)) {
        phase = TO_START;
        memset(visited, 0, sizeof(visited));
        int start_goal[1][2] = {{0,0}};
        run_floodfill(start_goal, 1);
    } else if (phase == TO_START && x==0 && y==0) {
        phase = FINAL_SPRINT;
        int center_goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
        run_floodfill(center_goals, 4);
    } else if (phase == FINAL_SPRINT && (x==7||x==8) && (y==7||y==8)) {
        return IDLE;
    }

    // Update walls if new cell
    if (phase != FINAL_SPRINT && !visited[x][y]) {
        updateWalls();
        visited[x][y] = true;
        if (phase == TO_CENTER) {
            int center_goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
            run_floodfill(center_goals, 4);
        } else {
            int start_goal[1][2] = {{0,0}};
            run_floodfill(start_goal, 1);
        }
    }

    // Find best move
    int min_dist = 999, best_dir = -1;
    int directions[4][2] = {{0,1}, {1,0}, {0,-1}, {-1,0}};
    for (int d = 0; d < 4; d++) {
        int nx = x + directions[d][0], ny = y + directions[d][1];
        if (nx<0||nx>=16||ny<0||ny>=16) continue;
        bool wall = false;
        switch (d) {
            case 0: wall = walls[x][y].north; break;
            case 1: wall = walls[x][y].east;  break;
            case 2: wall = walls[x][y].south; break;
            case 3: wall = walls[x][y].west;  break;
        }
        if (!wall && distances[nx][ny] < min_dist) {
            min_dist = distances[nx][ny];
            best_dir = d;
        }
    }

    // Execute movement logic
    if (best_dir == -1) {
        dir = (Heading)((dir + 1) % 4);
        return RIGHT;
    }

    if (best_dir == dir) {
        if (wallFront()) { // Safety check
            updateWalls();
            run_floodfill(phase == TO_CENTER ? 
                (int[][2]){{7,7},{7,8},{8,7},{8,8}} : 
                (int[][2]){{0,0}}, 
                phase == TO_CENTER ? 4 : 1
            );
            return IDLE;
        }
        return FORWARD;
    } else if ((best_dir + 1) % 4 == dir) {
        return LEFT;
    } else if ((dir + 1) % 4 == best_dir) {
        return RIGHT;
    } else {
        return RIGHT; // First part of 180° turn
    }
}

// Main Control Loop
void setup() {
    pinMode(IR_FRONT_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
    pinMode(MOTOR_L1, OUTPUT); pinMode(MOTOR_L2, OUTPUT);
    pinMode(MOTOR_R1, OUTPUT); pinMode(MOTOR_R2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encLISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encRISR, RISING);
    Serial.begin(115200);
    delay(2000);
}

void loop() {
    static Action next_action = IDLE;
    
    if (next_action == IDLE) {
        next_action = floodFill();
    } else {
        switch (next_action) {
            case FORWARD:
                moveForwardCell();
                // Update position after movement
                switch (dir) {
                    case NORTH: y++; break;
                    case EAST:  x++; break;
                    case SOUTH: y--; break;
                    case WEST:  x--; break;
                }
                break;
            case LEFT:
                turnLeft90();
                dir = (Heading)((dir + 3) % 4);
                break;
            case RIGHT:
                turnRight90();
                dir = (Heading)((dir + 1) % 4);
                break;
            case IDLE: break;
        }
        next_action = IDLE;
        delay(50); // Allow sensors to stabilize
    }
}