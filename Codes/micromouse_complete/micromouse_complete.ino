#include <Arduino.h>
#include <math.h>

// Maze parameters
#define SIZE 16
#define INF 99
#define MAX_QUEUE 256

// Direction vectors (N, E, S, W)
const int dx[] = {0, 1, 0, -1};
const int dy[] = {1, 0, -1, 0};

// Direction enum
enum Heading { NORTH, EAST, SOUTH, WEST };
enum Action { FORWARD, RIGHT, LEFT, IDLE };

// --- IR Sensor Pins ---
#define IR_LEFT   34
#define IR_FRONT  35
#define IR_RIGHT  32

// --- Encoder Pins ---
#define ENC_LEFT_A   4
#define ENC_LEFT_B   16
#define ENC_RIGHT_A  17
#define ENC_RIGHT_B  5

// --- Motor Driver Pins ---
#define IN1 25
#define IN2 26
#define ENA 27  // Enable pin for left motor
#define IN3 14
#define IN4 12
#define ENB 13  // Enable pin for right motor

// Motor Constants
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int MAX_PWM = 200;  // Reduce if motors overheat
const float COUNTS_PER_REV = 12.0 * 4.0;  // 12 CPR * 4X quadrature
const float WHEEL_DIAM = 43.0;  // mm
const float WHEEL_BASE = 90.0;  // mm
const int CELL_DISTANCE = 180;  // mm (1 cell)

// Global variables
static bool initialized = false;
static int distances[SIZE][SIZE];
static int walls[SIZE][SIZE][4] = {{{0}}};
static int x = 0, y = 0;
static Heading dir = NORTH;
static bool traveled[SIZE][SIZE];

// Floodfill queue
typedef struct { int x, y; } Cell;
Cell queue[MAX_QUEUE];
int front = 0, rear = 0;

// Motor control variables
volatile long encL = 0, encR = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// IR Sensor Calibration
const int WALL_THRESHOLD = 2000;  // Adjust based on testing

// Function prototypes
void IRAM_ATTR encL_ISR();
void IRAM_ATTR encR_ISR();
void setLeftMotor(int pwm);
void setRightMotor(int pwm);
void moveStraight(float distance, int maxPWM = MAX_PWM);
void turnRobot(float degrees);
bool wallFront();
bool wallLeft();
bool wallRight();
void addWall(int x, int y, int wallDir);
void bfsFloodfill();
void updateWalls();
void solver();
bool isAtCenter();
void initializeMaze();

// Encoder ISRs
void IRAM_ATTR encL_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  digitalRead(ENC_LEFT_B) ? encL-- : encL++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encR_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  digitalRead(ENC_RIGHT_B) ? encR-- : encR++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Motor control functions
void setLeftMotor(int pwm) {
  digitalWrite(IN1, pwm > 0 ? HIGH : LOW);
  digitalWrite(IN2, pwm > 0 ? LOW : HIGH);
  ledcWrite(0, abs(pwm));
}

void setRightMotor(int pwm) {
  digitalWrite(IN3, pwm > 0 ? HIGH : LOW);
  digitalWrite(IN4, pwm > 0 ? LOW : HIGH);
  ledcWrite(1, abs(pwm));
}

// Move straight with PI control
void moveStraight(float distance, int maxPWM) {
  float targetCounts = (distance / (PI * WHEEL_DIAM)) * COUNTS_PER_REV;
  long startL = encL, startR = encR;
  float errorL = 0, errorR = 0, integralL = 0, integralR = 0;
  const float Kp = 0.8, Ki = 0.05;  // Tune these values
  unsigned long lastTime = millis();

  while (abs(encL - startL) < targetCounts || abs(encR - startR) < targetCounts) {
    // Calculate time delta
    unsigned long now = millis();
    float deltaT = (now - lastTime) / 1000.0;
    lastTime = now;
    if (deltaT <= 0) deltaT = 0.01;

    // Left motor PI control
    errorL = targetCounts - abs(encL - startL);
    integralL += errorL * deltaT;
    float correctionL = Kp * errorL + Ki * integralL;
    int pwmL = constrain(maxPWM + correctionL, 0, MAX_PWM);

    // Right motor PI control
    errorR = targetCounts - abs(encR - startR);
    integralR += errorR * deltaT;
    float correctionR = Kp * errorR + Ki * integralR;
    int pwmR = constrain(maxPWM + correctionR, 0, MAX_PWM);

    // Apply motor speeds
    setLeftMotor(pwmL);
    setRightMotor(pwmR);

    delay(10);
  }
  setLeftMotor(0);
  setRightMotor(0);
  delay(100);  // Brief pause after movement
}

// Turn robot with precise angle control
void turnRobot(float degrees) {
  float radians = degrees * PI / 180.0;
  float distance = (WHEEL_BASE / 2.0) * radians;
  float targetCounts = (distance / (PI * WHEEL_DIAM)) * COUNTS_PER_REV;
  long startL = encL, startR = encR;
  
  if (degrees > 0) {  // Turn right
    while (abs(encL - startL) < targetCounts || abs(encR - startR) < targetCounts) {
      setLeftMotor(MAX_PWM/2);
      setRightMotor(-MAX_PWM/2);
    }
  } else {  // Turn left
    while (abs(encR - startR) < targetCounts || abs(encL - startL) < targetCounts) {
      setLeftMotor(-MAX_PWM/2);
      setRightMotor(MAX_PWM/2);
    }
  }
  setLeftMotor(0);
  setRightMotor(0);
  delay(100);  // Brief pause after turn
}

// IR Sensor Functions
bool wallFront() { return analogRead(IR_FRONT) > WALL_THRESHOLD; }
bool wallLeft() { return analogRead(IR_LEFT) > WALL_THRESHOLD; }
bool wallRight() { return analogRead(IR_RIGHT) > WALL_THRESHOLD; }

// Maze functions
int isValid(int x, int y) {
  return x >= 0 && x < SIZE && y >= 0 && y < SIZE;
}

void enqueue(int x, int y) {
  if (rear < MAX_QUEUE) {
    queue[rear].x = x;
    queue[rear].y = y;
    rear++;
  }
}

Cell dequeue() {
  if (front < rear) return queue[front++];
  return (Cell){-1, -1};
}

void addWall(int x, int y, int wallDir) {
  if (!isValid(x, y)) return;
  walls[x][y][wallDir] = 1;

  // Add opposite wall to adjacent cell
  int nx = x + dx[wallDir];
  int ny = y + dy[wallDir];
  int opposite = (wallDir + 2) % 4;

  if (isValid(nx, ny)) walls[nx][ny][opposite] = 1;
}

void bfsFloodfill() {
  // Reset grid to INF
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      distances[i][j] = INF;
    }
  }

  // Reset queue
  front = rear = 0;

  // Set center goals
  int goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
  for (int i = 0; i < 4; i++) {
    int gx = goals[i][0];
    int gy = goals[i][1];
    distances[gx][gy] = 0;
    enqueue(gx, gy);
  }

  while (front < rear) {
    Cell current = dequeue();
    int cx = current.x;
    int cy = current.y;
    int val = distances[cx][cy];

    for (int d = 0; d < 4; d++) {
      if (walls[cx][cy][d]) continue;

      int nx = cx + dx[d];
      int ny = cy + dy[d];

      if (isValid(nx, ny) && distances[nx][ny] == INF) {
        distances[nx][ny] = val + 1;
        enqueue(nx, ny);
      }
    }
  }
}

void updateWalls() {
  // Get walls relative to current direction
  bool frontWall = wallFront();
  bool leftWall = wallLeft();
  bool rightWall = wallRight();

  // Convert to absolute directions
  int absFront = dir;
  int absLeft = (dir + 3) % 4;
  int absRight = (dir + 1) % 4;

  // Add detected walls
  if (frontWall) addWall(x, y, absFront);
  if (leftWall) addWall(x, y, absLeft);
  if (rightWall) addWall(x, y, absRight);
}

bool isAtCenter() {
  return (x == 7 || x == 8) && (y == 7 || y == 8);
}

void initializeMaze() {
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      traveled[i][j] = false;
      // Boundary walls
      if (i == 0) walls[i][j][WEST] = 1;
      if (i == SIZE-1) walls[i][j][EAST] = 1;
      if (j == 0) walls[i][j][SOUTH] = 1;
      if (j == SIZE-1) walls[i][j][NORTH] = 1;
    }
  }
  traveled[0][0] = true;
  initialized = true;
}

void solver() {
  if (!initialized) {
    initializeMaze();
    bfsFloodfill();
    return;
  }

  if (isAtCenter()) {
    // Stop motors and flash LED to indicate completion
    setLeftMotor(0);
    setRightMotor(0);
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // Update walls for current position
  updateWalls();
  bfsFloodfill();

  // Find best move
  int min_dist = INF;
  int best_dir = -1;

  for (int d = 0; d < 4; d++) {
    if (walls[x][y][d]) continue;

    int nx = x + dx[d];
    int ny = y + dy[d];

    if (isValid(nx, ny) && distances[nx][ny] < min_dist) {
      min_dist = distances[nx][ny];
      best_dir = d;
    }
  }

  // Handle case where no valid move is found
  if (best_dir == -1) {
    turnRobot(180);
    dir = (Heading)((dir + 2) % 4);
    return;
  }

  // Execute move
  if (best_dir == dir) {
    moveStraight(CELL_DISTANCE);
    x += dx[dir];
    y += dy[dir];
  } 
  else if (best_dir == (dir + 1) % 4) {
    turnRobot(90);
    dir = (Heading)((dir + 1) % 4);
  } 
  else if (best_dir == (dir + 3) % 4) {
    turnRobot(-90);
    dir = (Heading)((dir + 3) % 4);
  } 
  else {
    turnRobot(180);
    dir = (Heading)((dir + 2) % 4);
  }

  // Mark new position as traveled
  if (!traveled[x][y]) traveled[x][y] = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Motor Setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcSetup(0, PWM_FREQ, PWM_RES);  // Channel 0 for left motor
  ledcSetup(1, PWM_FREQ, PWM_RES);  // Channel 1 for right motor
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);

  // Encoder Setup
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encR_ISR, CHANGE);

  // IR Sensor Setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Initialize maze solver
  initializeMaze();
  bfsFloodfill();
}

void loop() {
  if (!initialized) return;
  solver();
  delay(50);  // Short delay between solver cycles
}