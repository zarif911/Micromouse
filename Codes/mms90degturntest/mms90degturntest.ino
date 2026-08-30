#include <driver/pcnt.h>
#include <driver/ledc.h>

// Pin Definitions
#define ENC_LEFT_A   4
#define ENC_LEFT_B   16
#define ENC_RIGHT_A  17
#define ENC_RIGHT_B  5

#define IN1 25
#define IN2 26
#define ENA 27
#define IN3 14
#define IN4 12
#define ENB 13

// Constants
const int COUNTER_LIMIT = 32767;  // Max 16-bit signed value
const int GEAR_RATIO = 100;
const int PPR = 3;
const int CPR = 4 * PPR * GEAR_RATIO;  // Counts Per Revolution (1200)

// Global Variables
volatile int64_t left_accumulator = 0;
volatile int64_t right_accumulator = 0;
int16_t left_prev_count = 0;
int16_t right_prev_count = 0;

// PCNT Unit Configuration
void setup_pcnt_unit(pcnt_unit_t unit, 
                     int pulse_pin, 
                     int ctrl_pin,
                     pcnt_channel_t channel) {
  pcnt_config_t config = {
    .pulse_gpio_num = pulse_pin,
    .ctrl_gpio_num = ctrl_pin,
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = COUNTER_LIMIT,
    .counter_l_lim = -COUNTER_LIMIT,
    .unit = unit,
    .channel = channel
  };
  pcnt_unit_config(&config);
}

// Initialize Encoders
void init_encoders() {
  // Left encoder (Unit 0)
  setup_pcnt_unit(PCNT_UNIT_0, ENC_LEFT_A, ENC_LEFT_B, PCNT_CHANNEL_0);
  setup_pcnt_unit(PCNT_UNIT_0, ENC_LEFT_B, ENC_LEFT_A, PCNT_CHANNEL_1);
  
  // Right encoder (Unit 1)
  setup_pcnt_unit(PCNT_UNIT_1, ENC_RIGHT_A, ENC_RIGHT_B, PCNT_CHANNEL_0);
  setup_pcnt_unit(PCNT_UNIT_1, ENC_RIGHT_B, ENC_RIGHT_A, PCNT_CHANNEL_1);

  // Initialize and start units
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_1);
}

// Initialize Motor Control
void init_motors() {
  // Configure motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup PWM channels
  //ledcSetup(0, 5000, 8);  // 5kHz, 8-bit resolution
  //ledcSetup(1, 5000, 8);
  //ledcAttachPin(ENA, 0);
  //ledcAttachPin(ENB, 1);
}

// Set Motor Speed and Direction
void set_motor_speed(bool left, int speed) {
  int in1, in2, pwm_ch;
  if (left) {
    in1 = IN1;
    in2 = IN2;
    pwm_ch = 0;
  } else {
    in1 = IN3;
    in2 = IN4;
    pwm_ch = 1;
  }

  // Set direction
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  
  // Set PWM (absolute value)
  ledcWrite(pwm_ch, abs(speed));
}

// Read and accumulate encoder counts
void read_encoders() {
  int16_t left_count, right_count;
  
  // Read current counts
  pcnt_get_counter_value(PCNT_UNIT_0, &left_count);
  pcnt_get_counter_value(PCNT_UNIT_1, &right_count);
  
  // Calculate differences with overflow handling
  int16_t left_diff = (int16_t)(left_count - left_prev_count);
  int16_t right_diff = (int16_t)(right_count - right_prev_count);
  
  // Update accumulators
  left_accumulator += left_diff;
  right_accumulator += right_diff;
  
  // Update previous counts
  left_prev_count = left_count;
  right_prev_count = right_count;
}

// Encoder Task (runs every 100ms)
void encoder_task(void *pvParameters) {
  while (1) {
    read_encoders();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  init_encoders();
  init_motors();
  
  // Create encoder reading task
  xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 1, NULL);
  
  Serial.println("System ready");
}

void loop() {
  // Example motor control (adjust as needed)
  set_motor_speed(true, 150);  // Left motor forward @ 150/255
  set_motor_speed(false, -200); // Right motor backward @ 200/255
  
  // Print encoder counts every second
  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    Serial.printf("Left: %lld | Right: %lld\n", 
                  left_accumulator, 
                  right_accumulator);
    last_print = millis();
    
    // Optional: Calculate revolutions
    float left_rev = (float)left_accumulator / CPR;
    float right_rev = (float)right_accumulator / CPR;
    Serial.printf("Left: %.2f rev | Right: %.2f rev\n", left_rev, right_rev);
  }
  
  delay(10);
}