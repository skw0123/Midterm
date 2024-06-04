#include <NewPing.h>

// 핑할 최대 거리(센티미터 단위) 정의
#define MAX_DISTANCE 150            

// 벽 간격 거리(밀리미터 단위) 정의
#define WALL_GAP_DISTANCE 350       
#define WALL_GAP_DISTANCE_HALF 250  

// 모터 PWM 오프셋 값
#define MOTOR_PWM_OFFSET 10

// 초음파 센서 핀 정의
#define FTRIGGER_PIN 3
#define FECHO_PIN 2
#define LTRIGGER_PIN 18
#define LECHO_PIN 19
#define RTRIGGER_PIN 14
#define RECHO_PIN 15

// 초음파 센서 초기화
NewPing sonar_front(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE);
NewPing sonar_left(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE);
NewPing sonar_right(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE);

// 센서 값 변수
float front_sonar = 0.0;
float left_sonar = 0.0;
float right_sonar = 0.0;

// 모터 제어 핀
#define ENR 6
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 8
#define ENL 7

// 현재 미로 상태 변수
int maze_status = 0;

void setup() 
{
  // 모터 제어 핀을 출력으로 설정
  pinMode(ENR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENL, OUTPUT);
  
  // 시리얼 통신을 115200의 속도로 초기화
  Serial.begin(115200);   
}

// 모터 A(왼쪽 모터)를 제어하는 함수
void motor_A_control(int direction_a, int motor_speed_a) 
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENR, motor_speed_a);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENR, motor_speed_a);
  }
}

// 모터 B(오른쪽 모터)를 제어하는 함수
void motor_B_control(int direction_b, int motor_speed_b) 
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENL, motor_speed_b);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENL, motor_speed_b);
  }
}

// 센서 값을 기반으로 현재 미로 상태를 확인하는 함수
void check_maze_status(void)
{
  // 왼쪽, 오른쪽, 앞쪽 센서 값이 각각 특정 범위 내에 있는지 확인
  // 왼쪽과 오른쪽의 거리 값이 WALL_GAP_DISTANCE 이하이고, 앞쪽의 거리 값이 WALL_GAP_DISTANCE_HALF 이하일 경우
  if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    // 이 조건을 만족하면 미로 상태를 4로 설정하고 시리얼 모니터에 출력
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  // 왼쪽과 오른쪽의 거리 값이 WALL_GAP_DISTANCE 이하이고, 앞쪽의 거리 값이 WALL_GAP_DISTANCE_HALF 이상일 경우
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= WALL_GAP_DISTANCE_HALF))
  {
    // 이 조건을 만족하면 미로 상태를 1로 설정하고 시리얼 모니터에 출력
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  // 왼쪽의 거리 값이 WALL_GAP_DISTANCE 이하이고, 앞쪽의 거리 값이 WALL_GAP_DISTANCE_HALF 이하일 경우
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    // 이 조건을 만족하면 미로 상태를 2로 설정하고 시리얼 모니터에 출력
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  // 오른쪽의 거리 값이 WALL_GAP_DISTANCE 이하이고, 앞쪽의 거리 값이 WALL_GAP_DISTANCE_HALF 이하일 경우
  else if((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    // 이 조건을 만족하면 미로 상태를 3로 설정하고 시리얼 모니터에 출력
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else
  {
    // 위 조건들을 모두 만족하지 않을 경우 미로 상태를 0으로 설정하고 시리얼 모니터에 출력
    maze_status = 0;
    Serial.println("maze_status = 0");
  }
}

// 벽 충돌을 피하고 모터 속도를 조정하는 함수
void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.5; // 수정할 비례 상수
  int right_pwm = 0;
  int left_pwm = 0;
  
  // 오른쪽과 왼쪽 초음파 센서 읽기 값의 차이로 에러 계산
  error = (right_sonar - left_sonar); 
  error = Kp * error;

  // 에러 값을 최대 및 최소 값으로 제한
  if(error >= 50) error = 50;
  if(error <= -50) error = -50;

  // 에러를 기반으로 PWM 값을 조정합니다.
  right_pwm = base_speed - error; 
  left_pwm = base_speed + error;

  // PWM 값이 유효한 범위 내에 있는지 확인
  if(left_pwm <= 0) left_pwm = 0;
  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm >= 255) left_pwm = 255;
  if(right_pwm >= 196) right_pwm = 200;

  // 조정된 PWM 값으로 모터를 제어
  motor_A_control(HIGH, right_pwm); // 오른쪽 모터 전진
  motor_B_control(HIGH, left_pwm); // 왼쪽 모터 전진
}

void loop() 
{
  // 센서 값을 읽고 밀리미터 단위로 변환
  front_sonar = sonar_front.ping_cm() * 10;
  left_sonar = sonar_left.ping_cm() * 10;
  right_sonar = sonar_right.ping_cm() * 10;

  // 읽기 값이 없는 경우 센서 값을 최대 거리로 설정
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10;
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE * 10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;

  // 시리얼 모니터에 센서 값을 출력
  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  // 센서 값을 기반으로 현재 미로 상태 확인
  check_maze_status();

  // 미로 상태에 따른 동작을 수행합니다.
  if(maze_status == 4)
  {
    // 정지 후 180도 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(300);

    motor_A_control(HIGH, 225); // 왼쪽 모터 전진
    motor_B_control(LOW, 225); // 오른쪽 모터 후진
    delay(698);

    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(400);

    // 직진
    Serial.println("Go Straight");
    motor_A_control(HIGH, 255); 
    motor_B_control(HIGH, 255); 
  }
  else if(maze_status == 1)
  {
    // 직진하면서 충돌 회피
    Serial.println("Run straight");
    wall_collision_avoid(255);
  } 
  else if(maze_status == 3)
  {
    // 정지 후 왼쪽으로 90도 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(300);

    motor_A_control(HIGH, 255); 
    motor_B_control(LOW, 90); 
    delay(415);

    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(300);
  }
  else if(maze_status == 2)
  {
    // 정지 후 오른쪽으로 90도 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(300);
    
    motor_A_control(LOW, 165); 
    motor_B_control(HIGH, 225); 
    delay(290);

    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(300);
  } 
  else
  {
    // 직진
    Serial.println("Go Straight");
    motor_A_control(HIGH, 255); 
    motor_B_control(HIGH, 255); 
  }
}
