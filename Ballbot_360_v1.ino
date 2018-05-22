#include <SPI.h> //Comunicação Serial
#include <Wire.h> //Comunicação I2C
#include <AccelStepper.h> //Biblioteca para Motores de Passo
#include <MultiStepper.h> //Biblioteca para controlar motores de passo paralelamente
#include <PID_v1.h> //Proporcional Integrativo Derivativo
#include <math.h> //Matemática

float calcAngle (float xVal, float yVal);
float calcMotorSpeed (float V_mag, float V_theta, float motorAngle, float w = 0.0, float R = 1.0);

int motor1Pin = 2;
int motor1Direction = 5;

int motor2Pin = 3;
int motor2Direction = 6;

int motor3Pin = 4;
int motor3Direction = 7;

#define myPI 3.141592

unsigned long t0 = 0;
unsigned long t;
long T = 9;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float temperature = 0.0;

float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

float mx = 0.0;
float my = 0.0;
float mz = 0.0;

float roll = 0.0;
float pitch = 0.0;

float mRoll = 0.0;
float mPitch = 0.0;
float mYaw = 0.0;

float speedStepper1 = 0.0;
float speedStepper2 = 0.0;
float speedStepper3 = 0.0;

float axSum = 0.0;
float aySum = 0.0;
float azSum = 0.0;
float gxSum = 0.0;
float gySum = 0.0;
float gzSum = 0.0;
float axCalibration = -0.1;
float ayCalibration = 0.0;
float azCalibration = 0.0;
float gxCalibration = -0.16;
float gyCalibration = -0.6;
float gzCalibration = 0.31;

float pitchSum = 0.0;
float rollSum = 0.0;
float pitchCalibration = -4.0;
float rollCalibration = 1.0;

// Filtro complementar constante
const float a = 0.02;

float vx = 0.0;
float vy = 0.0;
float vAngle = 0.0;
float vLinear = 0.0;

float X_tilt = 0.0;
float Y_tilt = 0.0;

float X_gyro = 0.0;
float Y_gyro = 0.0;

float X = 0.0;
float Y = 0.0;

double setpointX, inX, outX;
double setpointY, inY, outY;

double kp = 70.0;
double ki = 2.0;
double kd = 0.1;

/////////Variável marolo//////
double gain_marolo = 1.1;
double botSize;

double timer;

MultiStepper Steppers;

//inputX é o pitch e o inputY é o roll
PID xPID(&inX, &outX, &setpointX, kp, ki, kd, DIRECT);
PID yPID(&inY, &outY, &setpointY, kp, ki, kd, DIRECT);
unsigned long i = 0;

AccelStepper Stepper1 (AccelStepper::DRIVER,6,7);
AccelStepper Stepper2 (AccelStepper::DRIVER,8,9);
AccelStepper Stepper3 (AccelStepper::DRIVER,10,11);

void setup() {

  Serial.begin(115200);
  Wire.begin();

  setupMPU();
  // Inicializa IMU

  // Inicializa configurações PID
  setpointX = 0.0;
  setpointY = 0.0;

  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-255, 255);
  xPID.SetSampleTime(T);

  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-255, 255);
  yPID.SetSampleTime(T); // geralmente 200ms. freq.em milli

  pinMode(motor1Direction, OUTPUT);
  pinMode(motor2Direction, OUTPUT);
  pinMode(motor3Direction, OUTPUT);

  Stepper1.setEnablePin(8);
  Stepper2.setEnablePin(9);
  Stepper3.setEnablePin(10);

  Steppers.addStepper(Stepper1);
  Steppers.addStepper(Stepper2);
  Steppers.addStepper(Stepper3);
  
}

void loop() {
  recordRegisters();

  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.


  roll = atan2(ay, az)*180.0/myPI + rollCalibration;
  pitch = atan2(-ax, sqrt(ay*ay + az*az))*180.0/myPI + pitchCalibration;

  Y_tilt = -roll; //inverter roll
  X_tilt = -pitch; //inverter pitch

  Y_gyro = gx;
  X_gyro = gy;

  t = millis(); //contador de tempo de funcionamento
  
  //bloqueador de entrada. Só entra depois de algum tempo de funcionamento. Elimina primeiras leituras de cada loop
  if(t - t0 >= T) {
    t0 = t;
    // (1 - a é filtro) * (vx + leitura do gyro Y * ganho)
    vx = (1.0 - a)*(vx + X_gyro*dt) + a*X_tilt; //pitch
    vy = (1.0 - a)*(vy + Y_gyro*dt) + a*Y_tilt; //roll

    inX = vx;
    xPID.Compute();
    inY = vy;
    yPID.Compute();

    vLinear = sqrt (outX*outX + outY*outY); //vetor resultante
    vAngle = calcAngle (outX, outY); //angulo desse vetor resultante

    double speedStepper1 = calcMotorSpeed (vLinear, vAngle, (myPI/2.0), 0.0, 1.0);
    double speedStepper2 = calcMotorSpeed (vLinear, vAngle, myPI + (myPI/6.0), 0.0, 1.0);
    double speedStepper3 = calcMotorSpeed (vLinear, vAngle, 3.0 * (myPI/2.0) + (myPI/3.0), 0.0, 1.0);
    
    double speedSteppers = calcMotorSpeed (vLinear, vAngle, (myPI/2.0), 0.0, 1.0);

    //Calcula a relação diferencial de velocidade do movimento
    double diffStepper1 = cos(180 - vAngle);
    double diffStepper2 = cos(300 - vAngle);
    double diffStepper3 = cos(420 - vAngle);

    //Velocidade individual dos motores
    double velocityStepper1 = diffStepper1 * speedStepper1;
    double velocityStepper2 = diffStepper2 * speedStepper2;
    double velocityStepper3 = diffStepper3 * speedStepper3;

    //Corrigir:
    double convert_vLinear = (atan(90/vLinear) * botSize * gain_marolo);
    //Calcula quantidade de passos para correção
    double Step = (convert_vLinear/(67 * myPI) / (3200));
    //relação de passos por motor
    double StepsStepper1 = diffStepper1 * Step;
    double StepsStepper2 = diffStepper2 * Step;
    double StepsStepper3 = diffStepper3 * Step;

    //
    //
    //
    // calcular relação bola e rodinha
    //
    //
    //

    double Steps1 = (diffStepper1/(1.8/16));
    double Steps2 = (diffStepper2/(1.8/16));
    double Steps3 = (diffStepper3/(1.8/16));

    Stepper1.setMaxSpeed(speedSteppers);
    Stepper2.setMaxSpeed(speedSteppers);
    Stepper3.setMaxSpeed(speedSteppers);

    //MultiStepper
    long positions[3];

    positions[0] = StepsStepper1;
    positions[1] = StepsStepper2;
    positions[2] = StepsStepper3;
    
    Steppers.moveTo (positions);
    Steppers.runSpeedToPosition (); //

  }
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x08); //Setting the gyro to full scale +/- 500deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0x10); //Setting the accel to +/- 8g
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000); //Start communication with the address found during search
  Wire.write(0x1A); //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03); //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission(); //End the transmission with the gyro
}

void recordRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,14); //Request Accel Registers (3B - 40)
  while(Wire.available() < 14);
  ax = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  ay = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  az = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  temperature = Wire.read()<<8|Wire.read();
  gx = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gy = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gz = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

// v_k = vetor resultante * (cos(PI/2) * sen(angulo do movimento) - o contrario invertendo seno com cos) + ganho // w = 0.0 e R = 1.0
float calcMotorSpeed (float V_mag, float V_theta, float motorAngle, float w, float R) {
  float v_k = V_mag*(cos(motorAngle)*sin(V_theta) - sin(motorAngle)*cos(V_theta)) + R*w;
  int sign = (v_k < 0) ? -1 : 1; // se v_k for negativo, -1, se falso 1
  v_k = (abs(v_k) > 255.0) ? sign*255.0 : v_k; //se módulo de v_k > 255, -1 ou 1 * 255, caso contrário, continua v_k
  return v_k;
}

// Calcula o angulo yVal é o output do PID roll e o xVal é o output do PID pitch
float calcAngle (float xVal, float yVal) {
  // Conditionals required due to -pi to pi limit range of arctan calculation
  if(xVal >= 0.0 && yVal >= 0.0)
    return atan(yVal/xVal);
  else if((xVal < 0.0 && yVal >= 0.0) || (xVal < 0.0 && yVal < 0.0))
    return atan(yVal/xVal) + myPI;
  else
    return atan(yVal/xVal) + 2 * myPI;
}

