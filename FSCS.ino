#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Servo.h>

// Initialize MPU6050 and Servo objects
MPU6050 mpu;
Servo rudder;     // Rudder controls the yaw
Servo elevator;   // Elevator controls the pitch
Servo aileron;    // Aileron controls the roll

#define INTERRUPT_PIN 2 // Connect INT pin on MPU6050 to D2
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// Flags and variables
bool blinkState = false;
bool dmpReady = false; // True if DMP initialization was successful
uint8_t mpuIntStatus;  // Holds actual interrupt status byte from MPU
uint8_t devStatus;     // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;    // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;          // [w, x, y, z] quaternion container
VectorFloat gravity;   // [x, y, z] gravity vector
float ypr[3];          // [yaw, pitch, roll] container

volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high

// PID constants for yaw, pitch, and roll (adjust the gain as per your system)
float Kp = 2.0;     // Proportional gain
float Ki = 0.5;     // Integral gain
float Kd = 1.0;     // Derivative gain

// Target values
float targetYaw = 0;
float targetPitch = 0;
float targetRoll = 0;

// PID variables
float yawIntegral = 0;
float pitchIntegral = 0;
float rollIntegral = 0;
float lastYawError = 0;
float lastPitchError = 0;
float lastRollError = 0;

unsigned long lastTime = 0;

// Acceleration variables
float axyz[3];   //[Accel_x,Accel_y,Accel_z] container

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // Initialize I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial); // Wait for Leonardo enumeration, others continue immediately

    // Initialize MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Wait for user input
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // Empty buffer
    while (!Serial.available()); // Wait for data
    while (Serial.available() && Serial.read()); // Empty buffer again

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // Factory default for my test chip

    if (devStatus == 0) {
        // Calibration
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        // Enable the DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set DMP Ready flag
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // Error
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Initialize servos
    rudder.attach(9);    // Attach servo to pin 9 for yaw control
    elevator.attach(10); // Attach servo to pin 10 for pitch control
    aileron.attach(11);  // Attach servo to pin 11 for roll control

    // Set servos to middle position initially
    rudder.write(90);
    elevator.write(90);
    aileron.write(90);

    // Configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // Initialize timing for PID calculations
    lastTime = millis();
}

void loop() {
    if (!dmpReady) return;

    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Get the quaternion and calculate yaw, pitch, and roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert yaw, pitch, and roll to degrees
        float yaw = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        // Calculate time difference for PID
        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        // Read accelerometer values
        VectorInt16 accel;
        mpu.getAcceleration(&accel.x, &accel.y, &accel.z);

        // Convert to g's
        axyz[0] = accel.x / 16384.0; //this value is calculated using the gyroscopes resolution
        axyz[1] = accel.y / 16384.0;
        axyz[2] = accel.z / 16384.0;

        // Convert to m/s² (assuming g = 9.81 m/s²)
        axyz[0] *= 9.81;
        axyz[1] *= 9.81;
        axyz[2] *= 9.81;

        // PID calculations for Yaw
        float yawError = targetYaw - yaw;
        yawIntegral += yawError * dt;
        float yawDerivative = (yawError - lastYawError) / dt;
        float yawOutput = Kp * yawError + Ki * yawIntegral + Kd * yawDerivative;
        lastYawError = yawError;

        // PID calculations for Pitch
        float pitchError = targetPitch - pitch;
        pitchIntegral += pitchError * dt;
        float pitchDerivative = (pitchError - lastPitchError) / dt;
        float pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
        lastPitchError = pitchError;

        // PID calculations for Roll
        float rollError = targetRoll - roll;
        rollIntegral += rollError * dt;
        float rollDerivative = (rollError - lastRollError) / dt;
        float rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
        lastRollError = rollError;

        // Map PID outputs to servo angles (0-180 degrees)
        int rudderPos = map(yawOutput, -180, 180, 0, 180);
        int elevatorPos = map(pitchOutput, -90, 90, 0, 180); 
        int aileronPos = map(rollOutput, -180, 180, 0, 180);

        // Write the servo positions
        rudder.write(constrain(rudderPos, 0, 180));
        elevator.write(constrain(elevatorPos, 0, 180));
        aileron.write(constrain(aileronPos, 0, 180));

        // Blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        // Print Yaw, Pitch, and Roll for debugging
        Serial.print("Yaw: "); Serial.print(yaw);
        Serial.print(" Pitch: "); Serial.print(pitch);
        Serial.print(" Roll: "); Serial.print(roll);
        Serial.print(" Accel X: "); Serial.print(axyz[0]);
        Serial.print(" Accel Y: "); Serial.print(axyz[1]);
        Serial.print(" Accel Z: "); Serial.println(axyz[2]);
    }
}