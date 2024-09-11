class PID {
  private:
    float Kp, Ki, Kd;
    float previousError, integral;
    float integralMin, integralMax; // For windup protection
    unsigned long lastTime; // To track time for derivative calculation
    
  public:
    // Initializing the PID gains and windup protection limits
    PID(float _Kp, float _Ki, float _Kd, float _integralMin = -255.0, float _integralMax = 255.0) {
      Kp = _Kp;
      Ki = _Ki;
      Kd = _Kd;
      integralMin = _integralMin;
      integralMax = _integralMax;
      previousError = 0;
      integral = 0;
      lastTime = millis(); // Initialize lastTime to current time
    }
    
    // Function to compute the PID output
    float compute(float setPoint, float currentSpeed) {
      unsigned long currentTime = millis();
      float timeDelta = (currentTime - lastTime) / 1000.0; // Time in seconds
      if (timeDelta <= 0) timeDelta = 0.001; // Prevent division by zero

      // Calculate error
      float error = setPoint - currentSpeed;
      
      // Calculate integral with windup protection
      integral += error * timeDelta;
      if (integral > integralMax) integral = integralMax;
      else if (integral < integralMin) integral = integralMin;
      
      // Calculate derivative
      float derivative = (error - previousError) / timeDelta;
      
      // Compute PID output
      float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      // Save current values for next iteration
      previousError = error;
      lastTime = currentTime;
      
      return output;
    }
};

// PID class instance with integral windup limits
PID motorPID(2.0, 1.0, 0.5, -255.0, 255.0);

// Variables for motor control
float setPoint = 100.0;
float currentSpeed = 0.0;
float previousSpeed = 0.0;
float alpha = 0.1; // Smoothing factor
int motorPin = 9;

float ExponentialSmoothing(float newSpeed, float previousSpeed) {
    return (alpha * newSpeed) + ((1 - alpha) * previousSpeed);
}

void setup() {
    pinMode(motorPin, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // Simulate getting current speed from motor
    currentSpeed = analogRead(A0) * (255.0 / 1023.0);

    // Apply PID control using the class
    float pidOutput = motorPID.compute(setPoint, currentSpeed);

    // Apply exponential smoothing
    float smoothSpeed = ExponentialSmoothing(pidOutput, previousSpeed);
    previousSpeed = smoothSpeed;

    // Limit the output to PWM range (0 to 255)
    int motorSpeed = constrain(smoothSpeed, 0, 255);

    // Control motor speed
    analogWrite(motorPin, motorSpeed);

    // Print values for debugging
    Serial.print("Current Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" PID Output: ");
    Serial.print(pidOutput);
    Serial.print(" Smoothed Speed: ");
    Serial.println(smoothSpeed);

    delay(100);
}
