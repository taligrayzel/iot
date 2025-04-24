// Define motor driver pins
#define IN_A_1 22          // Motor  IN1
#define IN_A_2 23          // Motor  IN2
#define IN_B_1 25          // Motor  IN1
#define IN_B_2 26          // Motor  IN2

// Define encoder pins
#define ENC_A_1 32    // Magnetic Encoder Channel A
#define ENC_A_2 33    // Channel B
#define ENC_B_1 34    // Magnetic Encoder Channel A
#define ENC_B_2 35    // Channel B

#define SPEC_DEG 1080

#define GEAR_RATIO 100



class Motor {
  const int in1, in2;
  const int enc1, enc2;
  const int gearR;
  volatile int lastEncoded; // Here updated value of encoder store.
  volatile long encoderValue; // Raw encoder value
public:
  void updateEncoder(){
    // int MSB = digitalRead(enc1); //MSB = most significant bit
    // int LSB = digitalRead(enc2); //LSB = least significant bit
    // Serial.print(MSB);
    // Serial.println(LSB);

    // int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    // int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

    // if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
    // if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

    // lastEncoded = encoded; //store this value for next time
    lastEncoded++;
  }

  Motor(int in1, int in2, int enc1, int enc2, int gearR) : in1(in1), in2(in2), enc1(enc1), enc2(enc2), gearR(gearR) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enc1, INPUT_PULLUP);
    pinMode(enc2, INPUT_PULLUP);
    digitalWrite(enc1, HIGH); //turn pullup resistor on
    digitalWrite(enc2, HIGH); //turn pullup resistor on
    lastEncoded = 0;
    encoderValue = 0;
  }

  void forward() {
    Serial.print("Motor In Pins:");
    Serial.print("Motor In1: ");
    Serial.print(in1);
    Serial.print(", In2: ");
    Serial.println(in2);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  void reverse() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  void stop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  double get_deg(){
    return (double)lastEncoded/gearR;
  }
  void clear_movement(){
    lastEncoded = 0;
    encoderValue = 0;  
  }
};

Motor* encoderMap[40] = { nullptr }; // Assuming max 40 GPIOs (ESP32)

// Step 1: List active encoder pins
#define ENCODER_PINS \
  X(ENC_A_1)         \
  X(ENC_A_2)         \
  X(ENC_B_1)         \
  X(ENC_B_2)

// Step 2: Generate ISRs only for listed pins
#define EXPAND(x) x
#define X(PIN) \
  void IRAM_ATTR encoderISR##PIN() { \
    if (encoderMap[EXPAND(PIN)]) encoderMap[EXPAND(PIN)]->updateEncoder(); \
  }
ENCODER_PINS
#undef X


// Step 3: Function to attach all listed ISRs
void attachAllEncoderInterrupts() {
  #define X(PIN) attachInterrupt(digitalPinToInterrupt(PIN), encoderISR##PIN, CHANGE);
  ENCODER_PINS
  #undef X
}

//Motor mt1(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2, GEAR_RATIO);
Motor mt2(IN_B_1, IN_B_2, ENC_B_1, ENC_B_2, GEAR_RATIO);

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setup: Starting...");

  // encoderMap[ENC_A_1] = &mt1;
  // encoderMap[ENC_A_2] = &mt1;
  encoderMap[ENC_B_1] = &mt2;
  encoderMap[ENC_B_2] = &mt2;

  Serial.println("Setup: Attaching interrupts");
  attachAllEncoderInterrupts();

  Serial.println("Setup: Motors forward");
  // mt1.forward();
  mt2.forward();

  Serial.println("Setup: Done");
}

void loop() {
  Serial.println(mt2.get_deg());
  // Serial.println("loop");
  // if (mt1.get_deg() > SPEC_DEG) {
  //   mt1.stop();
  // }

}