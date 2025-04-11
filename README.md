# SYSTEM-MEASUREMENTS
• Two main functions: 
• setup() – runs once at start (pin setup, etc.) 
• loop() – keeps running (main code) 
�
�
 Important Functions: 
• pinMode(pin, mode) – sets pin as input/output 
• digitalWrite(pin, HIGH/LOW) – set digital pin ON/OFF 
• digitalRead(pin) – read digital input 
• analogRead(pin) / analogWrite(pin, value) – analog input/output 
• delay(ms) – wait for given milliseconds 
• print() / println() – print values to serial monitor 
�
�
 Pins Overview: 
• Digital Pins (0–13) – Can be used as input or output. 
• Analog Pins (A0–A5) – Used for analog sensor inputs. 
• PWM Pins (like 3, 5, 6, 9, 10, 11) – Used to control devices like LEDs, motors with 
variable voltage. 
⸻ 
○ LED traffic 
void setup() 
{ 
pinMode(7,OUTPUT); 
pinMode(11,OUTPUT); 
pinMode(13,OUTPUT); 
digitalWrite(7,LOW); 
digitalWrite(11,LOW); 
digitalWrite(13,LOW); 
} 
void loop() 
{ 
digitalWrite(7,HIGH); 
delay(5000); 
digitalWrite(7,LOW); 
digitalWrite(11,HIGH); 
delay(5000); 
digitalWrite(11,LOW); 
digitalWrite(13,HIGH); 
delay(5000); 
digitalWrite(13,LOW); 
} 
○ Voltage Measurement 
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x3F, 16, 2); // I2C LCD address 
void setup() { 
lcd.init();         
lcd.clear();        
// Initialize LCD 
// Clear previous display 
lcd.backlight();    // Turn on backlight 
  lcd.setCursor(4, 0); 
  lcd.print("Hello world!"); 
} 
void loop() { 
  int digcount = analogRead(A3);           // Read analog voltage 
  float Vout = digcount * 5.0 / 1023;      // Convert to voltage 
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("Voltage is"); 
  lcd.setCursor(0, 1); 
  lcd.print(Vout); 
  delay(5000);                             // Refresh every 5 sec 
} 
○ Resistance Measurement 
         +5V (Arduino) 
            | 
            | 
           R1   <-- Unknown resistor 
            | 
            |---- A0 (Analog pin) 
            | 
           R2   <-- Known resistor (1kΩ) 
            | 
           GND (Arduino) 
// Known resistor value in ohms (1kΩ) 
float R2 = 1000.0; 
float Vin = 5.0;  // Supply voltage from Arduino 
float Vout = 0.0; 
int adcValue = 0; 
float R1 = 0.0; 
void setup() { 
  Serial.begin(9600);       // Start serial communication 
} 
void loop() { 
  adcValue = analogRead(A0);               // Read voltage at middle of divider 
  Vout = (adcValue * Vin) / 1023.0;        // Convert ADC value to voltage 
  // Apply voltage divider formula to find R1 
  R1 = (R2 * Vin / Vout) - R2; 
  Serial.print("Measured Resistance (R1) = "); 
  Serial.print(R1); 
  Serial.println(" ohms"); 
  delay(5000);   // Wait 5 seconds before next reading 
} 
○ Capacitance Measurement 
                 +5V (from Arduino, via digital pin 12) 
                   | 
                   | 
                  [R]  ← Known resistor (e.g., 10kΩ) 
                   | 
                   |-------- A0 (Analog input) 
                   | 
                  [C]  ← Unknown Capacitor 
                   | 
                 GND (Arduino) 
unsigned long T=0; 
unsigned long R=10000;   // 10k ohm resistor 
float C=0.00; 
void setup() { 
  Serial.begin(9600);     // Start serial monitor 
  pinMode(12, OUTPUT);    // Pin to charge the capacitor 
} 
void loop() { 
  digitalWrite(12, HIGH);          // Start charging 
  unsigned long strtime = micros(); // Start timing 
  while (analogRead(A0) <= 646) {   // Wait till voltage reaches 3.16V 
  } 
  digitalWrite(12, LOW);           // Stop charging 
  T = micros() - strtime;          // Time taken 
  C = float(T) / R;                // Capacitance in farads 
  Serial.println(C);               // Print to monitor 
  delay(5000);                     // Wait before next reading 
} 
○ Freaquency measure  
void setup() { 
  pinMode(12, INPUT);         // Pin to read square wave 
  Serial.begin(9600); 
} 
void loop() { 
  int Ton = pulseIn(12, HIGH);   // Time HIGH 
  int Toff = pulseIn(12, LOW);   // Time LOW 
  int T = Ton + Toff;            // Total time 
  float F = 1000000.0 / T;       // Frequency in Hz 
  Serial.println("Frequency is "); 
  Serial.print(F); 
  Serial.println("   Hz"); 
  delay(2000); 
} 
○ Ditance Measurement 
int dist_cm = 0; 
long readUltrasonicDistance(int triggerPin, int echoPin) { 
  pinMode(triggerPin, OUTPUT); 
  digitalWrite(triggerPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(triggerPin, HIGH);  // Send 10us pulse 
  delayMicroseconds(10); 
  digitalWrite(triggerPin, LOW); 
  pinMode(echoPin, INPUT); 
  return pulseIn(echoPin, HIGH);   // Measure time for pulse to return 
} 
void setup() { 
  Serial.begin(9600); 
  pinMode(12, OUTPUT); // Buzzer 
} 
void loop() { 
  dist_cm = 0.0173 * readUltrasonicDistance(4, 8); // Distance in cm 
  Serial.print(dist_cm); 
  Serial.println("cm"); 
  if (dist_cm < 20) {             // If object is closer than 20 cm 
    digitalWrite(12, HIGH);       // Turn buzzer ON 
    delay(100); 
  } else { 
    digitalWrite(12, LOW);        // Turn buzzer OFF 
    delay(1000); 
  } 
} 
○ T ,P,A measurement 
#include <Adafruit_BMP085.h>        // Include BMP180 library 
Adafruit_BMP085 bmp;                // Create bmp object 
void setup() { 
  Serial.begin(9600); 
  if (!bmp.begin()) { 
    Serial.println("Could not find a valid BMP085 sensor, check wiring!"); 
  } 
} 
void loop() { 
  Serial.print("Temperature = "); 
  Serial.print(bmp.readTemperature());     // °C 
  Serial.println(" *C"); 
  Serial.print("Pressure = "); 
  Serial.print(bmp.readPressure());        // in Pascal 
  Serial.println(" Pa"); 
  Serial.print("Altitude = "); 
  Serial.print(bmp.readAltitude());        // in meters 
  Serial.println(" meters"); 
  Serial.println(); 
  delay(500); 
} 
○ Temp and humidity measurement 
#include<DHT.h> 
#define DHTPIN 8          // DHT signal pin connected to Arduino pin 8 
#define DHTTYPE DHT11     // Define sensor type 
DHT dht_obj(DHTPIN, DHTTYPE); // Create sensor object 
void setup() { 
  Serial.begin(9600); 
  dht_obj.begin();            // Start the sensor 
} 
void loop() { 
  float temp = dht_obj.readTemperature(); 
  Serial.print("Temperature: "); 
  Serial.println(temp, 1);    // 1 decimal place 
  float rel_hum = dht_obj.readHumidity(); 
  Serial.print("Humidity: "); 
  Serial.println(rel_hum, 1); 
  delay(2000);                // Wait 2 seconds before next reading 
} 
○ Moisture reading 
const int moisturePin=A0; 
void setup() 
{ 
  Serial.begin(9600); 
  pinMode(7, OUTPUT); 
} 
void loop() 
{ 
  digitalWrite(7,HIGH); 
  delay(100); 
  int moistureValue=analogRead(moisturePin); 
  Serial.print("moisture Level:"); 
  Serial.println(moistureValue); 
  delay(1000); 
  if(moistureValue>900) 
    Serial.print("Soil Is Dry"); 
  else if(moistureValue<450) # the value may vary between (300 -450) 
    Serial.print("Soil is extremelyWet"); 
  else 
Serial.print("Soil medium Wet”); 
 delay(2000); 
digitalWrite(7,LOW); 
} 
○ Event Counter 
int IRSensor = 9; // connect IR sensor module to Arduino pin 9 
int LED = 7; // connect LED to Arduino pin 7 
int c=0; 
  void setup() 
  { 
  Serial.begin(9600); 
  Serial.println("Serial Working"); // Test to check if serial is working or not 
  pinMode(IRSensor, INPUT); // IR Sensor pin INPUT 
  pinMode(LED, OUTPUT); // LED Pin Output 
} 
void loop() 
{ 
   int sensorStatus = digitalRead(IRSensor); // Set the GPIO as Input 
    if (sensorStatus == 0) // Check if the pin high or not 
  { 
    digitalWrite(LED, HIGH); 
    c++; 
   } 
  else 
   { 
    //else turn off the onboard LED 
    digitalWrite(LED,LOW); // 
     } 
    Serial.println(c); 
  delay(2000); 
} 
 
