#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <time.h>

#define TdsSensorPin 33
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30    // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25; 

// Define water flow sensor parameters
#define SENSOR1 2
#define SENSOR2 4

// Define ultrasonic sensors
const int trigPin = 13;
const int echoPin = 12;

// Define relay pin
#define RELAY_PIN 15

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

// Note: SIM800L TX & RX are connected to Arduino pins 17 & 16 respectively
SoftwareSerial mySerial(17, 16);

// Define WiFi credentials
#define WIFI_SSID "Dialog 4G 231"
#define WIFI_PASSWORD "mathakanA"

// Define Firebase API Key, Project ID, and user credentials
#define API_KEY "AIzaSyADn5BiVUsiChcEvp2fi0BildorSD1KI-w"
#define FIREBASE_PROJECT_ID "aquaguard-uom"
#define USER_EMAIL "dana@gmail.com"
#define USER_PASSWORD "123456"

// Define Firebase Data object, Firebase authentication, and configuration
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 4.5;
volatile byte pulseCount1;
volatile byte pulseCount2;
byte pulse1Sec1 = 0;
byte pulse1Sec2 = 0;
float flowRate1;
float flowRate2;  // New flow rate for second sensor
unsigned int flowMilliLitres1;
unsigned int flowMilliLitres2;  // New flow millilitres for second sensor
unsigned long totalMilliLitres1 = 0;
unsigned long totalMilliLitres2 = 0;  // New total millilitres for second sensor
float monthlybill = 0.00;
float dailybill = 0.00;

// Define your time zone offset in seconds (GMT+5:30)
const long utcOffsetSeconds = 5.5 * 3600;
time_t lastResetTime = 0;

bool isNotified = false;

void IRAM_ATTR pulseCounter1() {
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2() {
  pulseCount2++;
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Set time
  configTime(utcOffsetSeconds, 0, "pool.ntp.org", "time.nist.gov");

  // Print Firebase client version
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Assign the API key
  config.api_key = API_KEY;

  // Assign the user sign-in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Begin Firebase with configuration and authentication
  Firebase.begin(&config, &auth);

  // Send AT command to check communication with SIM800L
  mySerial.println("AT"); // Once the handshake test is successful, it will respond with OK
  updateSerial();

  // Set SMS mode to TEXT
  mySerial.println("AT+CMGF=1"); 
  updateSerial();

  // Initialize waterflow sensor pins
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);

  // Initialize relay pin
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pulseCount1 = 0;
  pulseCount2 = 0;  // New pulse count initialization
  flowRate1 = 0.0;
  flowRate2 = 0.0;  // New flow rate initialization
  flowMilliLitres1 = 0;
  flowMilliLitres2 = 0;  // New flow millilitres initialization
  totalMilliLitres1 = 0;
  totalMilliLitres2 = 0;  // New total millilitres initialization
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR1), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR2), pulseCounter2, FALLING);
}

void loop() {
  // Define the path to the Firestore document
  String documentPath = "ActivationCodes/AG123";

  // Get current time
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  Serial.println(&timeinfo);

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);

  // Open and close valve
  if (distanceCm > 15) {
    digitalWrite(RELAY_PIN, HIGH);
  } else if (distanceCm < 15) {
    digitalWrite(RELAY_PIN, LOW);
  }

  // Analog sampling for TDS sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 200U) { // Adjust sampling interval as needed
    analogSampleTimepoint = millis();

    // Perform analog reading and processing
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }

    // Calculate TDS value
    averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (float)VREF / 4095.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    // Print TDS value
    Serial.print("Voltage: ");
    Serial.print(averageVoltage, 2);
    Serial.print(" V\t");
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");

    // Update the tdsLevel in Firestore
    FirebaseJson content;
    content.set("fields/tdsLevel/doubleValue", tdsValue);

    // Check if tdsLevel exists and update it
    if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(), "tdsLevel")) {
      Serial.printf("Updated tdsLevel in Firestore: %.2f\n", tdsValue);
    } else {
      Serial.println("Failed to update tdsLevel in Firestore.");
      Serial.println(fbdo.errorReason());
    }
  }

  // Fetching the document from Firestore
  if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), "")) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, fbdo.payload().c_str());

    if (!error) {
      // Check if the 'totalMilliLitres1' field exists in the Firestore document
      if (doc["fields"]["totalMilliLitres1"]["integerValue"]) {
        totalMilliLitres1 = doc["fields"]["totalMilliLitres1"]["integerValue"].as<unsigned long>();
      } else {
        totalMilliLitres1 = 0;
      }

      // Check if the 'totalMilliLitres2' field exists in the Firestore document
      if (doc["fields"]["totalMilliLitres2"]["integerValue"]) {
        totalMilliLitres2 = doc["fields"]["totalMilliLitres2"]["integerValue"].as<unsigned long>();
      } else {
        totalMilliLitres2 = 0;
      }

      bool isNotified = false;

      if (doc["fields"]["isNotified"]["booleanValue"]) {
        isNotified = doc["fields"]["isNotified"]["booleanValue"].as<bool>();
      }

      if (timeinfo.tm_hour == 23 && timeinfo.tm_min == 59 && timeinfo.tm_sec >= 59 && difftime(now, lastResetTime) >= 86400) {
        totalMilliLitres2 = 0;
        lastResetTime = now;
        Serial.println("totalMilliLitres1 reset to 0.");

        FirebaseJson content;
        content.set("fields/isNotified/booleanValue", false);

        char formattedTime[20];
        strftime(formattedTime, sizeof(formattedTime), "%Y %b %d", &timeinfo);
        String message = "Daily update from Aqua Guard " + String(formattedTime) + "\nWater Usage: " + String(totalMilliLitres1 / 1000) + "L\nBill Amount: " + String(monthlybill) + "\nThis month water bill so far: " + String(monthlybill);
        sendSMS(message);

        if (isLastDayOfMonth(timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900)) {
          saveMonthlyDataToFirestore(timeinfo, totalMilliLitres1, monthlybill);
          totalMilliLitres1 = 0;
        }
      }

      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
        pulse1Sec1 = pulseCount1;
        pulse1Sec2 = pulseCount2;
        pulseCount1 = 0;
        pulseCount2 = 0;

        flowRate1 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec1) / calibrationFactor;
        flowRate2 = ((1000.0 / (millis() - previousMillis)) * pulse1Sec2) / calibrationFactor;
        previousMillis = millis();
        flowMilliLitres1 = (flowRate1 / 60) * 1000;
        flowMilliLitres2 = (flowRate2 / 60) * 1000;
        totalMilliLitres1 += flowMilliLitres1;
        totalMilliLitres2 += flowMilliLitres2;

        Serial.println(totalMilliLitres1);
        Serial.println(totalMilliLitres2);

        if (!isnan(totalMilliLitres1) && !isnan(totalMilliLitres2)) {
          FirebaseJson content;
          content.set("fields/totalMilliLitres1/integerValue", totalMilliLitres1);
          content.set("fields/totalMilliLitres2/integerValue", totalMilliLitres2);

          if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(), "totalMilliLitres1,totalMilliLitres2")) {
            Serial.printf("Updated totalMilliLitres1 and totalMilliLitres2 in Firestore: %lu, %lu\n", totalMilliLitres1, totalMilliLitres2);
          } else {
            Serial.println("Failed to update totalMilliLitres in Firestore.");
            Serial.println(fbdo.errorReason());
          }
        } else {
          Serial.println("Failed to read ultrasonic sensor data.");
        }
      }

      float totalPoints1 = totalMilliLitres1 / 100.00;
      float totalPoints2 = totalMilliLitres2 / 100.00;

      Serial.println(totalPoints1);
      Serial.println(totalPoints2);

      float monthlyService0_5 = doc["fields"]["rates"]["mapValue"]["fields"]["0-5"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService6_10 = doc["fields"]["rates"]["mapValue"]["fields"]["6-10"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService11_15 = doc["fields"]["rates"]["mapValue"]["fields"]["11-15"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService16_20 = doc["fields"]["rates"]["mapValue"]["fields"]["16-20"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService21_25 = doc["fields"]["rates"]["mapValue"]["fields"]["21-25"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService26_30 = doc["fields"]["rates"]["mapValue"]["fields"]["26-30"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService31_40 = doc["fields"]["rates"]["mapValue"]["fields"]["31-40"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService41_50 = doc["fields"]["rates"]["mapValue"]["fields"]["41-50"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService51_75 = doc["fields"]["rates"]["mapValue"]["fields"]["51-75"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService76_100 = doc["fields"]["rates"]["mapValue"]["fields"]["76-100"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];
      float monthlyService100 = doc["fields"]["rates"]["mapValue"]["fields"]["Over 100"]["mapValue"]["fields"]["monthlyServiceCharge"]["integerValue"];

      float monthly0_5 = doc["fields"]["rates"]["mapValue"]["fields"]["0-5"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly6_10 = doc["fields"]["rates"]["mapValue"]["fields"]["6-10"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly11_15 = doc["fields"]["rates"]["mapValue"]["fields"]["11-15"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly16_20 = doc["fields"]["rates"]["mapValue"]["fields"]["16-20"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly21_25 = doc["fields"]["rates"]["mapValue"]["fields"]["21-25"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly26_30 = doc["fields"]["rates"]["mapValue"]["fields"]["26-30"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly31_40 = doc["fields"]["rates"]["mapValue"]["fields"]["31-40"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly41_50 = doc["fields"]["rates"]["mapValue"]["fields"]["41-50"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly51_75 = doc["fields"]["rates"]["mapValue"]["fields"]["51-75"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly76_100 = doc["fields"]["rates"]["mapValue"]["fields"]["76-100"]["mapValue"]["fields"]["usageCharge"]["integerValue"];
      float monthly100 = doc["fields"]["rates"]["mapValue"]["fields"]["Over 100"]["mapValue"]["fields"]["usageCharge"]["integerValue"];

      int monthlyLimit = doc["fields"]["amount"]["integerValue"];
      int dailyLimit = monthlyLimit / 30;

      if (totalPoints1 >= 0 && totalPoints1 <= 5) {
        monthlybill = (totalPoints1 * monthly0_5) + monthlyService0_5;
      } else if (totalPoints1 > 5 && totalPoints1 <= 10) {
        monthlybill = (5 * monthly0_5) + ((totalPoints1 - 5) * monthly6_10) + monthlyService6_10;
      } else if (totalPoints1 > 10 && totalPoints1 <= 15) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + ((totalPoints1 - 10) * monthly11_15) + monthlyService11_15;
      } else if (totalPoints1 > 15 && totalPoints1 <= 20) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + ((totalPoints1 - 15) * monthly16_20) + monthlyService16_20;
      } else if (totalPoints1 > 20 && totalPoints1 <= 25) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + ((totalPoints1 - 20) * monthly21_25) + monthlyService21_25;
      } else if (totalPoints1 > 25 && totalPoints1 <= 30) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + ((totalPoints1 - 25) * monthly26_30) + monthlyService26_30;
      } else if (totalPoints1 > 30 && totalPoints1 <= 40) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + ((totalPoints1 - 30) * monthly31_40) + monthlyService31_40;
      } else if (totalPoints1 > 40 && totalPoints1 <= 50) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + ((totalPoints1 - 40) * monthly41_50) + monthlyService41_50;
      } else if (totalPoints1 > 50 && totalPoints1 <= 75) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + ((totalPoints1 - 50) * monthly51_75) + monthlyService51_75;
      } else if (totalPoints1 > 75 && totalPoints1 <= 100) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + (25 * monthly51_75) + ((totalPoints1 - 75) * monthly76_100) + monthlyService76_100;
      } else if (totalPoints1 > 100) {
        monthlybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + (25 * monthly51_75) + (25 * monthly76_100) + ((totalPoints1 - 100) * monthly100) + monthlyService100;
      }

      if (totalPoints2 >= 0 && totalPoints2 <= 5) {
        dailybill = (totalPoints2 * monthly0_5) + monthlyService0_5;
      } else if (totalPoints2 > 5 && totalPoints2 <= 10) {
        dailybill = (5 * monthly0_5) + ((totalPoints2 - 5) * monthly6_10) + monthlyService6_10;
      } else if (totalPoints2 > 10 && totalPoints2 <= 15) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + ((totalPoints2 - 10) * monthly11_15) + monthlyService11_15;
      } else if (totalPoints2 > 15 && totalPoints2 <= 20) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + ((totalPoints2 - 15) * monthly16_20) + monthlyService16_20;
      } else if (totalPoints2 > 20 && totalPoints2 <= 25) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + ((totalPoints2 - 20) * monthly21_25) + monthlyService21_25;
      } else if (totalPoints2 > 25 && totalPoints2 <= 30) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + ((totalPoints2 - 25) * monthly26_30) + monthlyService26_30;
      } else if (totalPoints2 > 30 && totalPoints2 <= 40) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + ((totalPoints2 - 30) * monthly31_40) + monthlyService31_40;
      } else if (totalPoints2 > 40 && totalPoints2 <= 50) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + ((totalPoints2 - 40) * monthly41_50) + monthlyService41_50;
      } else if (totalPoints2 > 50 && totalPoints2 <= 75) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + ((totalPoints2 - 50) * monthly51_75) + monthlyService51_75;
      } else if (totalPoints2 > 75 && totalPoints2 <= 100) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + (25 * monthly51_75) + ((totalPoints2 - 75) * monthly76_100) + monthlyService76_100;
      } else if (totalPoints2 > 100) {
        dailybill = (5 * monthly0_5) + (5 * monthly6_10) + (5 * monthly11_15) + (5 * monthly16_20) + (5 * monthly21_25) + (5 * monthly26_30) + (10 * monthly31_40) + (10 * monthly41_50) + (25 * monthly51_75) + (25 * monthly76_100) + ((totalPoints2 - 100) * monthly100) + monthlyService100;
      }

      Serial.print("Total monthlyBill: ");
      Serial.println(monthlybill);

      Serial.print("Total dailyBill: ");
      Serial.println(dailybill);

      FirebaseJson monthlybillContent;
      monthlybillContent.set("fields/monthlyConsumption/doubleValue", monthlybill);

      FirebaseJson dailybillContent;
      dailybillContent.set("fields/dailyConsumption/doubleValue", dailybill);

      if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), monthlybillContent.raw(), "monthlyConsumption")) {
        Serial.println("Successfully updated monthlyConsumption in Firestore.");
      } else {
        Serial.print("Failed to update monthlyConsumption: ");
        Serial.println(fbdo.errorReason());
      }

      if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), dailybillContent.raw(), "dailyConsumption")) {
        Serial.println("Successfully updated dailyConsumption in Firestore.");
      } else {
        Serial.print("Failed to update dailyConsumption: ");
        Serial.println(fbdo.errorReason());
      }

      // Daily limit notification
      if ((totalMilliLitres2 > dailyLimit) && !isNotified) {
        sendSMS("Attention: You've exceeded your daily water usage limit. Please use water carefully.");

        FirebaseJson content;
        content.set("fields/isNotified/booleanValue", true);

        if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(), "isNotified")) {
          Serial.println("Updated isNotified in Firestore.");
        } else {
          Serial.println("Failed to update isNotified in Firestore.");
          Serial.println(fbdo.errorReason());
        }
      }
    } else {
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.println("Failed to get document");
    Serial.println(fbdo.errorReason());
  }
}

void sendSMS(String message) {
  mySerial.println("AT+CMGS=\"+94768574082\"");
  updateSerial();

  mySerial.print(message);
  delay(100);
  updateSerial();

  mySerial.write(26);
  updateSerial();
}

void updateSerial() {
  delay(500);

  while (Serial.available()) {
    mySerial.write(Serial.read());
  }

  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

bool isLastDayOfMonth(int day, int month, int year) {
  if (day < 1 || day > 31 || month < 1 || month > 12) {
    return false;
  }
  if (day == 31 || (day == 30 && (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12)) ||
      (day == 29 && month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) || 
      (day == 28 && month == 2 && (year % 4 != 0 || (year % 100 == 0 && year % 400 != 0)))) {
    return true;
  } else {
    return false;
  }
}

// Function to save the monthly data to Firestore
void saveMonthlyDataToFirestore(struct tm timeinfo, unsigned long totalMilliLitres1, float monthlybill) {
  String monthName;
  char formattedTime[20];

  strftime(formattedTime, sizeof(formattedTime), "%Y %b", &timeinfo);
  monthName = String(formattedTime);

  FirebaseJson monthlyData;
  FirebaseJsonArray monthlyArray;

  monthlyArray.add(monthName);
  monthlyArray.add(totalMilliLitres1);
  monthlyArray.add(monthlybill);

  monthlyData.set("fields/" + monthName + "/arrayValue/values", monthlyArray);

  String historyPath = "ActivationCodes/AG123/history";
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", historyPath.c_str(), monthlyData.raw(), monthName.c_str())) {
    Serial.println("Successfully saved monthly data to Firestore.");
  } else {
    Serial.print("Failed to save monthly data: ");
    Serial.println(fbdo.errorReason());
  }
}
