/*
 * 
 * Serial protocol looks like so:
 * 
 * {"task": "/state_get"}
 * returns:
 * 
++
{"identifier_name":"UC2_Feather","identifier_id":"V0.1","identifier_date":"2022-02-04","identifier_author":"BD"}
--



turn on the laser:
{"task": "/laser_act", "LASERid":1, "LASERval":2}

move the motor
{"task": "/motor_act", "axis":1, "speed":1000, "position":1000, "isabsolute":1, "isblocking":1}

 */

#define DEBUG 1
// CASES:
// 1 Arduino -> Serial only
// 2 ESP32 -> Serial only
// 3 ESP32 -> Wifi only
// 4 ESP32 -> Wifi + Serial ?

// load configuration
#define ARDUINO_SERIAL
//#define ESP32_SERIAL
//#define ESP32_WIFI
//#define ESP32_SERIAL_WIFI

#ifdef ARDUINO_SERIAL
#define IS_SERIAL
#define IS_ARDUINO
#endif

#ifdef ESP32_SERIAL
#define IS_SERIAL
#define IS_ESP32
#endif

#ifdef ESP32_WIFI
#define IS_WIFI
#define IS_ESP32
#endif

#ifdef ESP32_SERIAL_WIFI
#define IS_WIFI
#define IS_SERIAL
#define IS_ESP32
#endif


// load modules
# ifdef IS_ESP32
#define IS_DAC // ESP32-only
#define IS_PS3 // ESP32-only
#endif
#define IS_LASER 
//#define IS_MOTOR



#define BAUDRATE 115200

/*
   START CODE HERE
*/


#ifdef IS_WIFI
#include <WiFi.h>
#include <WebServer.h>
#include "wifi_parameters.h"
#endif


#include <ArduinoJson.h>
//#include <AccelStepper.h>
#include "A4988.h"
#include "motor_parameters.h"
#include "LASER_parameters.h"



//Where the JSON for the current instruction lives
#ifdef IS_ARDUINO
StaticJsonDocument<400> jsonDocument;
//DynamicJsonDocument jsonDocument(400);
#else
char buffer[2500];
DynamicJsonDocument jsonDocument(2048);
#endif
String output;


#ifdef IS_WIFI && IS_ESP32
WebServer server(80);
#endif

#ifdef IS_DAC
#include "dac_Module.h"
dac_Module *dac = new dac_Module();
#endif

#ifdef IS_PS3
#include <Ps3Controller.h>
#endif

/*

   Register devices

*/
#ifdef IS_MOTOR
// https://www.pjrc.com/teensy/td_libs_AccelStepper.html
A4988 stepper_X(FULLSTEPS_PER_REV_X, DIR_X, STEP_X, SLEEP, MS1, MS2, MS3);
A4988 stepper_Y(FULLSTEPS_PER_REV_Y, DIR_Y, STEP_Y, SLEEP, MS1, MS2, MS3);
A4988 stepper_Z(FULLSTEPS_PER_REV_Z, DIR_Z, STEP_Z, SLEEP, MS1, MS2, MS3);
//AccelStepper stepper_X = AccelStepper(AccelStepper::DRIVER, STEP_X, DIR_X);
//AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, STEP_Y, DIR_Y);
//AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, STEP_Z, DIR_Z);
#endif

/*
   Register functions
*/
const String laser_act_endpoint = "/laser_act";
const String laser_set_endpoint = "/laser_set";
const String laser_get_endpoint = "/laser_get";
const String motor_act_endpoint = "/motor_act";
const String motor_set_endpoint = "/motor_set";
const String motor_get_endpoint = "/motor_get";
const String dac_act_endpoint = "/dac_act";
const String dac_set_endpoint = "/dac_set";
const String dac_get_endpoint = "/dac_get";
const String state_act_endpoint = "/state_act";
const String state_set_endpoint = "/state_set";
const String state_get_endpoint = "/state_get";

/*
   Setup
*/
void setup(void)
{
  // Start Serial
  Serial.begin(BAUDRATE);
  Serial.println("Start");

  // connect to wifi if necessary
#ifdef IS_WIFI && IS_ESP32
  connectToWiFi();
  setup_routing();
#endif


Serial.println(state_act_endpoint);
Serial.println(state_get_endpoint);
Serial.println(state_set_endpoint);


#ifdef IS_MOTOR
  /*
     Motor related settings
  */
  Serial.println("Setting Up Motors");
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);

  Serial.println("Setting Up Motor X");
  stepper_X.begin(RPM);
  stepper_X.enable();
  stepper_X.setMicrostep(1);
  stepper_X.move(100);
  stepper_X.move(-100);

  Serial.println("Setting Up Motor X");
  stepper_Y.begin(RPM);
  stepper_Y.enable();
  stepper_Y.setMicrostep(1);
  stepper_Y.move(100);
  stepper_Y.move(-100);

  Serial.println("Setting Up Motor X");
  stepper_Z.begin(RPM);
  stepper_Z.enable();
  stepper_Z.setMicrostep(1);
  stepper_Z.move(100);
  stepper_Z.move(-100);

  /*
    stepper_X.setMaxSpeed(MAX_VELOCITY_X_mm * steps_per_mm_X);
    stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm * steps_per_mm_Y);
    stepper_Z.setMaxSpeed(MAX_VELOCITY_Z_mm * steps_per_mm_Z);

    stepper_X.setAcceleration(MAX_ACCELERATION_X_mm * steps_per_mm_X);
    stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm * steps_per_mm_Y);
    stepper_Z.setAcceleration(MAX_ACCELERATION_Z_mm * steps_per_mm_Z);

    stepper_X.enableOutputs();
    stepper_Y.enableOutputs();
    stepper_Z.enableOutputs();
  */
#endif

#ifdef IS_PS3
  Serial.println("Connnecting to the PS3 controller, please please the magic round button in the center..");
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("01:02:03:04:05:06");
  Serial.println("PS3 controler is set up.");
#endif


#ifdef IS_LASER
  Serial.println("Setting Up LASERs");
  // switch of the LASER directly
  pinMode(LASER_PIN_1, OUTPUT);
  pinMode(LASER_PIN_2, OUTPUT);
  pinMode(LASER_PIN_3, OUTPUT);
  digitalWrite(LASER_PIN_1, LOW);
  digitalWrite(LASER_PIN_2, LOW);
  digitalWrite(LASER_PIN_3, LOW);

  #ifdef IS_ESP32
  /* setup the PWM ports and reset them to 0*/
  ledcSetup(PWM_CHANNEL_LASER_1, pwm_frequency, pwm_resolution);
  ledcAttachPin(LASER_PIN_1, PWM_CHANNEL_LASER_1);
  ledcWrite(PWM_CHANNEL_LASER_1, 10000); delay(500);
  ledcWrite(PWM_CHANNEL_LASER_1, 0);

  ledcSetup(PWM_CHANNEL_LASER_2, pwm_frequency, pwm_resolution);
  ledcAttachPin(LASER_PIN_2, PWM_CHANNEL_LASER_2);
  ledcWrite(PWM_CHANNEL_LASER_2, 10000); delay(500);
  ledcWrite(PWM_CHANNEL_LASER_2, 0);

  ledcSetup(PWM_CHANNEL_LASER_3, pwm_frequency, pwm_resolution);
  ledcAttachPin(LASER_PIN_3, PWM_CHANNEL_LASER_3);
  ledcWrite(PWM_CHANNEL_LASER_3, 10000); delay(500);
  ledcWrite(PWM_CHANNEL_LASER_3, 0);
  #endif
#endif

#ifdef IS_DAC
  Serial.println("Setting Up DAC");
  dac->Setup(dac_CHANNEL_1, 0, 1000, 0, 0, 2);
  //delay(1000);
  //dac->Stop(dac_CHANNEL_1);
#endif


  // list modules
#ifdef IS_SERIAL
  Serial.println("IS_SERIAL");
#endif
#ifdef IS_WIFI
  Serial.println("IS_WIFI");
#endif
#ifdef IS_ARDUINO
  Serial.println("IS_ARDUINO");
#endif
#ifdef IS_ESP32
  Serial.println("IS_ESP32");
#endif
#ifdef IS_PS3
  Serial.println("IS_PS3");
#endif
#ifdef IS_DAC
  Serial.println(dac_act_endpoint);
  Serial.println(dac_get_endpoint);
  Serial.println(dac_set_endpoint);
  delay(50);
#endif
#ifdef IS_MOTOR
  Serial.println(motor_act_endpoint);
  Serial.println(motor_get_endpoint);
  Serial.println(motor_set_endpoint);
  delay(50);
#endif
#ifdef IS_LASER
  Serial.println(laser_act_endpoint);
  Serial.println(laser_get_endpoint);
  Serial.println(laser_set_endpoint);
  delay(50);
#endif
}




void loop() {
#ifdef IS_SERIAL
  if (Serial.available()) {
    DeserializationError error = deserializeJson(jsonDocument, Serial);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    //if (DEBUG) serializeJsonPretty(jsonDocument, Serial);
    String task = jsonDocument["task"];

    /*if (task == "null") return;
    if (DEBUG) {
      Serial.print("TASK: "); 
      Serial.println(String(jsonDocument["task"]));
    }
    */

    // Return state
    if (task == state_act_endpoint)
      state_act_fct(jsonDocument);
    if (task == state_set_endpoint)
      state_set_fct(jsonDocument);
    if (task == state_get_endpoint)
      state_get_fct(jsonDocument);

#ifdef IS_MOTOR
    if (task == motor_act_endpoint) {
      motor_act_fct(jsonDocument);
    }
    else if (task == motor_set_endpoint)
      motor_set_fct(jsonDocument);
    else if (task == motor_get_endpoint)
      jsonDocument = motor_get_fct(jsonDocument);
#endif

#ifdef IS_DAC
    else if (task == dac_act_endpoint)
      dac_act_fct(jsonDocument);
    else if (task == dac_set_endpoint)
      dac_set_fct(jsonDocument);
    else if (task == dac_get_endpoint)
      jsonDocument = dac_get_fct(jsonDocument);
#endif


#ifdef IS_LASER
    else if (task == laser_act_endpoint)
      jsonDocument = LASER_act_fct(jsonDocument);
    else if (task == laser_set_endpoint)
      jsonDocument = LASER_get_fct(jsonDocument);
    else if (task == laser_get_endpoint)
      jsonDocument = LASER_set_fct(jsonDocument);
#endif

    // Send JSON information back
    Serial.println("++");
    serializeJson(jsonDocument, Serial);
    Serial.println();
    Serial.println("--");

    jsonDocument.clear();


  }
#endif


#ifdef IS_PS3
  control_PS3();
#endif
  /*
    stepper_X.run();
    stepper_Y.run();
    stepper_Z.run();
  */
#ifdef IS_WIFI && IS_ESP32
  server.handleClient();
#endif

}


/*


   Define Endpoints


*/

#ifdef IS_WIFI && IS_ESP32
void setup_routing() {
  // GET
  //  server.on("/temperature", getTemperature);
  //server.on("/env", getEnv);
  // https://www.survivingwithandroid.com/esp32-rest-api-esp32-api-server/

  server.on(state_act_endpoint, HTTP_POST, state_act_fct_http);
  server.on(state_get_endpoint, HTTP_POST, state_get_fct_http);
  server.on(state_set_endpoint, HTTP_POST, state_set_fct_http);

#ifdef IS_MOTOR
  // POST
  server.on(motor_act_endpoint, HTTP_POST, motor_act_fct_http);
  server.on(motor_get_endpoint, HTTP_POST, motor_get_fct_http);
  server.on(motor_set_endpoint, HTTP_POST, motor_set_fct_http);
#endif

#ifdef IS_DAC
  server.on(dac_act_endpoint, HTTP_POST, dac_act_fct_http);
  server.on(dac_get_endpoint, HTTP_POST, dac_get_fct_http);
  server.on(dac_set_endpoint, HTTP_POST, dac_set_fct_http);
#endif

#ifdef IS_LASER
  server.on(laser_act_endpoint, HTTP_POST, LASER_act_fct_http);
  server.on(laser_get_endpoint, HTTP_POST, LASER_get_fct_http);
  server.on(laser_set_endpoint, HTTP_POST, LASER_set_fct_http);
#endif

  // start server
  server.begin();
}
#endif
