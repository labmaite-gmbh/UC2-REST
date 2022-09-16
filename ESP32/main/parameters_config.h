/* 
sample config file for ESP WEMOS D1 E32 + CNC Shield
{
  "motXstp": 26,
  "motXdir": 16,
  "motYstp": 25,
  "motYdir": 27,
  "motZstp": 17,
  "motZdir": 14,
  "motAstp": 0,
  "motAdir": 0,
  "motEnable": 12,
  "ledArrPin": 4,
  "ledArrNum": 64,
  "digitalPin1": 0,
  "digitalPin2": 0,
  "analogPin1": 0,
  "analogPin2": 0,
  "analogPin3": 0,
  "laserPin1": 19,
  "laserPin2": 18,
  "laserPin3": 0,
  "dacFake1": 0,
  "dacFake2": 0,
  "identifier": "",
  "ssid": "",
  "PW": ""
}

sample config file for Uc2 Standalone 

led -> LIMX
laser1 -> LIMY
laser2 -> LIMZ
  {
  "task":"/config_set", 
  "motXstp": 2,
  "motXdir": 33,
  "motYstp": 27,
  "motYdir": 16,
  "motZstp": 12,
  "motZdir": 14,
  "motAstp": 22,
  "motAdir": 21,
  "motEnable": 13,
  "ledArrPin": 17,
  "ledArrNum": 16,
  "digitalPin1":10,
  "digitalPin2":11,
  "analogPin1":0,
  "analogPin2":0,
  "analogPin3":0,
  "laserPin1":4,
  "laserPin2":15,
  "laserPin3":0,
  "dacFake1":0,
  "dacFake2":0,
  "identifier": "UC2Standalone",
  "ssid": "Blynk",
  "PW": "12345678", 
  "PS4Mac": "1a:2b:3c:01:01:01"
  }
  {"task":"/config_act", "applyConfig":1}
  {"task":"/config_get"}
  


sample config file for Uc2 Standalone  1,1

  {
  "task":"/config_set", 
  "motXstp": 2,
  "motXdir": 33,
  "motYstp": 27,
  "motYdir": 16,
  "motZstp": 12,
  "motZdir": 14,
  "motAstp": 22,
  "motAdir": 21,
  "motXLim": -1,
  "motYLim": 34,
  "motZLim": 35,
  "motEnable": 13,
  "ledArrPin": 32,
  "ledArrNum": 16,
  "digitalPin1":0,
  "digitalPin2":0,
  "analogPin1":0,
  "analogPin2":0,
  "analogPin3":0,
  "laserPin1":17,
  "laserPin2":4,
  "laserPin3":15,
  "dacFake1":0,
  "dacFake2":0,
  "identifier": "UC2Standalone",
  "ssid": "Blynk",
  "PW": "12345678", 
  "PS4Mac": "1a:2b:3c:01:01:01"
  }
  {"task":"/config_act", "applyConfig":1}
  {"task":"/config_get"}

  
// for cellstorm
led -> LIMX
lens1 -> LIMY
lens2 -> LIMZ
laser -> xlim

  {
  "task":"/config_set", 
  "motXstp": 2,
  "motXdir": 33,
  "motYstp": 27,
  "motYdir": 16,
  "motZstp": 12,
  "motZdir": 14,
  "motAstp": 22,
  "motAdir": 21,
  "motEnable": 13,
  "ledArrPin": 0,
  "ledArrNum": 16,
  "digitalPin1":10,
  "digitalPin2":11,
  "analogPin1":0,
  "analogPin2":0,
  "analogPin3":0,
  "laserPin1":4,
  "laserPin2":15,
  "laserPin3":17,
  "dacFake1":0,
  "dacFake2":0,
  "identifier": "UC2Standalone",
  "ssid": "Blynk",
  "PW": "12345678", 
  "PS4Mac": "1a:2b:3c:01:01:01"
  }
  {"task":"/config_act", "applyConfig":1}
  {"task":"/config_get"}
  


*/

const char* prefNamespace = "UC2";

const char* keyMotXStepPin = "motXstp";
const char* keyMotXDirPin = "motXdir";
const char* keyMotYStepPin = "motYstp";
const char* keyMotYDirPin = "motYdir";
const char* keyMotZStepPin = "motZstp";
const char* keyMotZDirPin = "motZdir";
const char* keyMotAStepPin = "motAstp";
const char* keyMotADirPin = "motAdir";
const char* keyMotEnable = "motEnable";

const char* keyLEDArray = "ledArrPin";
const char* keyLEDNumLEDArray = "ledArrNum";

const char* keyDigital1Pin = "digitalPin1";
const char* keyDigital2Pin = "digitalPin2";

const char* keyAnalog1Pin = "analogPin1";
const char* keyAnalog2Pin = "analogPin2";
const char* keyAnalog3Pin = "analogPin3";

const char* keyLaser1Pin = "laserPin1";
const char* keyLaser2Pin = "laserPin2";
const char* keyLaser3Pin = "laserPin3";

const char* keyDACfake1Pin = "dacFake1";
const char* keyDACfake2Pin = "dacFake2";

const char* keyIdentifier = "identifier";

const char* keyWifiSSID = "ssid";
const char* keyWifiPW = "PW";

const char* keyPS3Mac = "PS3Mac";
const char* keyPS4Mac = "PS4Mac";
