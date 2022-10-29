// Custom function accessible by the API
void motor_act_fct() {
  if (DEBUG) Serial.println("motor_act_fct");


  // assign default values to thhe variables
  if (jsonDocument.containsKey("speed0")) {
    mspeed0 = jsonDocument["speed0"];
  }
  else if (jsonDocument.containsKey("speed")) {
    mspeed0 = jsonDocument["speed"];
  }
  if (jsonDocument.containsKey("speed1")) {
    mspeed1 = jsonDocument["speed1"];
  }
  else if (jsonDocument.containsKey("speed")) {
    mspeed1 = jsonDocument["speed"];
  }
  if (jsonDocument.containsKey("speed2")) {
    mspeed2 = jsonDocument["speed2"];
  }
  else if (jsonDocument.containsKey("speed")) {
    mspeed2 = jsonDocument["speed"];
  }
  if (jsonDocument.containsKey("speed3")) {
    mspeed3 = jsonDocument["speed3"];
  }
  else if (jsonDocument.containsKey("speed")) {
    mspeed3 = jsonDocument["speed"];
  }

  if (jsonDocument.containsKey("pos0")) {
    mposition0 = jsonDocument["pos0"];
  }
  else {
    mposition0 = 0;
  }
  if (jsonDocument.containsKey("pos1")) {
    mposition1 = jsonDocument["pos1"];
  }
  else {
    mposition1 = 0;
  }

  if (jsonDocument.containsKey("pos2")) {
    mposition2 = jsonDocument["pos2"];
  }
  else {
    mposition2 = 0;
  }

  if (jsonDocument.containsKey("pos3")) {
    mposition3 = jsonDocument["pos3"];
  }
  else {
    mposition3 = 0;
  }

  if (jsonDocument.containsKey("isabs")) {
    isabs = jsonDocument["isabs"];
  }
  else {
    isabs = 0;
  }

  if (jsonDocument.containsKey("isstop")) {
    isstop = jsonDocument["isstop"];
  }
  else {
    isstop = 0;
  }

  if (jsonDocument.containsKey("isaccel")) {
    isaccel = jsonDocument["isaccel"];
  }
  else {
    isaccel = 1;
  }

  if (jsonDocument.containsKey("isen")) {
    isen = jsonDocument["isen"];
  }
  else {
    isen = 0;
  }

  if (jsonDocument.containsKey("isforever")) {
    isforever = jsonDocument["isforever"];
  }
  else {
    isforever = 0;
  }

  // check if homing is expected
  if (jsonDocument.containsKey("ishomeX")) {
    ishomeX = jsonDocument["ishomeX "];
  }
  else {
    ishomeX  = 0;
  }
  // check if homing is expected
  if (jsonDocument.containsKey("ishomeY")) {
    ishomeY = jsonDocument["ishomeY"];
  }
  else {
    ishomeY  = 0;
  }
  // check if homing is expected
  if (jsonDocument.containsKey("ishomeZ")) {
    ishomeZ = jsonDocument["ishomeZ"];
  }
  else {
    ishomeZ  = 0;
  }

  // check if homing is expected
  if (jsonDocument.containsKey("homePinX")) {
    homePinX = jsonDocument["homePinX"];
    pinMode(homePinX, INPUT_PULLUP);
  }
  if (jsonDocument.containsKey("homePinY")) {
    homePinY = jsonDocument["homePinY"];
    pinMode(homePinY, INPUT_PULLUP);
  }
  if (jsonDocument.containsKey("homePinZ")) {
    homePinZ = jsonDocument["homePinZ"];
    pinMode(homePinZ, INPUT_PULLUP);
  }
  if (jsonDocument.containsKey("homePinA")) {
    homePinA = jsonDocument["homePinA"];
    pinMode(homePinA, INPUT_PULLUP);
  }



  jsonDocument.clear();

  if (DEBUG) {
    Serial.print("speed0 "); Serial.println(mspeed0);
    Serial.print("speed1 "); Serial.println(mspeed1);
    Serial.print("speed2 "); Serial.println(mspeed2);
    Serial.print("speed3 "); Serial.println(mspeed3);
    Serial.print("position0 "); Serial.println(mposition0);
    Serial.print("position1 "); Serial.println(mposition1);
    Serial.print("position2 "); Serial.println(mposition2);
    Serial.print("position3 "); Serial.println(mposition3);
    Serial.print("isabs "); Serial.println(isabs);
    Serial.print("isen "); Serial.println(isen);
    Serial.print("isstop "); Serial.println(isstop);
    Serial.print("isaccel "); Serial.println(isaccel);
    Serial.print("isforever "); Serial.println(isforever);
    Serial.print("ishomeX "); Serial.println(ishomeX);
    Serial.print("ishomeY "); Serial.println(ishomeY);
    Serial.print("ishomeZ "); Serial.println(ishomeZ);
  }

  if (isstop) {
    // Immediately stop the motor
    stepper_A.stop();
    stepper_X.stop();
    stepper_Y.stop();
    stepper_Z.stop();
    isforever = 0;
    setEnableMotor(false);

    POSITION_MOTOR_A = stepper_A.currentPosition();
    POSITION_MOTOR_X = stepper_X.currentPosition();
    POSITION_MOTOR_Y = stepper_Y.currentPosition();
    POSITION_MOTOR_Z = stepper_Z.currentPosition();

    jsonDocument["POSA"] = POSITION_MOTOR_A;
    jsonDocument["POSX"] = POSITION_MOTOR_X;
    jsonDocument["POSY"] = POSITION_MOTOR_Y;
    jsonDocument["POSZ"] = POSITION_MOTOR_Z;
    return;
  }

  if (abs(ishomeX)) {
    doHoming("X", ishomeX);
  }
  if (abs(ishomeY)) {
    doHoming("Y", ishomeY);
  }
  if (abs(ishomeZ)) {
    doHoming("Z", ishomeZ);
  }

  if (IS_PSCONTROLER_ACTIVE)
    IS_PSCONTROLER_ACTIVE = false; // override PS controller settings #TODO: Somehow reset it later?

  // prepare motor to run
  setEnableMotor(true);
  stepper_A.setSpeed(mspeed0);
  stepper_X.setSpeed(mspeed1);
  stepper_Y.setSpeed(mspeed2);
  stepper_Z.setSpeed(mspeed3);
  stepper_A.setMaxSpeed(mspeed0);
  stepper_X.setMaxSpeed(mspeed1);
  stepper_Y.setMaxSpeed(mspeed2);
  stepper_Z.setMaxSpeed(mspeed3);


  if (not isforever) {
    if (isabs) {
      // absolute position coordinates
      stepper_A.moveTo(SIGN_A * mposition0);
      stepper_X.moveTo(SIGN_X * mposition1);
      stepper_Y.moveTo(SIGN_Y * mposition2);
      stepper_Z.moveTo(SIGN_Z * mposition3);
    }
    else {
      // relative position coordinates
      stepper_A.move(SIGN_A * mposition0);
      stepper_X.move(SIGN_X * mposition1);
      stepper_Y.move(SIGN_Y * mposition2);
      stepper_Z.move(SIGN_Z * mposition3);
    }
  }

  if (DEBUG) Serial.println("Start rotation in background");

  POSITION_MOTOR_A = stepper_A.currentPosition();
  POSITION_MOTOR_X = stepper_X.currentPosition();
  POSITION_MOTOR_Y = stepper_Y.currentPosition();
  POSITION_MOTOR_Z = stepper_Z.currentPosition();

  jsonDocument["POSA"] = POSITION_MOTOR_A;
  jsonDocument["POSX"] = POSITION_MOTOR_X;
  jsonDocument["POSY"] = POSITION_MOTOR_Y;
  jsonDocument["POSZ"] = POSITION_MOTOR_Z;
}

void setEnableMotor(bool enable) {
  isBusy = enable;
  digitalWrite(ENABLE_PIN, !enable);
  motor_enable = enable;
}

bool getEnableMotor() {
  return motor_enable;
}


void doHoming(String axis, int signHomging) {
  // FIXME: Move motor until switch is hit...
  long tStart = millis();
  if(DEBUG) Serial.println("Start homing");
  if (axis == "A") {
    stepper_A.setSpeed(signHomging*mspeed0);
    while (millis() - tStart < 1000 or !digitalRead(homePinA)) { // Here we want to check a button
      stepper_A.runSpeed();
    }
    stepper_A.setCurrentPosition(0);
  }
  if (axis == "X") {
    stepper_X.setSpeed(signHomging*mspeed1);
    while (millis() - tStart < 1000 or !digitalRead(homePinX)) { // Here we want to check a button
      stepper_X.runSpeed();
    }
    stepper_X.setCurrentPosition(0);
  }
  if (axis == "Y") {
    stepper_Y.setSpeed(signHomging*mspeed0);
    while (millis() - tStart < 1000 or !digitalRead(homePinY)) { // Here we want to check a button
      stepper_Y.runSpeed();
    }
    stepper_A.setCurrentPosition(0);
  }
  if (axis == "Z") {
    stepper_Z.setSpeed(signHomging*mspeed0);
    while (millis() - tStart < 1000 or !digitalRead(homePinZ)) { // Here we want to check a button
      stepper_Z.runSpeed();
    }
    stepper_Z.setCurrentPosition(0);
  }
  if(DEBUG) Serial.println("Finishing homing");
}

void motor_set_fct() {


  // default value handling
  int axis = -1;
  if (jsonDocument.containsKey("axis")) {
    int axis = jsonDocument["axis"];
  }

  int currentposition = NULL;
  if (jsonDocument.containsKey("currentposition")) {
    int currentposition = jsonDocument["currentposition"];
  }

  int maxspeed = NULL;
  if (jsonDocument.containsKey("maxspeed")) {
    int maxspeed = jsonDocument["maxspeed"];
  }

  int accel = NULL;
  if (jsonDocument.containsKey("accel")) {
    int accel = jsonDocument["accel"];
  }

  int pinstep = -1;
  if (jsonDocument.containsKey("pinstep")) {
    int pinstep = jsonDocument["pinstep"];
  }

  int pindir = -1;
  if (jsonDocument.containsKey("pindir")) {
    int pindir = jsonDocument["pindir"];
  }

  int isen = -1;
  if (jsonDocument.containsKey("isen")) {
    int isen = jsonDocument["isen"];
  }

  int sign = NULL;
  if (jsonDocument.containsKey("sign")) {
    int sign = jsonDocument["sign"];
  }

  // DEBUG printing
  if (DEBUG) {
    Serial.print("axis "); Serial.println(axis);
    Serial.print("currentposition "); Serial.println(currentposition);
    Serial.print("maxspeed "); Serial.println(maxspeed);
    Serial.print("pinstep "); Serial.println(pinstep);
    Serial.print("pindir "); Serial.println(pindir);
    Serial.print("isen "); Serial.println(isen);
    Serial.print("accel "); Serial.println(accel);
    Serial.print("isen: "); Serial.println(isen);
  }


  if (accel >= 0) {
    if (DEBUG) Serial.print("accel "); Serial.println(accel);
    switch (axis) {
      case 0:
        MAX_ACCELERATION_A = accel;
        stepper_A.setAcceleration(MAX_ACCELERATION_A);
        break;
      case 1:
        MAX_ACCELERATION_X = accel;
        stepper_X.setAcceleration(MAX_ACCELERATION_X);
        break;
      case 2:
        MAX_ACCELERATION_Y = accel;
        stepper_Y.setAcceleration(MAX_ACCELERATION_Y);
        break;
      case 3:
        MAX_ACCELERATION_Z = accel;
        stepper_Z.setAcceleration(MAX_ACCELERATION_Z);
        break;
    }
  }

  if (sign != NULL) {
    if (DEBUG) Serial.print("sign "); Serial.println(sign);
    switch (axis) {
      case 0:
        SIGN_A = sign;
        break;
      case 1:
        SIGN_X = sign;
        break;
      case 2:
        SIGN_Y = sign;
        break;
      case 3:
        SIGN_Z = sign;
        break;
    }
  }


  if (currentposition != NULL) {
    if (DEBUG) Serial.print("currentposition "); Serial.println(currentposition);
    switch (axis) {
      case 0:
        POSITION_MOTOR_A = currentposition;
        stepper_A.setCurrentPosition(currentposition);
        break;
      case 1:
        POSITION_MOTOR_X = currentposition;
        stepper_X.setCurrentPosition(currentposition);
        break;
      case 2:
        POSITION_MOTOR_Y = currentposition;
        stepper_Y.setCurrentPosition(currentposition);
      case 3:
        POSITION_MOTOR_Z = currentposition;
        stepper_Z.setCurrentPosition(currentposition);
        break;
    }
  }
  if (maxspeed != 0) {
    switch (axis) {
      case 1:
        stepper_X.setMaxSpeed(maxspeed);
        break;
      case 2:
        stepper_Y.setMaxSpeed(maxspeed);
        break;
      case 3:
        stepper_Z.setMaxSpeed(maxspeed);
        break;
    }
  }


  //if (DEBUG) Serial.print("isen "); Serial.println(isen);
  if (isen != 0 and isen) {
    digitalWrite(ENABLE_PIN, 0);
  }
  else if (isen != 0 and not isen) {
    digitalWrite(ENABLE_PIN, 1);
  }
  jsonDocument.clear();
  jsonDocument["return"] = 1;
}



// Custom function accessible by the API
void motor_get_fct() {
  int axis = jsonDocument["axis"];
  if (DEBUG) Serial.println("motor_get_fct");
  if (DEBUG) Serial.println(axis);

  int mmaxspeed = 0;
  int mposition = 0;
  int pinstep = 0;
  int pindir = 0;
  int sign = 0;
  int mspeed = 0;

  if (axis == -1) {
    //then we ask for position of all axis
    jsonDocument.clear();
    jsonDocument["positionX"] = stepper_X.currentPosition();
    jsonDocument["positionY"] = stepper_Y.currentPosition();
    jsonDocument["positionZ"] = stepper_Z.currentPosition();
    jsonDocument["positionA"] = stepper_A.currentPosition();
  }
  else {
    if (axis == 0) {
      if (DEBUG) Serial.println("AXIS 0");
      mmaxspeed = stepper_A.maxSpeed();
      mspeed = stepper_A.speed();
      POSITION_MOTOR_A = stepper_A.currentPosition();
      mposition = POSITION_MOTOR_A;
      pinstep = STEP_PIN_A;
      pindir = DIR_PIN_A;
      sign = SIGN_A;
    }
    if (axis == 1) {
      if (DEBUG) Serial.println("AXIS 1");
      mmaxspeed = stepper_X.maxSpeed();
      mspeed = stepper_X.speed();
      POSITION_MOTOR_X = stepper_X.currentPosition();
      mposition = POSITION_MOTOR_X;
      pinstep = STEP_PIN_X;
      pindir = DIR_PIN_X;
      sign = SIGN_X;
    }
    if (axis == 2) {
      if (DEBUG) Serial.println("AXIS 2");
      mmaxspeed = stepper_Y.maxSpeed();
      mspeed = stepper_Y.speed();
      POSITION_MOTOR_Y = stepper_Y.currentPosition();
      mposition = POSITION_MOTOR_Y;
      pinstep = STEP_PIN_Y;
      pindir = DIR_PIN_Y;
      sign = SIGN_Y;
    }
    if (axis == 3) {
      if (DEBUG) Serial.println("AXIS 3");
      mmaxspeed = stepper_Z.maxSpeed();
      mspeed = stepper_Z.speed();
      POSITION_MOTOR_Z = stepper_Z.currentPosition();
      mposition = POSITION_MOTOR_Z;
      pinstep = STEP_PIN_Z;
      pindir = DIR_PIN_Z;
      sign = SIGN_Z;
    }


    jsonDocument.clear();
    jsonDocument["position"] = mposition;
    jsonDocument["speed"] = mspeed;
    jsonDocument["maxspeed"] = mmaxspeed;
    jsonDocument["pinstep"] = pinstep;
    jsonDocument["pindir"] = pindir;
    jsonDocument["sign"] = sign;
  }
}


void setup_motor() {

  /*
     Motor related settings
  */
  if (DEBUG) Serial.println("Setting Up Motors");
  pinMode(ENABLE_PIN, OUTPUT);
  setEnableMotor(true);
  if (DEBUG) Serial.println("Setting Up Motor A");
  if (DEBUG) Serial.println("PIN A"); Serial.println(STEP_PIN_A); Serial.println(DIR_PIN_A);
  stepper_A = AccelStepper(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN_A);
  stepper_A.setMaxSpeed(MAX_VELOCITY_A);
  stepper_A.setAcceleration(MAX_ACCELERATION_A);
  stepper_A.enableOutputs();
  stepper_A.runToNewPosition(-100);
  stepper_A.runToNewPosition(100);
  stepper_A.setCurrentPosition(0);

  if (DEBUG) Serial.println("Setting Up Motor X");
  if (DEBUG) Serial.println("PIN X"); Serial.println(STEP_PIN_X); Serial.println(DIR_PIN_X);
  stepper_X = AccelStepper(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
  stepper_X.setMaxSpeed(MAX_VELOCITY_X);
  stepper_X.setAcceleration(MAX_ACCELERATION_X);
  stepper_X.enableOutputs();
  stepper_X.runToNewPosition(-100);
  stepper_X.runToNewPosition(100);
  stepper_X.setCurrentPosition(0);

  if (DEBUG) Serial.println("Setting Up Motor Y");
  if (DEBUG) Serial.println("PIN Y"); Serial.println(STEP_PIN_Y); Serial.println(DIR_PIN_Y);
  stepper_Y = AccelStepper(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y);
  stepper_Y.enableOutputs();
  stepper_Y.runToNewPosition(-100);
  stepper_Y.runToNewPosition(100);
  stepper_Y.setCurrentPosition(0);

  if (DEBUG) Serial.println("Setting Up Motor Z");
  if (DEBUG) Serial.println("PIN Z"); Serial.println(STEP_PIN_Z); Serial.println(DIR_PIN_Z);
  stepper_Z = AccelStepper(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);
  stepper_Z.setMaxSpeed(MAX_VELOCITY_Z);
  stepper_Z.setAcceleration(MAX_ACCELERATION_Z);
  stepper_Z.enableOutputs();
  stepper_Z.runToNewPosition(-100);
  stepper_Z.runToNewPosition(100);
  stepper_Z.setCurrentPosition(0);

  if (DEBUG) Serial.println("Done Setting Up Motor Z");
  setEnableMotor(false);
}



bool drive_motor_background() {

  // update motor positions
  POSITION_MOTOR_A = stepper_A.currentPosition();
  POSITION_MOTOR_X = stepper_X.currentPosition();
  POSITION_MOTOR_Y = stepper_Y.currentPosition();
  POSITION_MOTOR_Z = stepper_Z.currentPosition();

  // this function is called during every loop cycle
  if (isforever) {
    // run forever
    // is this a bug? Otherwise the speed won't be set properly - seems like it is accelerating eventhough it shouldnt
    stepper_A.setSpeed(mspeed0);
    stepper_X.setSpeed(mspeed1);
    stepper_Y.setSpeed(mspeed2);
    stepper_Z.setSpeed(mspeed3);
    // we have to at least set this. It will be recomputed or something?!
    stepper_A.setMaxSpeed(mspeed1);
    stepper_X.setMaxSpeed(mspeed1);
    stepper_Y.setMaxSpeed(mspeed2);
    stepper_Z.setMaxSpeed(mspeed3);

    stepper_A.runSpeed();
    stepper_X.runSpeed();
    stepper_Y.runSpeed();
    stepper_Z.runSpeed();
  }
  else {
    // run at constant speed
    if (isaccel) {
      stepper_A.run();
      stepper_X.run();
      stepper_Y.run();
      stepper_Z.run();
    }
    else {
      stepper_A.runSpeedToPosition();
      stepper_X.runSpeedToPosition();
      stepper_Y.runSpeedToPosition();
      stepper_Z.runSpeedToPosition();
    }
  }

  // PROBLEM; is running wont work here!
  if ((stepper_A.distanceToGo() == 0) and (stepper_X.distanceToGo() == 0) and (stepper_Y.distanceToGo() == 0) and (stepper_Z.distanceToGo() == 0 ) and not isforever) {
    if (not isen) {
      setEnableMotor(false);
    }
    isBusy = false;
    return true;
  }
  isBusy = true;
  return false; //never reached, but keeps compiler happy?
}


/*
   wrapper for HTTP requests
*/

void motor_act_fct_http() {
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  if (DEBUG) serializeJsonPretty(jsonDocument, Serial);
  motor_act_fct();
  serializeJson(jsonDocument, output);
  server.send(200, "application/json", output);
}

// wrapper for HTTP requests
void motor_get_fct_http() {
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  if (DEBUG) serializeJsonPretty(jsonDocument, Serial);
  motor_get_fct();
  serializeJson(jsonDocument, output);
  server.send(200, "application/json", output);
}

// wrapper for HTTP requests
void motor_set_fct_http() {
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  if (DEBUG) serializeJsonPretty(jsonDocument, Serial);
  motor_set_fct();
  serializeJson(jsonDocument, output);
  server.send(200, "application/json", output);
}
