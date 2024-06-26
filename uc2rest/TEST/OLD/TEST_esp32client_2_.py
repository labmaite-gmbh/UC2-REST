import uc2rest
import numpy as np
import time

port = "unknown"
port = "/dev/cu.SLAB_USBtoUART"
port = "/dev/cu.wchusbserial14310"
#port = "/dev/cu.wchusbserial1440"

esp32 = uc2rest.UC2Client(serialport=port)
# setting debug output of the serial to true - all message will be printed
esp32.serial.DEBUG=True

# test Serial
test_cmd = "{'task': '/motor_get'}"
esp32.serial.writeSerial(test_cmd)
cmd_return = esp32.serial.readSerial()
print(cmd_return)

''' ################
analog
################'''
esp32.analog.set_analog(readanaloginID=1, readanaloginPIN=35, nanaloginavg=1)
esp32.analog.get_analog(readanaloginID=1)
analogValueAVG = esp32.analog.read_sensor(sensorID=1, NAvg=100)
print(analogValueAVG)


''' ################
MOTOR
################'''

# sestup all motors at once 
print(esp32.motor.settingsdict)
esp32.motor.settingsdict["motor"]["steppers"][1]["dir"]=16
esp32.motor.settingsdict["motor"]["steppers"][1]["step"]=26
esp32.motor.settingsdict["motor"]["steppers"][2]["dir"]=27
esp32.motor.settingsdict["motor"]["steppers"][2]["step"]=25
esp32.motor.settingsdict["motor"]["steppers"][3]["dir"]=14
esp32.motor.settingsdict["motor"]["steppers"][3]["step"]=17
esp32.motor.settingsdict["motor"]["steppers"][0]["dir"]=18
esp32.motor.settingsdict["motor"]["steppers"][0]["step"]=19
esp32.motor.settingsdict["motor"]["steppers"][0]["enable"]=12
esp32.motor.settingsdict["motor"]["steppers"][1]["enable"]=12
esp32.motor.settingsdict["motor"]["steppers"][2]["enable"]=12
esp32.motor.settingsdict["motor"]["steppers"][3]["enable"]=12
esp32.motor.set_motors(esp32.motor.settingsdict)

# print settings 
print(esp32.motor.get_motors())

# OR setup motors individually (according to WEMOS R32 D1)
esp32.motor.set_motor(stepperid = 1, position = 0, stepPin = 26, dirPin=16, enablePin=12, maxPos=None, minPos=None, acceleration=None, isEnable=1)
esp32.motor.set_motor(stepperid = 2, position = 0, stepPin = 25, dirPin=27, enablePin=12, maxPos=None, minPos=None, acceleration=None, isEnable=1)
esp32.motor.set_motor(stepperid = 3, position = 0, stepPin = 17, dirPin=14, enablePin=12, maxPos=None, minPos=None, acceleration=None, isEnable=1)
esp32.motor.set_motor(stepperid = 0, position = 0, stepPin = 19, dirPin=18, enablePin=12, maxPos=None, minPos=None, acceleration=None, isEnable=1)

# get individual motors
print(esp32.motor.get_motor(axis = 1))

esp32.motor.set_motor_currentPosition(axis=0, currentPosition=10000)
esp32.motor.set_motor_acceleration(axis=0, acceleration=10000)
esp32.motor.set_motor_enable(is_enable=1)
esp32.motor.set_direction(axis=1, sign=1, timeout=1)
esp32.motor.set_position(axis=1, position=0, timeout=1)

# wait to settle
time.sleep(2)

# test Motor
position1 = esp32.motor.get_position(timeout=1)
print(position1)
esp32.motor.move_x(steps=10000, speed=10000, is_blocking=True)
esp32.motor.move_y(steps=1000, speed=1000, is_blocking=True)
esp32.motor.move_z(steps=1000, speed=1000, is_blocking=True)
esp32.motor.move_t(steps=1000, speed=1000)
esp32.motor.move_xyzt(steps=(0,10000,10000,0), speed=10000, is_blocking=True)
esp32.motor.move_xyzt(steps=(0,0,0,0), speed=10000, is_absolute=True, is_blocking=True)
esp32.motor.move_forever(speed=(0,100,0,0), is_stop=False)
time.sleep(1)
esp32.motor.move_forever(speed=(0,0,0,0), is_stop=True)

position2 = esp32.motor.get_position(timeout=1)
print(position2)


''' ################
LASER 
################'''
# set laser pins 
esp32.laser.set_laserpin(laserid=1, laserpin=15)
esp32.laser.set_laserpin(laserid=2, laserpin=16)
esp32.laser.set_laserpin(laserid=3, laserpin=17)

# set laser values
esp32.laser.set_laser(channel=1, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)
esp32.laser.set_laser(channel=2, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)
esp32.laser.set_laser(channel=3, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)


''' ################
LED 
################'''
# setup led configuration
esp32.led.set_ledpin(ledArrPin=4, ledArrNum=16)
print(esp32.led.get_ledpin())

# test LED
led_pattern = np.zeros((1, 5, 5, 3), dtype=np.uint8)
esp32.led.send_LEDMatrix_array(led_pattern=led_pattern, timeout=1)
esp32.led.send_LEDMatrix_full(intensity=(255, 0, 0), timeout=1)
esp32.led.send_LEDMatrix_single(indexled=0, intensity=(0, 255, 0), timeout=1)


''' ################
Wifi
################'''
# wifi
esp32.wifi.scanWifi()


''' ################
Config Manager 
################'''

configfile = esp32.config.loadConfigDevice(timeout=1)
print(configfile)
configfile["motorconfig"][0]["dir"]=17 # change parameter
esp32.config.setConfigDevice(configfile)


''' ################
State
################'''
# test state
_state = esp32.state.get_state()
print(_state)
esp32.state.set_state(debug=False)
_mode = esp32.state.isControllerMode()
print(_mode)
esp32.state.espRestart()
time.sleep(5)
esp32.state.setControllerMode(isController=True)
_busy = esp32.state.isBusy()
print(_busy)

#