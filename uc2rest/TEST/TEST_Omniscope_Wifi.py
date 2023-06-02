import uc2rest
import numpy as np
import time

host = "192.168.137.60"
port = 80

ESP32 = uc2rest.UC2Client(host=host, port=port, DEBUG=True)

# check if we are connected 
# see if it's the right device
mState = ESP32.state.get_state()
print(mState)


''' ################
LED 
################'''
# test LED
mResult = ESP32.led.send_LEDMatrix_full(intensity=(25, 25, 25))
mResult = ESP32.led.send_LEDMatrix_full(intensity=(0,0,0))


ESP32.motor.move_x(steps=10000, speed=10000, is_blocking=True)
ESP32.motor.move_y(steps=1000, speed=1000, is_blocking=True, is_enabled=False)
ESP32.motor.move_z(steps=1000, speed=1000, is_blocking=True)
ESP32.motor.move_y(steps=1000, speed=1000, is_blocking=True, is_enabled=False)

lkjalksdjf

# single LED
for iLED in range(5):
    # timeout = 0 means no timeout => mResult will be rubish!
    mResult = ESP32.led.send_LEDMatrix_single(indexled=iLED, intensity=(255, 255, 255), timeout=0.)
    mResult = ESP32.led.send_LEDMatrix_single(indexled=iLED, intensity=(0, 0, 0), timeout=0.)
    assert mResult["success"] == 1, "Failed sending LED command"

# display random pattern
for i in range(5):
    led_pattern = np.random.randint(0,55, (25,3))
    mResult = ESP32.led.send_LEDMatrix_array(led_pattern=led_pattern,timeout=0)
    assert mResult["success"] == 1, "Failed sending LED command"


#%% left
if(0):
    led_pattern = np.zeros((25,3))
    list_left = (0,1,2,3,4,5,9,10,11,12,13,14,15,16,17)
    list_right = (0,5,6,7,8,9,18,19,20,21,22,23,24)
    led_pattern[list_left,0] = 255
    led_pattern[list_right,1] = 255
    ESP32.led.send_LEDMatrix_array(led_pattern=led_pattern, timeout=1)
    time.sleep(1)
    ESP32.led.send_LEDMatrix_array(led_pattern=led_pattern*0, timeout=1)

#%%
#%%
''' ################
MOTOR
################'''
ESP32.motor.move_x(steps=10000, speed=10000, is_blocking=True)

# mResult = ESP32.motor.move_x(steps=0, is_enabled=False)
mResult = ESP32.motor.set_motor_enable(enable=1)
mResult = ESP32.motor.set_motor_enable(enable=0)
assert mResult["success"] == 1, "Failed sending motor command"

# test Motor
mResult = ESP32.motor.set_position(axis=0, position=1000)
assert mResult["success"] == 1, "Failed sending motor command"

position1 = ESP32.motor.get_position(timeout=1)
assert position1[0]==1000, "Failed getting motor position"
print(position1)

ESP32.motor.move_x(steps=10000, speed=10000, is_blocking=True)
ESP32.motor.move_y(steps=1000, speed=1000, is_blocking=True, is_enabled=False)
ESP32.motor.move_z(steps=1000, speed=1000, is_blocking=True)
ESP32.motor.move_y(steps=1000, speed=1000, is_blocking=True, is_enabled=False)
ESP32.motor.move_t(steps=1000, speed=1000)
ESP32.motor.move_xyzt(steps=(0,1000,100,0), speed=10000, is_blocking=True)
ESP32.motor.move_xyzt(steps=(0,0,0,0), speed=10000, is_absolute=True, is_blocking=True)
ESP32.motor.move_forever(speed=(0,100,0,0), is_stop=False)
ESP32.motor.move_forever(speed=(0,100,0,0), is_stop=False)
time.sleep(1)
ESP32.motor.move_forever(speed=(0,0,0,0), is_stop=True)

position2 = ESP32.motor.get_position(timeout=1)
print(position2)

dDist = 1000
speed = 20000
nDist = 4

# test Motor in scanning mode
ESP32.motor.move_xyzt(steps=(0,0,0,0), speed=speed, is_absolute = True, is_blocking=True)

for ix in range(nDist):
    for iy in range(nDist):
        if ix%2==0:
            iy=nDist-iy
        ESP32.motor.move_xyzt(steps=(0,ix*dDist,iy*dDist,0), speed=speed, is_absolute = True, is_blocking=True)
ESP32.motor.move_xyzt(steps=(0,nDist*dDist,nDist*dDist,0), speed=speed, is_absolute = True, is_blocking=True)
ESP32.motor.move_xyzt(steps=(0,0,0,0), speed=speed, is_absolute = True, is_blocking=True)

#%%

''' ################
analog
################'''
ESP32.analog.set_analog(readanaloginID=1, readanaloginPIN=35, nanaloginavg=1)
ESP32.analog.get_analog(readanaloginID=1)
#analogValueAVG = ESP32.analog.read_sensor(sensorID=1, NAvg=100)
#print(analogValueAVG)

#%%
''' ################
HOME
################'''
ESP32.home.home_x(speed =15000, direction = -1, endposrelease = 3000, timeout=20, isBlocking=True)
ESP32.home.home_y(speed =15000, direction = 1, endposrelease = 3000, timeout=20, isBlocking=True)

''' ################
LASER 
################'''
# set laser pins 
if ESP32.APIVersion == 2:
    ESP32.laser.set_laserpin(laserid=1, laserpin=15)
    ESP32.laser.set_laserpin(laserid=2, laserpin=16)
    ESP32.laser.set_laserpin(laserid=3, laserpin=17)

# get laser pins
ESP32.laser.get_laserpins()
ESP32.laser.get_laserpin(laserid=1)

# set laser values
ESP32.laser.set_laser(channel=1, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)
ESP32.laser.set_laser(channel=2, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)
ESP32.laser.set_laser(channel=3, value=1000, despeckleAmplitude=0, despecklePeriod=10, timeout=20, is_blocking = True)




''' ################
Wifi
################'''
# wifi
if ESP32.APIVersion == 2:
    ESP32.wifi.scanWifi()


''' ################
Config Manager 
################'''
if ESP32.APIVersion == 2:
    configfile = ESP32.config.loadConfigDevice(timeout=1)
    print(configfile)
    configfile["motorconfig"][0]["dir"]=17 # change parameter
    ESP32.config.setConfigDevice(configfile)

''' ################
State
################'''
# test state
_state = ESP32.state.get_state()
print(_state)
ESP32.state.set_state(debug=False)
_mode = ESP32.state.isControllerMode()
print(_mode)
ESP32.state.espRestart()
time.sleep(5)
ESP32.state.setControllerMode(isController=True)
_busy = ESP32.state.isBusy()
print(_busy)
