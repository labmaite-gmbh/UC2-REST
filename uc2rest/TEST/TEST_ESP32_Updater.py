import uc2rest as uc2

# define the serial port
serialport = "/dev/cu.SLAB_USBtoUART"
#serialport = "COM3"

# optional: create an ESP32 objejct
# ESP32 = uc2.UC2Client(serialport=serialport)

# create the updater object
updater = uc2.updater(port=serialport)

updater.downloadFirmware()
print(updater.firmwarePath)
updater.flashFirmware()

# print firmwarepath


# remove firmware.bin after flashing
updater.removeFirmware()


C:/Users/julia/AppData/Local/Temp/uc2rest/

python -m esptool --chip esp32 --port COM3 --baud 921600  --before default_reset --after hard_reset write_flash -z --flash_mode dio  --flash_freq 80m --flash_size 4MB  0x1000 C:/Users/julia/AppData/Local/Temp/uc2rest/UC2Rest/main.ino.bootloader.bin 0x8000 C:/Users/julia/AppData/Local/Temp/uc2rest/UC2Rest/main.ino.partitions.bin 0xe000 C:/Users/julia/AppData/Local/Temp/uc2rest/UC2Rest/boot_app0.bin  0x10000 C:/Users/julia/AppData/Local/Temp/uc2rest/UC2Rest/main.ino.bin