from data_collection import bluetooth_reciver

'''Setting up a device for data collection is composed of 2 steps:
    1. Create a MSenseCharacteristic object for every characteristic you would like to collect from,
    by inputting a Name, function, and UUID

    2. Create an MSenseDevice object, and add all desired characteristics to it through a list.

A Default Example using the Ohio State SENSE lab devices has been done for you below'''

ppg_sensor = bluetooth_reciver.MSenseCharacteristic("PPG", bluetooth_reciver.ppg_sensor_handle,
                                                    "da39c925-1d81-48e2-9c68-d0ae4bbd351f")
magnometer = bluetooth_reciver.MSenseCharacteristic("Magnometer",
                                                    bluetooth_reciver.notification_handler_magnometer,
                                                    "da39c922-1d81-48e2-9c68-d0ae4bbd351f")

accelorometer = bluetooth_reciver.MSenseCharacteristic("Accelorometer",
                                                       bluetooth_reciver.motion_sense_characteristic,
                                                       "da39c921-1d81-48e2-9c68-d0ae4bbd351f")

RightSensor = bluetooth_reciver.MSenseDevice("RightMotionSense2", [ppg_sensor, magnometer, accelorometer])

LeftSensor = bluetooth_reciver.MSenseDevice("LeftMotionSense2", [ppg_sensor, magnometer, accelorometer])

all_sensors = [RightSensor, LeftSensor]