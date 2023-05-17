# -*- coding: utf-8 -*-
"""
"""

import logging
import asyncio
import platform

import bleak.exc
import numpy
from bleak import BleakClient
from bleak import _logger as logger
import struct
import csv
import threading
import os
import datetime




from bleak import BleakScanner


Tensorflow_UUID = ""
ppg_UUID = ""

bleak_device = ""
start_collection_date = ""
file_name = ""

file_obj = None

use_previous_packet_format = False




''' This is a class for holding information about a single bluetooth attribute.'''
class MSenseCharacteristic:


    def __init__(self, name, function, uuid):
        self.name = name
        self.function = function
        self.uuid = uuid


    def __str__(self):
        return self.name

'''This is a class that holds a Device, consisting of bluetooth characteristics
This is different from the widget class, but the widget class contains it.'''
class MSenseDevice:

    def __init__(self, name:str, characteristics:list[MSenseCharacteristic]):
        self.name = name
        self.address = None
        self.characteristics = characteristics

        #TODO: Implement
        self.battery_level = None
        self.connection_strength = None


'''Utility class for what sensor options to generate'''
class MSense_collect_options:

    def __init__(self,ppg=False, magnomater=False, accelorometer=False):
        self.collect_ppg = ppg
        self.magnomater = magnomater
        self.accelorometer = accelorometer


    def __str__(self):
        return str(self.collect_ppg) + str(self.magnomater) + str(self.accelorometer)


class MSense_data:
    print_data_to_files = False
    tf_micro_packet = []
    tf_micro_packet_counter = []

    ppg_led1ir_arr = []
    ppg_led2ir_arr = []

    ppg_g1_arr = []
    ppg_g2_arr = []
    ppg_packet_counter = []
    ppg_packet_loss_counter = []
    # upon first execution of the handler, this variable will be
    # assigned a file object
    ppg_file = None


    accelorometer_data = []
    accelorometer_packet_counter = []
    accelorometer_packet_loss = []
    accelorometer_x = []
    accelorometer_y = []
    accelorometer_z = []
    accelorometer_file = None

    angular_velocity_x = []
    angular_velocity_y = []
    angular_velocity_z = []

    magnometer = []
    magnometer_packet = []
    magnometer_file = None



def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    print("{0}: {1}".format(sender, data))
    print(len(data))
    #siz = struct.calcsize(data)
    you = data[0:64]
    nk_data = struct.unpack("<ffffffffffffffff", you)
    packet_counter = data[64:66]
    packet_counter = struct.unpack("<h", packet_counter)
    #print(nk_data)
    counter = 0
    flt_arr2 = []
    for x in data:

        flt2 = data[counter]
        counter += 1

    print(nk_data)

    MSense_data.tf_micro_packet = nk_data
    MSense_data.tf_micro_packet_counter.append(packet_counter)
    arr = numpy.array(data)
    #arr2 = arr.astype(dtype=numpy)
    return nk_data


def notification_handler_general(sender, data, num=0, type:str="f", class_vari=None):
    #num should be in bytes
    assert num > 0
    th_data = data[0: num]
    len_string = "<"
    len_string += len_string.join([type for x in range(0, int(num/4))])
    packed_th_data = struct.unpack(len_string, th_data)
    #if there is a class_variable (has global typing), save the number to that
    if class_vari:
        class_vari = packed_th_data

    return packed_th_data


'''Begin Current functionality'''

def motionsense_handler(sender, data):
    global file_name
    #m_service = bleak_device.services.characteristics[30]
    #await bleak_device.read_gatt_char(m_service)
    accelorometer_data = []
    Accelorometer_X = data[0:2]
    print(Accelorometer_X)
    Accelorometer_Y = data[2:4]
    Accelerometer_Z = data[4:6]
    Angular_velocity_X = data[6:8]
    Angular_velocity_Y = data[8:10]
    Angular_velocity_Z = data[10:12]
    Accelorometer_X = struct.unpack(">h", Accelorometer_X)
    Accelorometer_Y = struct.unpack(">h", Accelorometer_Y)
    Accelerometer_Z = struct.unpack(">h", Accelerometer_Z)

    Angular_velocity_X = struct.unpack(">h", Angular_velocity_X)
    Angular_velocity_Y = struct.unpack(">h", Angular_velocity_Y)
    Angular_velocity_Z = struct.unpack(">h", Angular_velocity_Z)

    MSense_data.accelorometer_x.append(Accelorometer_X[0])
    MSense_data.accelorometer_y.append(Accelorometer_Y[0])
    MSense_data.accelorometer_z.append(Accelerometer_Z[0])
    MSense_data.angular_velocity_x.append(Angular_velocity_X[0])
    MSense_data.angular_velocity_y.append(Angular_velocity_Y[0])
    MSense_data.angular_velocity_z.append(Angular_velocity_Z[0])

    packet_counter = data[12:14]
    print("ppg packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)
    MSense_data.accelorometer_packet_counter.append(packet_counter[0])

    packets_recived = MSense_data.accelorometer_packet_counter[len(MSense_data.accelorometer_packet_counter) - 1] - \
                      MSense_data.accelorometer_packet_counter[
                          len(MSense_data.accelorometer_packet_counter) - 2]
    MSense_data.accelorometer_packet_loss.append(packets_recived)



def ppg_handler(sender, data:bytes):
    global file_name
    print(sender)
    if use_previous_packet_format:
        Led_ir1 = data[0:4]
        Led_ir2 = data[4:8]
        Led_g15 = data[12:16]
        Led_g2 = data[16:20]

        Led_ir1 = struct.unpack("<f", Led_ir1)
        Led_ir2 = struct.unpack("<f", Led_ir2)
    else:
        Led_ir11 = data[0]
        Led_ir11 <<= 11
        Led_ir12 = data[1]
        Led_ir12 <<= 3
        Led_ir13 = data[2]
        Led_ir13 >>= 5
        Led_ir1 = Led_ir11 + Led_ir12 + Led_ir13

        Led_ir21 = data[2]
        Led_ir21 &= 31
        Led_ir21 <<= 14
        Led_ir22 = data[3]
        Led_ir22 <<= 6
        Led_ir23 = data[4]
        Led_ir23 >>= 2
        Led_ir2 = Led_ir21 + Led_ir22 + Led_ir23

        Led_g11 = data[4]
        Led_g11 &= 3
        Led_g11 <<= 17
        Led_g12 = data[5]
        Led_g12 <<= 9
        Led_g13 = data[6]
        Led_g13 <<= 1
        Led_g14 = data[7]
        Led_g14 >>= 7
        Led_g15 = Led_g11 + Led_g12+ Led_g13+Led_g14

        Led_g21 = data[7]
        Led_g21 &= 127
        Led_g21 <<= 12
        Led_g22 = data[8]
        Led_g22 <<= 4
        Led_g23 = data[9]
        Led_g23 >>= 4
        Led_g2 = Led_g21 + Led_g22 + Led_g23

    arr = bytearray(data)
    hex_string = ''
    for x in arr:
        hex_string += hex(x) + " "
    #Led_ir1 = struct.unpack("<l", Led_ir1)
    hex_string += " interpreted value: Led IR1:" + str(Led_ir1) + " LED IR2 " + str(Led_ir2) + " LED G1 " + str(Led_g15) + " LED G2 " + str(Led_g2)
    packet_counter = data[10:12]
    print("ppg packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)

    MSense_data.ppg_led1ir_arr.append(Led_ir1)
    MSense_data.ppg_led2ir_arr.append(Led_ir2)
    MSense_data.ppg_g1_arr.append(Led_g15)
    MSense_data.ppg_g2_arr.append(Led_g2)

    # packet counter logic calculation
    MSense_data.ppg_packet_counter.append(packet_counter[0])
    packets_recived = MSense_data.ppg_packet_counter[len(MSense_data.ppg_packet_counter) - 1] - MSense_data.ppg_packet_counter[
        len(MSense_data.ppg_packet_counter) - 2]

    MSense_data.ppg_packet_loss_counter.append(10-packets_recived)


def prev_format_accel_HRV(sender, data: bytes):
    global file_name
    print(sender)

    ppg_file = open(file_name + "\\" + start_collection_date + "-ppg.txt", "a")

    # print(Led_ir1)
    arr = bytearray(data)
    hex_string = ''
    for x in arr:
        hex_string += hex(x) + " "

    ppg_file.write(hex_string + "\n")


def notification_handler_magnometer(sender, data):
    magnometer_ints = data[0:6]
    print("unpacking magnometer data")
    nk_data = struct.unpack("<hhh", magnometer_ints)
    #print(nk_data)
    magnometer_packet_information = data[6:8]

    magnometer_file = open(file_name + "\\" + start_collection_date + "-magnometer.txt", "a")

    magnometer_packet_information = struct.unpack(">h", magnometer_packet_information)

def orientation_handler(sender, data):
    pass


def build_uuid_dict(client):
    characteristics = client.services.characteristics.values()
    characteristics = list(characteristics)
    uuid_arr = {}
    for characteristic in characteristics:
        uuid_arr[characteristic.uuid] = characteristic.handle

    return uuid_arr



async def connect_address(Devices:list[MSenseDevice]=None):
    print("scanning connections")
    addr = None
    motion_sense_devices = []
    devices = await BleakScanner.discover()
    device_set = dict()
    if Devices is not None:
        for device in Devices:
            device_set[device.name] = device
    else:
        device_set["RightMotionSense2"] = MSenseDevice()


    for devi in devices:
        if devi.name in device_set:
            device_object = device_set[devi.name]
            device_object.address = devi.address
            addr = devi.address
            motion_sense_devices.append(device_object)
            print("found! with address", str(devi.address))
        print(devi)
        #if the device == MotionSense: get address
        #to do at home
    return motion_sense_devices

# a function for writing to induvidual files
# name will become a index later on
def write_to_csv_file(name:str, data):
    if name == "ppg":
        flie_obj:csv.writer = MSense_data.ppg_file
        flie_obj.writerow(data)
    # TODO: check for flag


def create_csv_file(name:str, path):
    assert path is not None
    file_name = path + "/" + name + ".csv"
    if name == "ppg":
        field_names = {"green_1": None, "green_2": None, "ir_1":None, "ir_2":None}
        th_csv_file = open(file_name, "w")
        MSense_data.ppg_file = csv.DictWriter(th_csv_file, fieldnames=field_names)
        MSense_data.ppg_file.writeheader()

def make_csv_dict(str, data):
    pass



async def run(address, debug=True, path=None, data_amount = 30.0, options:list[MSenseCharacteristic]=None):
    try:
        print("starting run function")
        # this has to be global because it is async
        global bleak_device
        global start_collection_date
        # just a little check to remember why this is global
        global file_name
        print(file_name)
        global file_obj
        start_collection_date += str(datetime.datetime.now())
        # windows doesn't like colons, so we have to remove them
        start_collection_date = start_collection_date.replace(":", "")
        assert options != None
        assert path != None
        file_name = path
        assert len(options) != 0
        print(type(options[0]))
        assert type(options[0]) == MSenseCharacteristic
        #maybe change the parameter to data amount in seconds
        if debug:
            import sys

            l = logging.getLogger("asyncio")
            l.setLevel(logging.DEBUG)
            h = logging.StreamHandler(sys.stdout)
            h.setLevel(logging.DEBUG)
            l.addHandler(h)
            logger.addHandler(h)

        print("trying to connect with client")
        async with BleakClient(address) as client:
            x = await client.is_connected()
            print("connected to MotionSense!")
            logger.info("Connected: {0}".format(x))
            clu = await client.get_services()
            uuid_arr = build_uuid_dict(client)
            #l_service = client.services.characteristics[17]
            #l_service is
            bleak_device = client

            #l2_service is latent ppg information
            #descriptor = l_service.descriptors[0]
            #o3 = await client.read_gatt_descriptor(19)
            #print(o3)

            write_pi = bytearray([0, 1])
            #await client.write_gatt_descriptor(19, write_pi)
            #o3 = await client.read_gatt_descriptor(19)
            #print(o3)
            #this should be magnometer service
            #m_service = bleak_device.services.characteristics[30]
            #d = await bleak_device.read_gatt_char(m_service)
            #motion_sense_characteristic()

            #await client.write_gatt_char()
            #await client.start
            #tf_kr_array = await client.start_notify(l2_service, data_adr2)

            current_services = []
            for characteristic in options:
                try:

                    service = client.services.characteristics[uuid_arr[characteristic.uuid]]
                except KeyError():
                    error_string = "Error: bluetooth characteristic UUID is invalid for device " + characteristic.name
                    print(error_string)
                    return error_string

                #ppg_service = client.services.characteristics[uuid_arr["da39c926-1d81-48e2-9c68-d0ae4bbd351f"]]

                current_services.append(service)

                #create_csv_file("ppg", path)
                print("starting notify for " + str(characteristic.name))

                status = await client.start_notify(service, characteristic.function)


            #we need to do the rest of the sensors as well
            #collect data
            await asyncio.sleep(data_amount)

    except Exception as e:
        print(e)
        await disconnect(client, current_services)

    try:
        write_all_files(file_name)
    except Exception as e:
        print(e)

    print("all files written")
    await disconnect(client, current_services)

        #j = MSense_data
    ppg_pack = MSense_data.ppg_packet_counter
    ppg_pack_loss = MSense_data.ppg_packet_loss_counter

    magno_pack = MSense_data.magnometer_packet
    return "Finished Data Collection for "


#async def enable_sec():

def write_all_files(file_name):
    # write accelorometer data
    if not os.path.exists(file_name):
        os.mkdir(file_name)


    MSense_data.accelorometer_file = open(file_name + "//Acelorometer.csv", "w", newline="")
    csv_writer = csv.writer(MSense_data.accelorometer_file)
    csv_rows = list()
    for data_element in range(len(MSense_data.accelorometer_x)):
        csv_rows.append([MSense_data.accelorometer_x[data_element], MSense_data.accelorometer_y[data_element],
                         MSense_data.accelorometer_z[data_element], MSense_data.angular_velocity_x[data_element],
                         MSense_data.angular_velocity_y[data_element], MSense_data.angular_velocity_z[data_element],
                         MSense_data.accelorometer_packet_counter[data_element], MSense_data.accelorometer_packet_loss[data_element]])
    csv_writer.writerow(["Acel_X", "Acel_Y", "Acel_Z", "AngVel_X", "AngVel_Y", "AngVel_Z", "PacketCounter", "PacketLoss"])
    csv_writer.writerows(csv_rows)
    MSense_data.accelorometer_file.close()

    MSense_data.ppg_file = open(file_name + "//PPG.csv", "w", newline="")
    csv_writer = csv.writer(MSense_data.ppg_file)
    csv_rows = list()
    for data_element in range(len(MSense_data.ppg_led1ir_arr)):
        csv_rows.append([MSense_data.ppg_led1ir_arr[data_element], MSense_data.ppg_led2ir_arr[data_element],
                         MSense_data.ppg_g1_arr[data_element], MSense_data.ppg_g2_arr[data_element],
                         MSense_data.ppg_packet_counter[data_element], MSense_data.ppg_packet_loss_counter[data_element]])
    csv_writer.writerow(["PPG_IR1", "PPG_IR2", "PPG_G1", "PPG_G2", "PacketCounter", "PacketLoss"])
    csv_writer.writerows(csv_rows)
    MSense_data.ppg_file.close()



async def disconnect(client, services:list):
    for service in services:
        try:
            await client.stop_notify(service)
        except Exception as e:
            print("failed to stop notify")
            print(e)
    try:
        await client.disconnect()
    except Exception as e:
        print("failed to disconnect")
        print(e)
    print('properly disconnected')

def start_background_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


def non_async_connect(devices_to_search=None):
    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    print("System: ", platform.system())
    # B9EA5233-37EF-4DD6-87A8-2A875E821C46
    loop = asyncio.get_event_loop()
    address = loop.run_until_complete(connect_address(devices_to_search))
    return address


# this is the function that is executed inside the GUI to make sure everything runs properly
def non_async_collect(address, path, max_length, collect_options, end_flag):
    print("starting non async collecion function with parameters:")
    print("address: " + address)
    print("path: " + path)
    print("collection options: " + str(collect_options))
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(run(address, True, path=path, data_amount=max_length, options=collect_options))
    except Exception as e:
        print("bleak client backend bluetooth error")
        print(e)




async def collect_with_adress(address):
    loop = asyncio.get_event_loop()
    address = loop.run_until_complete(connect_address())
    loop.run_until_complete(run(address, True, notification_handler, data_amount=10))

    return True




def turn_on(data_function=notification_handler, record_length = 300):
    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    print("System: ", platform.system())
    address = (
        ""  # <--- Change to your device's address here if you are using Windows or Linux
        if platform.system() != "Darwin"
        else "B9EA5233-37EF-4DD6-87A8-2A875E821C46"  # <--- Change to your device's address here if you are using macOS
    )
    # B9EA5233-37EF-4DD6-87A8-2A875E821C46
    loop = asyncio.get_event_loop()
    address = loop.run_until_complete(connect_address())
    if address == None or address == "":
        raise Exception("could not find MotionSense! Make sure device is turned on and flashing")
    print("address: to connect: " + address)
    print("attempting to connect to MotionSense...\n")  # loop.set_debug(True)
    loop.run_until_complete(run(address, True, notification_handler, data_amount=record_length))


if __name__ =='__main__':
    turn_on(record_length=30)
