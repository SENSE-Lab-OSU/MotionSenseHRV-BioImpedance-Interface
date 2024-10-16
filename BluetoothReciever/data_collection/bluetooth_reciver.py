# -*- coding: utf-8 -*-
"""
"""

import sys
sys.coinit_flags = 0  # 0 means MTA

import logging
import asyncio
import platform
import time
import atexit
import hashlib

import numpy
from bleak import BleakClient
from bleak import _logger as logger
from bleak import BleakScanner
from copy import deepcopy
import bleak.uuids
import scipy
import struct
import csv
import os

import datetime

debug_print_updates = False
show_matplotlib_graphs = False
use_lsl = True
if use_lsl:
    from data_collection import lsl_transmission
    ppg_stream_outlet = None
    accelorometer_outlet = None
    led_outlet = None
    enmo_outlet = None


import multiprocessing
def is_multiprocessing():
    """Check if the current script is running in a multiprocessing context."""
    return multiprocessing.get_start_method(allow_none=True) is not None

if True:
    try:
        # this is a quick fix for windows devices in which the backend is win32, because win32 does not allow
        # a gui tick with bleak for some reason
        from bleak.backends.winrt.util import allow_sta, uninitialize_sta
        print("performing sta logistics")
        print(sys.modules)
        if not is_multiprocessing():
            allow_sta()
        else:
            uninitialize_sta()
    except AttributeError as e:
        print("skipped sta")
        print(e)
        # other OSes and versions work, so we can just ignore.
        pass
    except ModuleNotFoundError as e:
        print("skipped sta")
        print(e)
        # other OSes and versions work, so we can just ignore.
        pass

Tensorflow_UUID = ""
ppg_UUID = ""

bleak_device = ""
start_collection_date = ""
file_name = ""

file_obj = None

use_previous_packet_format = False
sucessful_file_write = False
real_time_graph_updates = True

real_time_graph_counter = 0

status_flag = 0


previous_time = 0
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
    ppg_date_time = []
    # upon first execution of the handler, this variable will be
    # assigned a file object
    ppg_file = None
    ppg_lsl = None

    enmo_data = []
    enmo_packet_counter = []
    enmo_file = None
    enmo_lsl = None

    accelorometer_data = []
    accelorometer_packet_counter = []
    accelorometer_packet_loss = []
    accelorometer_x = []
    accelorometer_y = []
    accelorometer_z = []
    accelorometer_timestamp = []
    accelorometer_file = None
    accelorometer_lsl = None


    angular_velocity_x = []
    angular_velocity_y = []
    angular_velocity_z = []

    magnometer = []
    magnometer_packet = []
    magnometer_file = None

    BioImpedanceMag = []
    BioImpedancePhase = []
    BioImpedancePacketCounter = []
    BioImpedanceTimeStamp = []
    BioImpedanceFile = None



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
    if debug_print_updates:
        print("accelorometer packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)
    MSense_data.accelorometer_packet_counter.append(packet_counter[0])

    packets_recived = MSense_data.accelorometer_packet_counter[len(MSense_data.accelorometer_packet_counter) - 1] - \
                      MSense_data.accelorometer_packet_counter[
                          len(MSense_data.accelorometer_packet_counter) - 2]
    MSense_data.accelorometer_packet_loss.append(packets_recived)
    MSense_data.accelorometer_timestamp.append(str(datetime.datetime.now().time()))
    horizontal_array = [Accelorometer_X[0], Accelorometer_Y[0], Accelerometer_Z[0], Angular_velocity_X[0], Angular_velocity_Y[0], Angular_velocity_Y[0], packet_counter[0], packets_recived]
    if use_lsl:
        lsl_transmission.send_data(accelorometer_outlet, horizontal_array)


def enmo_handler(sender, data):
    global file_name
    #m_service = bleak_device.services.characteristics[30]
    #await bleak_device.read_gatt_char(m_service)
    accelorometer_data = []
    ENMO = data[0:4]
    packet_counter = data[4:6]
    ENMO = struct.unpack("<f", ENMO)

    if debug_print_updates:
        print("accelorometer packet counter: " + str(packet_counter))
    packet_counter = struct.unpack("<H", packet_counter)
    MSense_data.enmo_data.append(ENMO)
    MSense_data.enmo_packet_counter.append(packet_counter)
    horizontal_array = [ENMO[0], packet_counter[0]]
    print(str(horizontal_array))
    if use_lsl:
        global enmo_outlet
        lsl_transmission.send_data(enmo_outlet, horizontal_array)




def led_handler(sender, data):
    led_status = data[0]
    global previous_time
    packet_counter = data[1:3]
    if debug_print_updates:
        print("led packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)
    send_array = [led_status, packet_counter[0]]

    print(time.time()-previous_time)
    previous_time = time.time()
    if use_lsl:
        lsl_transmission.send_data(led_outlet, send_array)
        #lsl_transmission.send_data(led_outlet, send_array, custom_time_increment=1)
    #packets_recived = MSense_data.accelorometer_packet_counter[len(MSense_data.accelorometer_packet_counter) - 1] - \
    #                  MSense_data.accelorometer_packet_counter[
    #                      len(MSense_data.accelorometer_packet_counter) - 2]
    #MSense_data.accelorometer_packet_loss.append(packets_recived)
    #MSense_data.accelorometer_timestamp.append(str(datetime.datetime.now().time()))



def ppg_handler(sender, data:bytes):
    global file_name
    #print(sender)
    if use_previous_packet_format:
        Led_ir1 = data[0:4]
        Led_ir2 = data[4:8]
        Led_g15 = data[12:16]
        Led_g2 = data[16:20]

        Led_ir1 = struct.unpack("<f", Led_ir1)
        Led_ir2 = struct.unpack("<f", Led_ir2)
    else:
        # to decode ppg, we need to reverse the order of the data and perform bit shifting, as we are working with
        # 19 bit little endian numbers that are packed together 19 bit, then another 19 bit, etc

        # get the first 8 bits and shift it left by 11 bits to get the MSB (most significant bit) (8 + 11 = 19)
        # in position 19
        Led_ir11 = data[0]
        Led_ir11 <<= 11
        # get the 2nd 8 bits and move it behind the first 8 bits. so we move the MSB bit to the 11th position (8+3=11)
        Led_ir12 = data[1]
        Led_ir12 <<= 3
        # now that we have gotten the first 16 bits, there are only 3 bits left for our 19 bit number. we shift right
        # so that the MSB occupies the third position, and everything after that is zeros
        Led_ir13 = data[2]
        Led_ir13 >>= 5
        # add up all the bits to get our 19 bit floating number, as a python float
        Led_ir1 = Led_ir11 + Led_ir12 + Led_ir13
        # continue this pattern.
        Led_ir21 = data[2]
        Led_ir21 &= 31 # 31 = 00011111, so it acts as a bit mask to only keep the first 5 values starting from the right hand side and going left.
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
    if debug_print_updates:
        print("ppg packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)

    MSense_data.ppg_led1ir_arr.append(Led_ir1)
    MSense_data.ppg_led2ir_arr.append(Led_ir2)
    MSense_data.ppg_g1_arr.append(Led_g15)
    MSense_data.ppg_g2_arr.append(Led_g2)
    if real_time_graph_updates:
        global real_time_graph_counter
        real_time_graph_counter += 1
        if real_time_graph_counter > 20:
            real_time_graph_counter = 0
            show_realtime_graph(file_name + "filtered ppg graph",
                    [MSense_data.ppg_led1ir_arr, MSense_data.ppg_led2ir_arr, MSense_data.ppg_g1_arr,
                    MSense_data.ppg_g2_arr], ["ir1", "ir2", "g1", "g2"])
            
    # packet counter logic calculation
    MSense_data.ppg_packet_counter.append(packet_counter[0])
    packets_recived = MSense_data.ppg_packet_counter[len(MSense_data.ppg_packet_counter) - 1] - MSense_data.ppg_packet_counter[
        len(MSense_data.ppg_packet_counter) - 2]

    MSense_data.ppg_packet_loss_counter.append(10-packets_recived)

    time_string = str(datetime.datetime.now().time())
    MSense_data.ppg_date_time.append(time_string)

    horizontal_array = [Led_ir1, Led_ir2, Led_g15, Led_g2, packet_counter[0], 10-packets_recived]
    if use_lsl:
        global ppg_stream_outlet
        lsl_transmission.send_data(ppg_stream_outlet, horizontal_array)


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


### BioImpedance functions definitions ###



def BioImpedanceHandle(sender, data):
    if debug_print_updates:
        print("collecting data")
    ImpedanceMagRaw = data[0:4]
    ImpedanceMag = struct.unpack(">f", ImpedanceMagRaw)

    ImpedancePhaseRaw = data[4:8]
    ImpedancePhase = struct.unpack(">f", ImpedancePhaseRaw)

    ImpedanceCounterRaw = data[8:9]
    ImpedanceCounter = struct.unpack("<B", ImpedanceCounterRaw)
    #ImpedanceCounter = ord(ImpedanceCounter[0])

    MSense_data.BioImpedanceMag.append(ImpedanceMag[0])
    MSense_data.BioImpedancePhase.append(ImpedancePhase[0])
    MSense_data.BioImpedancePacketCounter.append(ImpedanceCounter[0])
    MSense_data.BioImpedanceTimeStamp.append(time.time())
    #MSense_data.BioImpedanceTimeStamp.append(str(datetime.datetime.now().time()))





def build_uuid_dict(client):
    characteristics = client.services.characteristics.values()
    characteristics = list(characteristics)
    uuid_arr = {}
    for characteristic in characteristics:
        uuid_arr[characteristic.uuid] = characteristic.handle

    return uuid_arr

def disconnect_callback(client):
    print("bleak error: device disconnected")
    # wait for 5 seconds to make sure the device doesn't reconnect, as per ble protocol?
    print(str(client.is_connected))
    #time.sleep(6)
    #if not client.is_connected:
    #    global file_name

    #    write_all_files(file_name)
    #    sys.exit()
    #else:
    #    print("device reconnected")



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
        #item = substring_in_dict(devi.name, device_set)
        #if item is not None:
            #device_object = deepcopy(item)
        if devi.name in device_set:
            device_object = device_set[devi.name]
            #device_object.name = devi.name
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

def test_function(address, path, record_length, options, test_flag, name):
    print("I am running in a test call!")
    print("imported")
    try:
        non_async_collect(address, path, record_length, options, test_flag, name)

    except Exception as err:
        print(err)

    print("I am done!")


# this is the function that is executed inside the GUI to make sure everything runs properly
def non_async_collect(address, path, max_length, collect_options, end_flag, name):
    print("starting non async collecion function with parameters:")
    print("address: " + address)
    print("path: " + path)
    print("collection options: " + str(collect_options))
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(run(address, True, path=path, data_amount=max_length, options=collect_options, status_flag=end_flag, Name=name))
    except Exception as e:
        print("bleak client backend bluetooth error")
        print(e)
        


async def run(address, debug=True, path=None, data_amount = 30.0, options:list[MSenseCharacteristic]=None, status_flag=None, Name="M"):
    global file_name
    file_name = path
    try:
        print("starting run function")
        # this has to be global because it is async
        global bleak_device
        global start_collection_date
        # just a little check to remember why this is global

        print(file_name)
        global file_obj
        start_collection_date += str(datetime.datetime.now())
        # windows doesn't like colons, so we have to remove them
        start_collection_date = start_collection_date.replace(":", "")
        assert options != None
        assert path != None

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
            #client.connect()
            x = client.is_connected
            client.__str__()
            print("connected to " + str(Name) + "!")
            logger.info("Connected: {0}".format(x))

            uuid_arr = build_uuid_dict(client)
            bleak_device = client

            write_pi = bytearray([0x01])

            # first try to get the battery level, which is known to always be uuid 2a19
            default_battery_characteristic = 'da39adf0-1d81-48e2-9c68-d0ae4bbd351f'
            use_battery = False

            battery_level = await check_battery(client)
            if battery_level is not None:
                print(battery_level)
                status_flag.value = battery_level # set_value(battery_level)

            current_services = []

            # loop thorugh our selected characteristics and check if they are located in the the device's
            # characteristics.

            # MSense 4 Needs enabling


            if  "MotionSenseHRV4" in Name or "MSense4" in Name:
                print("Found MSense4 Device!")
                unix_time = struct.pack("<Q", int(time.time()))
                encoding = compute_int_hash(path)
                await client.write_gatt_char(bleak.uuids.normalize_uuid_str("da39c932-1d81-48e2-9c68-d0ae4bbd351f"),
                                             unix_time)
                await client.write_gatt_char(bleak.uuids.normalize_uuid_str("da39c933-1d81-48e2-9c68-d0ae4bbd351f"), encoding)

                await client.write_gatt_char(bleak.uuids.normalize_uuid_str("da39c931-1d81-48e2-9c68-d0ae4bbd351f"), write_pi)
                if use_lsl:
                    global enmo_outlet
                    enmo_outlet = lsl_transmission.register_outlet(2, name=Name + "Enmo Timestamp",
                                                           type_array=["enmo", "pkt_counter"])
            else:
                if use_lsl:
                    global ppg_stream_outlet
                    global accelorometer_outlet
                    global led_outlet
                    ppg_stream_outlet = lsl_transmission.register_outlet(6, name=Name + " PPG", type_array=["ir1", "ir2", "g1", "g2", "packet counter", "packet loss"])
                    accelorometer_outlet = lsl_transmission.register_outlet(8, name=Name + " Accelerometer", type_array=["AccelX", "AccelY", "AccelZ", "AngX", "AngY", "AngZ", "PC", "PL"])
                    led_outlet = lsl_transmission.register_outlet(2, name=Name + " Led Status", type_array=["led", "PC"], hz=5)

            for characteristic in options:
                if characteristic.function is None:
                    continue
                try:

                    characteristic.uuid = bleak.uuids.normalize_uuid_str(characteristic.uuid) #characteristic.uuid.lower()
                    characteristic_number = uuid_arr[characteristic.uuid]
                    service = client.services.characteristics[characteristic_number]
                    print("Sucessfully obtained Service: " + str(service))

                except KeyError as e:
                    error_string = "Error: bluetooth characteristic UUID is invalid for device " + characteristic.name
                    print(error_string)

                except BaseException as e:
                    print("characteristic notify error" + e)

                current_services.append(service)

                print("starting notify for " + str(characteristic.name))
                
                status = await client.start_notify(service, characteristic.function)

            #we need to do the rest of the sensors as well
            #collect data
            
            for current_second in range(int(data_amount)):
                print("current seconds in collection for device: " + str(current_second) + "and status:" + str(status_flag))
                if (status_flag.value == -1) or not client.is_connected:
                    print("status triggered error, ending collection...")
                    break
                await asyncio.sleep(1.0)
            await disconnect(client, current_services, Name)
                
            

    except Exception as e:
        print("An Error Occured in the child thread during data Collection:")
        print(e)

    finally:
        try:
            print("trying to write to files")
            write_all_files(file_name)
        except Exception as e:
            print(e)
        print("all files written")


    return "Finished Data Collection for "


async def reset_device(address):
    reset_characteristic = "da39c934-1d81-48e2-9c68-d0ae4bbd351f"
    async with BleakClient(address) as client:
        try:
            print("resettting " + client.address)
            value = 68
            value = struct.pack("<I", value)
            await client.write_gatt_char(reset_characteristic, value)
        except Exception as e:
            print(e)
    print("reset command finished!")


def compute_int_hash(name:str):
    hash_object = hashlib.sha256(name.encode())
    hex_digest = hash_object.hexdigest()
    integer_representation = int(hex_digest, 16) % 32000
    byte_representation = struct.pack("<I", integer_representation)
    return byte_representation



def write_all_files(path = None):
    global sucessful_file_write
    if sucessful_file_write:
        return
    if path is None:
        global file_name
    else:
        file_name = path
    # write accelorometer data
    if not os.path.exists(file_name):

        os.makedirs(file_name, exist_ok=True)
    time_stamp = datetime.datetime.now().strftime("%d-%m-%Y-at%H-%M")
    
    csv_rows = list()
    for data_element in range(len(MSense_data.accelorometer_x)):
        csv_rows.append([MSense_data.accelorometer_x[data_element], MSense_data.accelorometer_y[data_element],
                         MSense_data.accelorometer_z[data_element], MSense_data.angular_velocity_x[data_element],
                         MSense_data.angular_velocity_y[data_element], MSense_data.angular_velocity_z[data_element],
                         MSense_data.accelorometer_packet_counter[data_element], MSense_data.accelorometer_packet_loss[data_element],
                         MSense_data.accelorometer_timestamp[data_element]])
    if len(csv_rows) != 0:
        MSense_data.accelorometer_file = open(file_name + "//Acelorometer" + time_stamp + ".csv", "w", newline="")
        csv_writer = csv.writer(MSense_data.accelorometer_file)
        csv_writer.writerow(["Acel_X", "Acel_Y", "Acel_Z", "AngVel_X", "AngVel_Y", "AngVel_Z", "PacketCounter", "PacketLoss", "Timestamp"])
        csv_writer.writerows(csv_rows)
        print("closing accelorometer file")
        MSense_data.accelorometer_file.close()



    
    csv_rows = list()
    for data_element in range(len(MSense_data.ppg_led1ir_arr)):
        csv_rows.append([MSense_data.ppg_led1ir_arr[data_element], MSense_data.ppg_led2ir_arr[data_element],
                         MSense_data.ppg_g1_arr[data_element], MSense_data.ppg_g2_arr[data_element],
                         MSense_data.ppg_packet_counter[data_element], MSense_data.ppg_packet_loss_counter[data_element],
                        MSense_data.ppg_date_time[data_element]])
    if len(csv_rows) != 0:    
        MSense_data.ppg_file = open(file_name + "//PPG" + time_stamp + ".csv", "w", newline="")
        csv_writer = csv.writer(MSense_data.ppg_file)

        csv_writer.writerow(["PPG_IR1", "PPG_IR2", "PPG_G1", "PPG_G2", "PacketCounter", "PacketLoss", "Timestamp"])
        csv_writer.writerows(csv_rows)
        print("closing ppg file")
        MSense_data.ppg_file.close()
        
        #show_graph(file_name +"unfiltered ppg graph", [MSense_data.ppg_led1ir_arr, MSense_data.ppg_led2ir_arr, MSense_data.ppg_g1_arr,
        #                         MSense_data.ppg_g2_arr], ["ir1", "ir2", "g1", "g2"], False)
    csv_rows = list()
    if len(MSense_data.enmo_data) > 0:
        for data_element in range(len(MSense_data.enmo_data)):
            csv_rows.append([MSense_data.enmo_data[data_element], MSense_data.enmo_packet_counter[data_element]])

        MSense_data.enmo_file = open(file_name + "//ENMO" + time_stamp + ".csv", "w", newline="")
        csv_writer = csv.writer(MSense_data.enmo_file)
        csv_writer.writerow(["Enmo", "Packet Counter"])
        csv_writer.writerows(csv_rows)
        MSense_data.enmo_file.close()


    print("begin BioImpedance Processing")
    csv_rows = list()
    for data_element in range(len(MSense_data.BioImpedancePhase)):
        csv_rows.append([MSense_data.BioImpedanceMag[data_element], MSense_data.BioImpedancePhase[data_element],
                         MSense_data.BioImpedancePacketCounter[data_element], MSense_data.BioImpedanceTimeStamp[data_element]])
    if len(csv_rows) != 0:
        MSense_data.BioImpedanceFile = open(file_name + "//BioImpedance" + time_stamp +".csv", "w", newline="")
        csv_writer = csv.writer(MSense_data.BioImpedanceFile)    
        csv_writer.writerow(["BioImpReal", "BioImpImaginary", "PacketCounter", "TimeStamp"])
        csv_writer.writerows(csv_rows)
        print("closing BioImpedance file")
        MSense_data.BioImpedanceFile.close()
    
    sucessful_file_write = True
    if not real_time_graph_updates:
        show_graph(file_name + "filtered ppg graph", [MSense_data.ppg_led1ir_arr, MSense_data.ppg_led2ir_arr, MSense_data.ppg_g1_arr,
                                 MSense_data.ppg_g2_arr], ["ir1", "ir2", "g1", "g2"], True)


def write_files(file_name:str, type:str, time_stamp:str, file_obj,
                csv_categories, csv_rows):
    if len(csv_rows) != 0:
        with open(file_name + type + time_stamp +".csv", "w", newline="") as file_obj:
            csv_writer = csv.writer(file_obj)    
            csv_writer.writerow(csv_categories)
            csv_writer.writerows(csv_rows)
            print("closing file")
            
# by just using plt, it now comes with auto zoom features which I somehow missed.

    # plot the data


def show_realtime_graph(title, data:list, labels:list, ppg_filter_passthrough=False):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.clear()
    for data_element in range(len(data)):
        row = len(data[data_element])
        if row > 250:
            row = 250
        real_y_data = data[data_element][-250:]
        real_x_data = numpy.arange(row)
        if ppg_filter_passthrough:
            Fs = 25  # sampling rate of PPG
            b = scipy.signal.firls(numtaps=33, bands=numpy.array([0, 0.2, 0.5, 2.5, 2.8, Fs / 2])
                                   , desired=numpy.array([0, 0, 1, 1, 0, 0]),
                                   weight=numpy.array([2000, 100, 1000]),
                                   fs=Fs)  # fit a filter
            real_y_data = scipy.signal.filtfilt(b, 1, real_y_data, axis=-1, padtype=None)
        
        ax.plot(real_x_data, real_y_data, label=labels[data_element])

    ax.legend()
    ax.set_title(title)

    # display the plot
    plt.pause(.001)
    




# shows a graph of ppg signals. data is a list of ppg signals (which is a list of samples.)
def show_graph(title, data:list, labels:list, ppg_filter_passthrough=False, pause=False):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    for data_element in range(len(data)):
        row = len(data[data_element])
        real_x_data = numpy.arange(row)
        real_y_data = data[data_element]
        if ppg_filter_passthrough:
            Fs = 25  # sampling rate of PPG
            b = scipy.signal.firls(numtaps=33, bands=numpy.array([0, 0.2, 0.5, 2.5, 2.8, Fs / 2])
                                   , desired=numpy.array([0, 0, 1, 1, 0, 0]),
                                   weight=numpy.array([2000, 100, 1000]),
                                   fs=Fs)  # fit a filter
            real_y_data = scipy.signal.filtfilt(b, 1, real_y_data, axis=-1, padtype=None)
        
        ax.plot(real_x_data, real_y_data, label=labels[data_element])

    
    ax.legend()
    # set the limits
    #ax.set_xlim([0, 1])
    #ax.set_ylim([-1000, 1000])
    
    ax.set_title(title)

    # display the plot

    # this may cause issues because we are supposed to shutdown this process after data
    # collection is done, which will shutdown this graph even if block=False.
    
    plt.show(block=True)







def show_impedance_graph(title):
    # by just using plt, it now comes with auto zoom features which I somehow missed.
    times = MSense_data.BioImpedanceTimeStamp
    # get the time where the array is equal to 10
    pre_times = numpy.array(times)
    bike_times = numpy.array(times)
    post_times = numpy.array(times)

    starting_time = times[0]


    pre_times -= (5 + starting_time)
    bike_times -= (10 + starting_time)
    post_times -= (15 + starting_time)

    pre_times = abs(pre_times)
    bike_times = abs(bike_times)
    post_times = abs(post_times)

    pre_index = numpy.argmin(pre_times)
    bike_index = numpy.argmin(bike_times)
    post_times = numpy.argmin(post_times)

    pre_bike_matrix = numpy.array(contruct_points_matrix(0, pre_index))
    bike_matrix = numpy.array(contruct_points_matrix(pre_index, bike_index))
    post_matrix = numpy.array(contruct_points_matrix(bike_index, post_matrix))
    
    # TODO: Figure this out
    pre_bike_y = []
    for array_element in pre_bike_matrix:
        pre_bike_y.append(numpy.mean(array_element))

    pre_bike_x = numpy.arange(len(pre_bike_y)) 


    

    # plot the data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(pre_bike_x, pre_bike_y)

    ax.legend()
    # set the limits
    # ax.set_xlim([0, 1])
    # ax.set_ylim([-1000, 1000])

    ax.set_title(title)

    # display the plot

    # this may cause issues because we are supposed to shutdown this process after data
    # collection is done, which will shutdown this graph even if block=False.
    plt.show(block=True)


def contruct_points_matrix(start_idx, stop_idx):
    net_array = []
    real_phase = MSense_data.BioImpedanceMag
    imagine_phase = MSense_data.BioImpedancePhase
    counter = MSense_data.BioImpedancePacketCounter
    # create the empty array
    for array_element in range(0, 100):
        net_array.append(list())

    for idx in range(start_idx, stop_idx):
        actual_idx = counter[idx]
        net_number = (real_phase[actual_idx] + imagine_phase[actual_idx])/ 2
        net_array[idx].append(net_number)

    return net_array



def construct_graph_from_csv(title, file):
    all_rows = []
    # should have used pandas here, but oh well
    # pandas implementation for reference:
    # a = pd.read_csv()
    # all_columns = numpy.transpose(all_rows)
    with open(file_name, "r", newline="") as file_obj:
            csv_reader = csv.reader(file_obj)    
            for index, row in enumerate(csv_reader):
                if index == 0:
                    titles = row
                else:
                    all_rows.append(row)
            
    print("closing file")
    all_columns = numpy.transpose(all_rows)
    show_graph(title, all_columns, titles)
        

async def disconnect(client, services:list, Name):

    if "MotionSenseHRV4" in Name or "MSense4" in Name:
        await client.write_gatt_char(bleak.uuids.normalize_uuid_str("da39c931-1d81-48e2-9c68-d0ae4bbd351f"),
                                     bytearray([0x00]))

    #try:
        #await client.disconnect()
    #except Exception as e:
    #    print("failed to disconnect")
    #    print(e)
    #print('properly disconnected')

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




async def collect_with_adress(address):
    loop = asyncio.get_event_loop()
    address = loop.run_until_complete(connect_address())
    loop.run_until_complete(run(address, True, notification_handler, data_amount=10))

    return True

async def check_battery(client):
    # for some reason the battery check right now causes issues, this boolean is here for now
    # to prevent this.
    check_battery = True
    battery_level = -9
    if check_battery:
        try:
            for characteristics in client.services.characteristics.values():
                if characteristics.description == "Battery Level":
                    battery_level = await client.read_gatt_char(characteristics.uuid)
                    print("Battery Level: " + str(battery_level[0]))
                    await asyncio.sleep(2.0)
                    battery_level = battery_level[0]
                    if battery_level > 2 and battery_level <= 100:
                        use_battery = True
                        return battery_level

        except BaseException as e:
            print("failed to get battery service")
            print(e)
    return battery_level



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
    loop.run_until_complete(run(address, True, notification_handler, data_amount = record_length))

atexit.register(write_all_files)

def substring_in_dict(substring:str, key_dict:dict):
    if substring is None:
        return None
    for key in key_dict.keys():
        if key in substring:
            return key_dict[key]
    return None



if __name__ =='__main__':
    turn_on(record_length=30)
