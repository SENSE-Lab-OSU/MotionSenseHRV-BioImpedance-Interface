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


use_previous_packet_format = True


'''This is a class that holds a Device, consisting of bluetooth characteristics'''
class MSenseDevice:

    def __init__(self, ):
        pass


''' This is a class for holding information about a single bluetooth attribute.'''
class MSenseCharacteristic:


    def __init__(self, name, function, uuid):
        self.name = name
        self.function = function
        self.uuid = uuid




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




async def disconnect_from_clients():
    if type(bleak_device) == BleakClient:
        await bleak_device.disconnect()

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



#this is not actually used
def motion_sense_characteristic(sender, data):
    m_service = bleak_device.services.characteristics[30]
    #await bleak_device.read_gatt_char(m_service)
    accelorometer_data = []
    Accelorometer_X = data[0:2]
    print(Accelorometer_X)
    Accelorometer_Y = data[2:4]
    Accelerometer_Z = data[4:6]
    Angular_velocity_X = data[6:10]
    Angular_velocity_Y = data[10:14]
    Angular_velocity_Z = data[14:18]
    Accelorometer_X = struct.unpack(">h", Accelorometer_X)
    Accelorometer_Y = struct.unpack(">h", Accelorometer_Y)
    Accelerometer_Z = struct.unpack(">h", Accelerometer_Z)

    Angular_velocity_X = struct.unpack("<f", Angular_velocity_X)
    Angular_velocity_Y = struct.unpack("<f", Angular_velocity_Y)
    Angular_velocity_Z = struct.unpack("<f", Angular_velocity_Z)



    MSense_data.accelorometer_x = Accelorometer_X
    MSense_data.accelorometer_y = Accelorometer_Y
    MSense_data.accelorometer_z = Accelerometer_Z
    MSense_data.angular_velocity_x = Angular_velocity_X
    MSense_data.angular_velocity_y = Angular_velocity_Y
    MSense_data.angular_velocity_z = Angular_velocity_Z
    if MSense_data.print_data_to_files:
        m_file = open("motionsense_data.txt", "a")

        info_string = "accelorometer: " + str(Accelorometer_X) + " " + str(Accelorometer_Y) + " " + str(Accelerometer_Z) + " Angular velocity: "
        info_string += str(Angular_velocity_X) + " " + str(Angular_velocity_Y) + " " + str(Angular_velocity_Z)
        m_file.write(info_string + "\n")
    #print(info_string)
    #print(str(Angular_velocity_X))
    #print(str(Angular_velocity_Y))
    #print(str(Angular_velocity_Z))

   



def notification_handler_magnometer(sender, data):
    magnometer_ints = data[0:6]
    print("unpacking magnometer data")
    nk_data = struct.unpack("<hhh", magnometer_ints)
    #print(nk_data)
    magnometer_packet_information = data[6:8]

    magnometer_file = open(file_name + "\\" + start_collection_date + "-magnometer.txt", "a")

    magnometer_packet_information = struct.unpack(">h", magnometer_packet_information)


    m_string = "magnometer: "
    magnometer_file.write(magnometer_packet_information)





def get_data_obj_from_uuid(uuid, clinet_list):
    for attr in clinet_list:
        pass




def build_uuid_dict(client):
    characteristics = client.services.characteristics.values()
    characteristics = list(characteristics)
    uuid_arr = {}
    for characteristic in characteristics:
        uuid_arr[characteristic.uuid] = characteristic.handle

    return uuid_arr


def ppg_sensor_handle(sender, data:bytes):
    global file_name
    print(sender)
    if use_previous_packet_format:
    	pass
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


    #ledir1_ex = bitarray.bitarray()
    #ledir1_ex.frombytes(bytes([data[2]]))
    #ledir1_ex >>= 5
    #Led_ir1 += ledir1_ex.tobytes()
    #Led_ir1 += bytes([0])
    #Led_ir1 = bitarray.bitarray(Led_ir1)

    ppg_file = open(file_name + "\\" + start_collection_date + "-ppg.txt", "a")


    #print(Led_ir1)
    arr = bytearray(data)
    hex_string = ''
    for x in arr:
        hex_string += hex(x) + " "
    #Led_ir1 = struct.unpack("<l", Led_ir1)
    hex_string += " interpreted value: Led IR1:" + str(Led_ir1) + " LED IR2 " + str(Led_ir2) + " LED G1 " + str(Led_g15) + " LED G2 " + str(Led_g2)
    #Led_ir2 = struct.unpack("<l", data[3:6].rjust(4, bytes(1)))
    #Led_g1 = struct.unpack("<l", data[6:9].rjust(4, bytes(1)))
    #Led_g2 = struct.unpack("<l", data[9:12].rjust(4, bytes(1)))
    packet_counter = data[10:12]
    print("ppg packet counter: " + str(packet_counter))
    packet_counter = struct.unpack(">h", packet_counter)
    ppg_file.write(hex_string + "\n")
    print(packet_counter[0])
    MSense_data.ppg_led1ir_arr.append(Led_ir1)
    MSense_data.ppg_led2ir_arr.append(Led_ir2)
    MSense_data.ppg_g1_arr.append(Led_g15)
    MSense_data.ppg_g2_arr.append(Led_g2)

    file_dict = {"green_1": Led_g15, "green_2": Led_g2, "ir_1":Led_ir1,
                 "ir_2":Led_ir2}


    # if this does not work use this forat:
    mydict = [{'branch': 'COE', 'cgpa': '9.0', 'name': 'Nikhil', 'year': '2'},
              {'branch': 'COE', 'cgpa': '9.1', 'name': 'Sanchit', 'year': '2'},
              {'branch': 'IT', 'cgpa': '9.3', 'name': 'Aditya', 'year': '2'},
              {'branch': 'SE', 'cgpa': '9.5', 'name': 'Sagar', 'year': '1'},
              {'branch': 'MCE', 'cgpa': '7.8', 'name': 'Prateek', 'year': '3'},
              {'branch': 'EP', 'cgpa': '9.1', 'name': 'Sahil', 'year': '2'}]


    # currently, writing to a csv file is not finished yet, so it is commented out for now
    #write_to_csv_file("ppg", file_dict)
    MSense_data.ppg_packet_counter.append(packet_counter[0])
    packets_recived = MSense_data.ppg_packet_counter[len(MSense_data.ppg_packet_counter) - 1] - MSense_data.ppg_packet_counter[
        len(MSense_data.ppg_packet_counter) - 2]

    MSense_data.ppg_packet_loss_counter.append(8-packets_recived)

    #print(MSense_data.ppg_led1ir_arr)


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




def orientation_handler(sender, data):
    pass

async def connect_address():
    print("scanning connections")
    addr = None
    motion_sense_devices = []
    devices = await BleakScanner.discover()
    for devi in devices:
        if devi.name == "MotionSenseHRV3":
            addr = devi.address
            motion_sense_devices.append(devi)
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


def write_to_file(name: str, data):
    # we will now create a viewable string that can be used

    for data_idx in data:
        name += str(data_idx) + " "
    print(name)

    if name == "magnometer":
        if MSense_data.print_data_to_files:
            magnometer_file.write(name + "\n")

        MSense_data.magnometer.append(data)
        MSense_data.magnometer_packet.append(magnometer_packet_information)

async def run(address, debug=True, path=None, data_amount = 30.0, options=None):
    print("starting run function")
    # this has to be global because it is async
    global bleak_device
    global start_collection_date
    # just a little check to remember why this is global
    global file_name
    print(file_name)
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

            service = client.services.characteristics[uuid_arr[characteristic.uuid]]

            #ppg_service = client.services.characteristics[uuid_arr["da39c923-1d81-48e2-9c68-d0ae4bbd351f"]]

            current_services.append(service)
            #create_csv_file("ppg", path)
            print("starting notify for " + str(characteristic.name))
            ppg_arr = await client.start_notify(service, characteristic.function)



            #orientation_service = client.services.characteristics[uuid_arr["da39c926-1d81-48e2-9c68-d0ae4bbd351f"]]
        #await client.start_notify(motion_sense_service, motion_sense_characteristic )

            #motion_sense_service = client.services.characteristics[uuid_arr["da39c924-1d81-48e2-9c68-d0ae4bbd351f"]]
            #current_services.append(motion_sense_service)
            #await client.start_notify(motion_sense_service, motion_sense_characteristic)


            #magnometer_service = client.services.characteristics[uuid_arr["da39c925-1d81-48e2-9c68-d0ae4bbd351f"]]
            #current_services.append(magnometer_service)
            #await client.start_notify(magnometer_service, notification_handler_magnometer)


        #we need to do the rest of the sensors as well
        #collect data
        await asyncio.sleep(data_amount)


        await close_fies(client, current_services)

        #j = MSense_data
        ppg_pack = MSense_data.ppg_packet_counter
        ppg_pack_loss = MSense_data.ppg_packet_loss_counter

        magno_pack = MSense_data.magnometer_packet


#async def enable_sec():

async def close_fies(client, services:list):


    try:
        MSense_data.ppg_file.close()
    except:
        print("file does not exist")

    for service in services:
        await client.stop_notify(service)
    await client.disconnect()

    print('properly disconnected')




def start_background_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


def non_async_connect():
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
    return address


# this is the function that is executed inside the GUI to make sure everything runs properly
def non_async_collect(address, path, max_length, collect_options, end_flag):
    print("starting non async collecion function with parameters:")
    print("address: " + address)
    print("path: " + path)
    print("collection options: " + str(collect_options))
    #address = "E0:06:E0:EA:CF:77"
    #path = "D:\tfdownload\OSUMotionSenseChip\MotionSenseHRV_v3_private\software\tutorials\AEHR_model_tutorial\bluetooth_data_collection\data"

    max_length = 15.0
    #print(collect_options)
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(run(address, True, path=path, data_amount=max_length, options=collect_options))
    except bleak.exc.BleakDotNetTaskError():
        print("bleak client backend bluetooth error")




async def collect_with_adress(address):
    loop = asyncio.get_event_loop()
    address = loop.run_until_complete(connect_address())
    loop.run_until_complete(run(address, True, notification_handler, data_amount=record_length))

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
