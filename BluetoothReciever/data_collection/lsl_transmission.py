"""Example program to demonstrate how to send a multi-channel time-series
with proper meta-data to LSL."""

import random
import time
import pylsl

from pylsl import StreamInfo, StreamOutlet, local_clock




def demo():
    # first create a new stream info (here we set the name to BioSemi,
    # the content-type to EEG, 8 channels, 100 Hz, and float-valued data) The
    # last value would be the serial number of the device or some other more or
    # less locally unique identifier for the stream as far as available (you
    # could also omit it but interrupted connections wouldn't auto-recover).

    info = StreamInfo('CustomTest', 'EEG', 8, 100, 'float32', 'myuid2424')

    # append some meta-data
    info.desc().append_child_value("manufacturer", "BioSemi")
    channels = info.desc().append_child("channels")
    for c in ["C3", "C4", "Cz", "FPz", "POz", "CPz", "O1", "O2"]:
        channels.append_child("channel")\
            .append_child_value("name", c)\
            .append_child_value("unit", "microvolts")\
            .append_child_value("type", "EEG")

    # next make an outlet; we set the transmission chunk size to 32 (4*8) samples and
    # the outgoing buffer size to 360 seconds (max.)
    outlet = StreamOutlet(info, 32, 360)
    while True:
        # make a new random 8-channel sample; this is converted into a
        # pylsl.vectorf (the data type that is expected by push_sample)
        mysample = [random.random(), random.random(), random.random(),
                    random.random(), random.random(), random.random(),
                    random.random(), random.random()]
        # get a time stamp in seconds (we pretend that our samples are actually
        # 125ms old, e.g., as if coming from some external hardware)
        stamp = local_clock()-0.125
        # now send it and wait for a bit
        outlet.push_sample(mysample, stamp)
        time.sleep(0.01)


def register_outlet(channel_num, name = "MotionSense", type_array = [], hz=25):
    info = StreamInfo(name, 'MotionSense', channel_num, hz, 'float32', 'myuid2428')

    # append some meta-data
    info.desc().append_child_value("manufacturer", "BioSemi")
    channels = info.desc().append_child("channels")
    for c in type_array:
        channels.append_child("channel") \
            .append_child_value("name", c) \
            .append_child_value("unit", "") \
            .append_child_value("type", c)

    # next make an outlet; we set the transmission chunk size to 32 (4*8) samples and
    # the outgoing buffer size to 360 seconds (max.)
    outlet = StreamOutlet(info, channel_num*4, 360)
    print("now ready to sending data...")
    return outlet

def send_data(outlet, data, custom_time_increment=0):
    #print("now sending data...")
    # make a new random 8-channel sample; this is converted into a
    # pylsl.vectorf (the data type that is expected by push_sample)
    mysample = []
    #print(data)
    for counter, sample in enumerate(data):
        mysample.append(sample)


    # get a time stamp in seconds (we pretend that our samples are actually
    # 125ms old, e.g., as if coming from some external hardware)
    stamp = local_clock()
    if custom_time_increment != 0:
        stamp += custom_time_increment
    # now send it and wait for a bit
    outlet.push_sample(mysample, stamp)




if __name__ == "__main__":
    demo()
