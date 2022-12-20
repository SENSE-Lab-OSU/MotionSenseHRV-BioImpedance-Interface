#!/usr/bin/env python
# coding: utf-8
import numpy

exp_id = '1_1'
run_training = False
if ((exp_id == '1_1') and (run_training)):
    assert False, "Switch run_training to False for exp_id=1_1."
# set get_data to Download the Data via. an automatic python script. Run this only once.
get_data = False
# this option is for if you already have a bluetooth configured MotionSense chip (see our 2nd tutorial on deploying
# and would like to recive the encoded signal within this program
use_bluetooth_reciver = True

if get_data:
    from data import get_data

    get_data.main()

# **Make sure parameters and sample data are in the correct folder**

# We will start by getting the necessary model parameters and sample data in order to first test the model. To do this, run the get_data.py file in the repository, or run the following code:

# In[2]:


# TensorFlow is an open source machine learning library
import tensorflow as tf
import time
# Keras is TensorFlow's high-level API for deep learning
from tensorflow import keras
from tensorflow.keras import layers

# Numpy is a math library
import numpy as np
# Pandas is a data manipulation library
import pandas as pd
# Scipy is a signal-processing library
from scipy.signal import detrend
import scipy.signal
# Matplotlib is a graphing library
import matplotlib.pyplot as plt
import bluetooth_reciver
# import seaborn as sns;sns.set()
# Math is Python's math library
import math
import os
import glob
import shutil
import pickle
# import some custom utility functions
from utils import make_data_pipe, Rpeak2HR, sliding_window_fragmentation
from sig2HR_model import create_model as create_model_HR
from sig2HR_model import create_infer_model as create_infer_model_HR
import sys
use_previous_data = True
use_previous_method = False
# tf.config.set_visible_devices([], 'GPU')



#for now we leave the test data here, will eventually be replaced after bug
#testing
def collect_data():
    #the previous method uses a convolutional method

    if use_previous_data:
        ppg_data = None
        ppg_green_1 = open("ppg_data_greench1", "rb")
        ppg_green_2 = open("ppg_data_greench2", "rb")
        green_a = pickle.load(ppg_green_1)
        green_b = pickle.load(ppg_green_2)
    else:
        bluetooth_reciver.turn_on(record_length=200)
        time.sleep(5)
        green_a = bluetooth_reciver.MSense_data.ppg_g1_arr
        green_b = bluetooth_reciver.MSense_data.ppg_g2_arr
        ppg_green_1 = open("ppg_data_greench1", "wb")
        ppg_green_2 = open("ppg_data_greench2", "wb")

        #serializing the two ppg signals into an array for reuse in use_previous_data
        pickle.dump(green_a, ppg_green_1)
        pickle.dump(green_b, ppg_green_2)

        ppg_green_1.close()
        ppg_green_2.close()

    #the reason why this does not work yet is because of two bluetooth versions
    acceleration_ar = bluetooth_reciver.MSense_data.accelorometer_data
    green_a = green_a[2000:-1]
    green_b = green_b[2000: -1]

    green_a_raw = numpy.array(green_a)
    green_b_raw = numpy.array(green_b)
    green_a_mean = float(np.mean(green_a))
    green_b_mean = float(np.mean(green_b))
    green_a = np.array(green_a)- green_a_mean
    green_b = np.array(green_b)-green_b_mean
    green_a = green_a/np.std(green_a)
    green_b = green_b/np.std(green_b)
    acceleration = []
    if use_previous_method:
        dd = dlmread('2019092801_3154_clean.csv');
        conv_filter = [0.006472,0.011553,0.008098,0.001543,0.003484,0.011371,0.011303,0.001802,-0.001942,0.006467,
0.011921,0.002491,-0.008549,-0.003595,0.007925,0.003195,-0.014319,-0.017678,-0.002243,0.002865,
-0.016935,-0.032973,-0.019191,-0.000164,-0.014191,-0.045205,-0.042638,-0.008331,-0.003738,-0.048396,
-0.073158,-0.026925,0.020820,-0.028855,-0.125639,-0.090593,0.139589,0.376075,0.376075,0.139589,
-0.090593,-0.125639,-0.028855,0.020820,-0.026925,-0.073158,-0.048396,-0.003738,-0.008331,-0.042638,
-0.045205,-0.014191,-0.000164,-0.019191,-0.032973,-0.016935,0.002865,-0.002243,-0.017678,-0.014319,
0.003195,0.007925,-0.003595,-0.008549,0.002491,0.011921,0.006467,-0.001942,0.001802,0.011303,
0.011371,0.003484,0.001543,0.008098,0.011553,0.006472]

        b = [0.000650, 0.000794, 0.000688, 0.000533, 0.000627, 0.000954, 0.001155, 0.000982, 0.000659, 0.000638, ...
             0.000992, 0.001266, 0.001036, 0.000506, 0.000300, 0.000646, 0.001017, 0.000771, 0.000028, -0.000415, ...
             - 0.000105, 0.000403, 0.000214, -0.000710, -0.001411, -0.001155, -0.000455, -0.000482, -0.001513,
             -0.002469, ...
             - 0.002269, -0.001312, -0.001052, -0.002091, -0.003278, -0.003138, -0.001861, -0.001181, -0.002117,
             -0.003518, ...
             - 0.003459, -0.001821, -0.000597, -0.001330, -0.002954, -0.003046, -0.001047, 0.000820, 0.000372,
             -0.001532, ...
             - 0.001923, 0.000370, 0.002954, 0.002859, 0.000566, -0.000365, 0.002066, 0.005411, 0.005744, 0.002926, ...
             0.001114, 0.003427, 0.007553, 0.008441, 0.004977, 0.001864, 0.003685, 0.008599, 0.010270, 0.006131, ...
             0.001249, 0.002087, 0.007779, 0.010617, 0.005951, -0.001162, -0.001932, 0.004496, 0.009097, 0.004324, ...
             - 0.005423, -0.008594, -0.001543, 0.005674, 0.001603, -0.011090, -0.017716, -0.010309, 0.000720, -0.001287,
             ...
             - 0.017156, -0.028798, -0.021629, -0.005033, -0.002684, -0.021962, -0.041388, -0.035822, -0.010622,
             0.000566, ...
             - 0.022634, -0.056333, -0.056281, -0.015048, 0.018012, -0.010410, -0.084063, -0.112514, -0.017492,
             0.173084, ...
             0.329006, 0.329006, 0.173084, -0.017492, -0.112514, -0.084063, -0.010410, 0.018012, -0.015048, -0.056281,
             ...
             - 0.056333, -0.022634, 0.000566, -0.010622, -0.035822, -0.041388, -0.021962, -0.002684, -0.005033,
             -0.021629, ...
             - 0.028798, -0.017156, -0.001287, 0.000720, -0.010309, -0.017716, -0.011090, 0.001603, 0.005674, -0.001543,
             ...
             - 0.008594, -0.005423, 0.004324, 0.009097, 0.004496, -0.001932, -0.001162, 0.005951, 0.010617, 0.007779,
             ...
             0.002087, 0.001249, 0.006131, 0.010270, 0.008599, 0.003685, 0.001864, 0.004977, 0.008441, 0.007553, ...
             0.003427, 0.001114, 0.002926, 0.005744, 0.005411, 0.002066, -0.000365, 0.000566, 0.002859, 0.002954, ...
             0.000370, -0.001923, -0.001532, 0.000372, 0.000820, -0.001047, -0.003046, -0.002954, -0.001330, -0.000597,
             ...
             - 0.001821, -0.003459, -0.003518, -0.002117, -0.001181, -0.001861, -0.003138, -0.003278, -0.002091,
             -0.001052, ...
             - 0.001312, -0.002269, -0.002469, -0.001513, -0.000482, -0.000455, -0.001155, -0.001411, -0.000710,
             0.000214, ...
             0.000403, -0.000105, -0.000415, 0.000028, 0.000771, 0.001017, 0.000646, 0.000300, 0.000506, 0.001036, ...
             0.001266, 0.000992, 0.000638, 0.000659, 0.000982, 0.001155, 0.000954, 0.000627, 0.000533, 0.000688, ...
             0.000794, 0.000650]




        #green_a = np.convolve(green_a, conv_filter)
        #green_b = np.convolve(green_b, conv_filter)
    else:
        Fn = 25
        b1_desired = (0, 0, 1, 1, 0, 0)
        b1_bands = (0, 0.3*2/Fn, 0.5*2/Fn, 4.5*2/Fn, 5*2/Fn, 1)
        b2_desired = (0, 0, 1, 1)
        b2_bands = (0, 0.3*2/Fn, 0.5*2/Fn, 1)

        b1_fir_filter = scipy.signal.firls(221, b1_bands,b1_desired, weight=(10*0.02 ,0.02,0.02))
        b2_fir_filter = scipy.signal.firls(221, b2_bands, b2_desired, weight=(10*0.02 ,0.02))

        green_a = scipy.signal.filtfilt(b1_fir_filter, 1, green_a)
        green_b = scipy.signal.filtfilt(b1_fir_filter, 1, green_b)




    for x in acceleration_ar:
        #technically this could be written via a list comprehension, but this feels cleaner
        acceleration.append((x[0]*x[0])+(x[1]*x[1])+(x[2]+x[2]))

    total_ppgs = []
    #should have just combined the two arrays, but oh well
    number_of_samples = int(len(green_a)/800)
    if number_of_samples == 0:
        raise Exception("This model accepts samples > 800! Please set your record time to a longer length")
    for x in range(0, number_of_samples):
        next_inc = x+1
        ppg_data = np.array((green_a[x*800:next_inc*800], green_b[x*800:next_inc*800]))
        ppg_data = np.squeeze(ppg_data)
        ppg_data = np.transpose(ppg_data)
        #ppg_data = np.reshape(ppg_data, (ppg_data.shape[1], ppg_data.shape[2]))
        ppg_data = np.expand_dims(ppg_data, axis=0)
        total_ppgs.append(ppg_data)

    #if len(total_ppgs) > 1:
    total_ppgs = np.concatenate(total_ppgs, axis=0)

    #else:
        #total_ppgs = np.array(total_ppgs)
        #total_ppgs = np.squeeze(total_ppgs)

    packet_loss = bluetooth_reciver.MSense_data.ppg_packet_loss_counter
    return total_ppgs, acceleration, green_a_raw, green_b_raw

ppg_data, acceleration, green_a_raw, green_b_raw = collect_data()
#ppg_signal_arr = np.array(bluetooth_reciver.MSense_data.ppg_g1_arr)
path_prefix = 'data/pre-training'

# current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
# log_prefix='../experiments/{}_{}'.format(exp_id,current_time)
log_prefix = 'data/post-training/experiments/{}'.format(exp_id)

# path_prefix= 'E:/Box Sync/' #'C:/Users/agarwal.270/Box/' #
path = (path_prefix + '/')
val_files = [path + '2019092801_3154_clean.csv']
test_files = [path + '2019092820_5701_clean.csv']
win_len = 8  # in sec
step = 1  # in n_samples
Fs_pks = 100  # in Hz


# Now that we've configured our paths and imports, we should start by first loading our training data for the model, located in the repository as a spreadsheet csv file of a sample recorded ppg signal through time. We do this by using pd.readcsv(), whitch gets our data, and then we reformat and combine using numpy concatenations.

# In[3]:


def get_train_data(path, val_files=[], test_files=[],
                   win_len=8, step=1, Fs_pks=100):
    '''
    Use all files in the folder 'path' except the val_files and test_files
    '''

    def get_clean_ppg_and_ecg(files):
        list_clean_ppg = [];
        list_arr_pks = []
        for i in range(len(files)):
            df = pd.read_csv(files[i], header=None)
            arr = df.values
            if 'clean' in files[i]:
                arr[:, 41:45] = (detrend(arr[:, 41:45].reshape(-1), 0, 'constant')
                                 ).reshape((-1, 4))
                list_clean_ppg += [np.concatenate([arr[:, 29:31], arr[:, 41:45]],
                                                  axis=-1), arr[:, 39:41]]
                list_arr_pks += [arr[:, 45:49].reshape(-1)]
        return list_clean_ppg, list_arr_pks

    files = glob.glob(path + '*.csv')
    # files=[fil for fil in files if 'WZ' in fil] #get wenxiao's data
    # separate val and test files
    s3 = set(files);
    s4 = set(val_files + test_files)
    files_2 = list(s3.difference(s4))
    # files_2=[files_2[0]]
    # files_2=[fil for fil in files if not((val_names[0] in fil))]
    list_clean_ppg, list_arr_pks = get_clean_ppg_and_ecg(files_2)

    dsample_factr = 4
    Fs_pks = int(Fs_pks / dsample_factr)
    win_len = win_len * Fs_pks

    list_r_pk_locs = [np.arange(len(arr_pks))[arr_pks.astype(bool)] for
                      arr_pks in list_arr_pks]

    # get nearest dsampled idx
    # TODO: Started using round instead of floor
    list_r_pk_locs_dsampled = [np.round(r_pk_locs / dsample_factr).astype(int) for
                               r_pk_locs in list_r_pk_locs]
    # print([np.max(r_pks) for r_pks in list_r_pk_locs_dsampled])
    # print([len(ppg) for ppg in list_clean_ppg[::4]])

    list_arr_pks_dsampled = []
    for j in range(len(list_arr_pks)):
        arr_pks_dsampled = np.zeros([int(len(list_arr_pks[j]) / dsample_factr), 1])
        # check & correct for rare rounding up issue in the last element
        if list_r_pk_locs_dsampled[j][-1] == len(arr_pks_dsampled):
            list_r_pk_locs_dsampled[j][-1] -= 1
        arr_pks_dsampled[list_r_pk_locs_dsampled[j]] = 1
        list_arr_pks_dsampled.append(arr_pks_dsampled)
    # print([len(ppg) for ppg in list_arr_pks_dsampled])

    list_HR = [2 * [Rpeak2HR(arr_pks, win_len, step, Fs_pks)]
               for arr_pks in list_arr_pks_dsampled]
    list_HR = sum(list_HR, [])
    # list_HR=[HR[::dsample_factr] for HR in list_HR]

    return list_clean_ppg, list_HR


list_sigs, list_HR = get_train_data(path, val_files, test_files, win_len,
                                    step, Fs_pks)

# **import training data from csv file**

# **Convert and Visualize Data**

# We will feed the data to the network using tensorflow's [tf.data](https://www.tensorflow.org/guide/data) pipeline which comes with lots of benefits (check out the link to learn more).



# Pre-process data
dsample_factr = 4;
Fs_new = int(Fs_pks / dsample_factr)
sample_win_len, step_size = win_len * Fs_new, 2 * Fs_new
HR_win_len = sample_win_len * 3  # TODO: Can change this later, 4 is arbitrary choice after profs suggestion
ppg_win_len = sample_win_len + HR_win_len

model_sigs_in, model_HR_out = [], []
for j in range(len(list_HR)):
    # HR=list_HR[j][list_arr_pks[j].astype(bool)]
    ppg, HR = list_sigs[j][:, 0:2], list_HR[j]
    ppg = sliding_window_fragmentation([ppg], ppg_win_len, step_size)
    HR = sliding_window_fragmentation([HR], HR_win_len, step_size)
    # print(len(ppg),len(HR))
    model_sigs_in.append(ppg)
    model_HR_out.append(HR[:len(ppg)])  # clipping extra HRs at the end
model_sigs_in = np.concatenate(model_sigs_in, axis=0)
model_HR_out = np.concatenate(model_HR_out, axis=0)

#ok right here we need to switch out the test data and get our actual data



if True:
    decoder_noq = tf.keras.models.load_model("data/post-training/models/decoder_noq_s_model")
    HR_prediction_model = tf.keras.models.load_model("data/post-training/models/hr_prediction_s_model")
    encoder = tf.keras.models.load_model("data/post-training/models/encoder_s_model")
    decoder = tf.keras.models.load_model("data/post-training/models/decoder_s_model")
    model_e2e = tf.keras.models.load_model("data/post-training/models/model_e2e")




ppg_out_list = []
HR_out_list = []

dec_mem=np.zeros([1,4])
HR_mem=np.zeros([1,64])

#now we will feed the data through the model. we feed it iteratively
#by passing the data in (1, 800, 2) sizes, which is our input tensor size for the autoencoder.
for i in range(ppg_data.shape[0]):
    extra_dim_ppg = numpy.expand_dims(ppg_data[i:i + 1], axis=0)
    z = model_e2e.predict(extra_dim_ppg)
    HR_out_list.append(z)


HR_hat_e2e = np.concatenate(HR_out_list, axis=0)
ppg_data *= 3

#Visualize our PPG signal
idx=1
plt.figure()
plt.subplot(211)
plt.title('A sample PPG and HR')
plt.plot(ppg_data[idx,:,:])
#plt.plot(model_sigs_in[idx,:,:])
plt.ylabel('PPG')
plt.grid(True)
plt.subplot(212)
plt.plot(HR_hat_e2e[idx,:])
plt.plot(model_HR_out[idx,:])
plt.ylabel('HR (BPS)')
plt.grid(True)
plt.xlabel('Sample No.')
plt.show()

plt.figure()
plt.plot(green_a_raw)
plt.plot(green_b_raw)
plt.ylabel('unprocessed PPG')
plt.xlabel("Sample No.")
plt.grid(True)
plt.show()



#partition
val_perc=0.14
val_idx=int(val_perc*len(model_in))

def process_auto_encoded_data():
    pass