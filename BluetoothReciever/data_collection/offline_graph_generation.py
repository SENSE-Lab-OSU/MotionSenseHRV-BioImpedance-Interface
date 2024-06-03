import csv
import numpy
import matplotlib.pyplot as plt
import pandas as pd
import scipy

mydataset = {
  'cars': ["BMW", "Volvo", "Ford"],
  'passings': [3, 7, 2]
}



print(list(myvar['cars']))

path = "path here"
title = "title here"
def get_data():
    data = []
    labels = []
    first_time = True
    reader = csv.reader(open(path), delimiter=",")
    full_data = []
    for row in reader:
        if first_time:
            labels.extend(row)
            first_time = False
        else:
            data.append(row)
    data = numpy.transpose(data)
    # just making the data redundant instead


def pd_graph_generation(title:str, path:str, labels:list):
    #data_set = pd.read_csv()
    data_set = pd.DataFrame(mydataset)
    signals = []
    for label in labels:
        signals.append(list(data_set[label]))
    show_graph(title, signals, labels, False)

# shows a graph of ppg signals. data is a list of ppg signals (which is a list of samples.)
def show_graph(title, data: list, labels: list, ppg_filter_passthrough=False):
    # by just using plt, it now comes with auto zoom features which I somehow missed.

    # plot the data
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
    # ax.set_xlim([0, 1])
    # ax.set_ylim([-1000, 1000])

    ax.set_title(title)

    # display the plot

    # this may cause issues because we are supposed to shutdown this process after data
    # collection is done, which will shutdown this graph even if block=False.
    plt.show(block=True)


