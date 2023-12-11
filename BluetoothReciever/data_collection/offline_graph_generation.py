import csv
import numpy
import bluetooth_reciver


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
    bluetooth_reciver.show_graph(title, data, labels)








