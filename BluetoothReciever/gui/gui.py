import os
import sys
import time
import atexit
import PyQt5.QtBluetooth
from PyQt5.QtCore import QRunnable, QThreadPool
import PyQt5.QtCore
import multiprocessing
from PyQt5.QtCore import pyqtSlot
from datetime import datetime

print(str(datetime.now()))


'''This is the UI script for the OSU MotionSense Bluetooth
device

This code is designed to be as dependency free as possible,
so that making new scripts for data collection can be easily be swapped out by
replacing singular function calls to collect_data'''


import PyQt5
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFormLayout,
    QLineEdit,
    QVBoxLayout,
    QWidget,
    QLabel,
    QScrollArea,
    QMainWindow

)
from PyQt5.QtGui import QPixmap, QFont

from PyQt5.QtCore import Qt
from datetime import datetime
import PyQt5.QtWidgets

import PyQt5.Qt
from data_collection import bluetooth_reciver

threadpool = QThreadPool()

bold_font = QFont()
bold_font.setBold(True)
bold_font.setPointSize(12)


# our __init__  should be dependency free
# for device or bluetooth backend specific functionalities, modify the functions
# for connecting and updating
class MotionSenseApp(QWidget):
    def __init__(self):

        super().__init__()
        self.threads = []

        # a logging file for notes and documentation.
        # is given a file once data collection starts
        self.logging_file = None
        #
        self.devices = []
        self.addresses = []



        # initialize all of the components of the UI - The Checkboxs, Line Edits, Logo, Text, what have you not

        # Create an outer layout
        outerLayout = QVBoxLayout()

        # create a seperate layout just for logo
        picture_layout = QFormLayout()

        # Create a form layout for the label and line edit
        topLayout = QFormLayout()

        #try to add OSU logo image
        image = QLabel(self)
        pixmap = QPixmap('gui/OSU.png')
        pixmap.scaled(6, 6)
        image.setPixmap(pixmap)
        image.setAlignment(Qt.AlignCenter)
        picture_layout.addWidget(image)

        # Add a label and a line edit to the form layout
        header_label = QLabel("Welcome to OSU SENSELAB MotionSense Datacollection!")

        # Setting title text font to be bold and big
        font = PyQt5.Qt.QFont()
        font.setBold(True)
        font.setPointSize(14)
        header_label.setFont(font)
        topLayout.addWidget(header_label)

        self.file_line = QLineEdit()
        self.file_line.setText(os.getcwd())

        self.file_line2 = QLineEdit()
        self.file_line2.setText("data")

        # we create a counter for max length
        self.file_line3 = QLineEdit()
        self.file_line3.setText(str(180.0))
        topLayout.addRow("Save Location:", self.file_line)
        topLayout.addRow("Folder Name:", self.file_line2)
        topLayout.addRow("Stop Recording Data after: (sec)", self.file_line3)

        self.enable_csv = QCheckBox()
        topLayout.addRow("Convert to CSV after collection", self.enable_csv)


        # layout for connecting to motionsense UI, with a button to connect
        collections_layout = QFormLayout()
        self.button = PyQt5.QtWidgets.QPushButton("Connect to MotionSense")
        #button.setText("lol")
        self.gather_button = PyQt5.QtWidgets.QPushButton("Start data_collection")
        self.gather_button.setDisabled(True)


        self.th_log = QLineEdit()
        self.log_button = PyQt5.QtWidgets.QPushButton("log text to file:")
        self.log_button.clicked.connect(self.send_note)
        collections_layout.addRow(self.log_button, self.th_log)
        collections_layout.addWidget(self.button)
        collections_layout.addWidget(self.gather_button)

        # add out button action to the widget
        self.button.clicked.connect(self.update_connect_ui)
        # when self.button is clicked, it will now call self.update_connect_ui
        self.button.clicked.connect(self.connect_to_motionsense_handle)

        self.optionsLayout = QFormLayout()
        self.gather_button.clicked.connect(self.collect_data)

        self.log_disp = PyQt5.QtWidgets.QLabel("Waiting to connect to a device...")
        self.log_disp.setFont(bold_font)

        collections_layout.addWidget(self.log_disp)
        self.notice = PyQt5.QtWidgets.QLabel("Note: In order to run this you will need to have "
                                                            "an apropriate MotionSenseDevice ready and turned on!")
        collections_layout.addWidget(self.notice)

        # Nest the inner layouts into the outer layout
        outerLayout.addLayout(picture_layout)
        outerLayout.addLayout(topLayout)
        outerLayout.addLayout(self.optionsLayout)
        outerLayout.addLayout(collections_layout)

        # Set the window's main layout
        self.setLayout(outerLayout)

        self.currently_collecting = False



    '''this is the connection function that is only executed upon the 'connect' button press
    '''
    def connect_to_motionsense_handle(self):
        print("button pressed")

        connect_addresses = bluetooth_reciver.non_async_connect()
        if len(connect_addresses) != 0:
            self.log_disp.setText("collection will be saved to")
            self.button.setText("connected!")
            self.button.setDisabled(True)
            self.gather_button.setEnabled(True)
            self.addresses.extend(connect_addresses)

            # for every MotionSense ble device found add it to the list of devices
            for count, adress in enumerate(self.addresses):
                device = MotionSense_device_QWidget(count, adress.address, adress.name)
                self.devices.append(device)
                self.optionsLayout.addWidget(device)
            path = self.file_line.text() + "\\" + self.file_line2.text()
            print(path)

            # try and create the directory for storing data
            self.log("making directories..")
            try:
                os.mkdir(path)
            except FileExistsError:
                self.log("directory already exists, keeping old")
            except:
                self.log("Could not create the directory! " + str(path) + " is invalid.")
                return
            finally:
                self.log("moving on")
            self.log("sucessfully created directories!")
            self.logging_file = open(path + "\\" + "log.txt", "w")
            self.log("created and activated log file...")

        else:
            print("failed to connect")
            self.log_disp.setText("failed to connect!")
            self.button.setText("Connect to MotionSense")
            self.button.setEnabled(True)
        QApplication.processEvents()



    def update_connect_ui(self):
        self.button.setText("connecting...")
        self.button.setDisabled(True)
        self.log_disp.setText("Trying to Connect")
        self.update()
        QApplication.processEvents()




    # this is device/implementation specific, and may require some modifications
    # for future versions to work
    def collect_data(self):
        print("collect button pressed")
        # we assume here that address is valid, because this button should be greyed out if there
        # is no connection

        if self.currently_collecting:
            # this means the button has been pressed before, and we need to stop it
            self.currently_collecting = False
            self.log("Stopped data collection")
            self.gather_button.setText("Saved!")
            self.log_disp.setText("Stopping Collection and saving to file...")
            self.gather_button.setText("Start")
            self.logging_file.close()

            print("trying to close app")
            for thread in self.threads:
                thread.close()

            print(datetime.now())
            return
        else:
            # need to figure out how to handel async events here

            path = self.file_line.text() + "\\" + self.file_line2.text()
            self.log("trying to start collection...")
            try:
                record_length = float(self.file_line3.text())
            except ValueError:
                record_length = 180.0
                self.log_disp.setText("record length input invalid, defaulting to 180.0")



            self.log("registering devices...")
            # for all MSense devices, get the characteristics that are checked and collect data from them
            for device in self.devices:
                self.log("registering device " + str(device.address))
                options = device.get_characteristics()
                if len(options) == 0:
                    # if the user didn't check any boxes we don't need to run any data
                    return
                self.log("got options...")
                self.log("creating thread...") #data_collection.bluetooth_reciver.non_async_collect

                th_thread = Collection_Worker(bluetooth_reciver.non_async_collect, device.address,
                                              path, record_length, options)


                # a bit of a messy solution. For more compatibility options,
                # use PyQt threadpool
                self.log("creating child shared memory flag for end")
                exit_flag = multiprocessing.Value("b")
                self.log("creating Process...")
                p = multiprocessing.Process(target=test_function, args=(device.address, path, record_length, options, exit_flag))
                self.log("attempting to start thread")
                p.start()


                #threadpool.start(th_thread)
                #th_thread.run(self.addresses, path, record_length, options)
                self.log("adding thread to list")
                self.threads.append((p, exit_flag))
                #data_collection.bluetooth_reciver.non_async_collect(self.address, path, record_length, options)
            #start logging for files
            self.log("sucessfully registered all devices")
            self.currently_collecting = True
            self.log("started data collection")
            # need to program this to get the data in the future
            # This is a class of bool values containing what options we would like
            self.log_disp.setText("Collecting Data and saving to file...")
            self.gather_button.setText("Stop")

            print("returning to main menu...")

            #self.update()





    def log(self, text):
        self.log_disp.setText(text)
        print(text)
        self.log_file("[LOG]: " + text)



    def log_file(self, text):
        if self.logging_file is not None:
            logging_string = str(datetime.now()) + ":  " + text + "\n"
            self.logging_file.write(logging_string)



    def send_note(self):
        text_to_send = self.th_log.text()
        self.th_log.clear()
        self.log_file(text_to_send)



# using this class you can create custom functionality for individual sensors that may be different. For example, this class
# represents the MotionSense device
class MotionSense_device_QWidget(QWidget):

    def __init__(self, number, address, name):
        super().__init__()
        print("trying to register " + str(name))
        # Create a layout for the sensor option checkboxes
        optionsLayout = QFormLayout()
        self.options = []
        self.characteristics = []
        # every device must have an address, and name
        self.number = number
        self.address = address

        if name == None or name == "":
            print("error registering device")
            return -1

        self.name = name




        #bold font
        #remove this from here and into the log

        # Add the checkboxes to the layout
        device_name = QLabel(name + " Device " + str(address))
        device_name.setFont(bold_font)
        optionsLayout.addWidget(device_name)
        # here is where we customize the attributes
        if name == "MotionSenseHRV3":
            # for every ble characteristic we want to collect from, we set up this 2 element array:
            # the first element of the Array contains a QCheckBox representing an enabled or disabled
            # state in the application, while the other element is a custom class.
            self.check1 = [QCheckBox("PPG collection")]
            self.check2 = [QCheckBox("Magnometer collection")]
            self.check3 = [QCheckBox("Accelorometer")]
            self.check4 = [QCheckBox("Tensorflow")]



            self.check1.append(bluetooth_reciver.MSenseCharacteristic("PPG", bluetooth_reciver.ppg_sensor_handle,
                                                                      "da39c923-1d81-48e2-9c68-d0ae4bbd351f"))

            self.check2.append(bluetooth_reciver.MSenseCharacteristic("Magnometer",
                                                                      bluetooth_reciver.notification_handler_magnometer,
                                                                      "da39c922-1d81-48e2-9c68-d0ae4bbd351f"))
            self.options.append(self.check1)
            self.options.append(self.check2)
            self.options.append(self.check3)
            self.options.append(self.check4)






        # quick example to illustrate custom devices
        elif name == "Custom Device":


            self.check1 = [QCheckBox("Characteristic 1 name")]
            self.check2 = [QCheckBox("Characteristic 2 name")]

            self.check1.append(bluetooth_reciver.MSenseCharacteristic("Characteristic 1", bluetooth_reciver.ppg_sensor_handle,
                                                                      "Characteristic1UUID"))


            #now, all we need to do is append the MSenseCharacteristic Class containing
            # the name (can be anything), reciver function, and characteristic UUID

            #self.check4.append(bluetooth_reciver.MSenseCharacteristic("l2"), bluetooth_reciver.notification_handler_magnometer,
            #                   "da39c922-1d81-48e2-9c68-d0ae4bbd351f")




        # now we will use the 'new' format
        elif name == "MotionSenseF2":
            self.check1 = [QCheckBox("accel and ppg")]
            self.check1.append(bluetooth_reciver.MSenseCharacteristic("accel and ppg", bluetooth_reciver.ppg_sensor_handle,
                                                                      "da39c923-1d81-48e2-9c68-d0ae4bbd351f"
                                                                      ))

            self.options.append(self.check1)


        for check_widget in self.options:
            optionsLayout.addWidget(check_widget[0])


        #optionsLayout.addWidget(self.check2[0])
        #optionsLayout.addWidget(self.check3[0])

        self.setLayout(optionsLayout)


    def get_characteristics(self):
        for options in self.options:
            if options[0].isChecked():
                self.characteristics.append(options[1])

        return self.characteristics





class Collection_Worker(QRunnable):



    def __init__(self, function_handler, address, path, record_length, options=None):
        print("worker starting")
        super(Collection_Worker, self).__init__()
        self.options = options
        self.address = address
        self.path = path
        self.record_length = record_length
        self.function_handler = function_handler





    @pyqtSlot()
    def run(self):

        self.function_handler(self.address, self.path, self.record_length, self.options)


def test_function(address, path, record_length, options, test_flag):
    print("I am running in a test call!")
    import data_collection.bluetooth_reciver
    print("imported")
    try:
        data_collection.bluetooth_reciver.non_async_collect(address, path, record_length, options, test_flag)

    except Exception as err:
        raise

    print("I am done!")



class Window(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MotionSense Bluetooth GUI")
        self.initUI()

    def initUI(self):
        self.scroll = QScrollArea()  # Scroll Area which contains the widgets, set as the centralWidget
        self.widget = MotionSenseApp()
        self.vbox = QVBoxLayout()  # The Vertical Box that contains the Horizontal Boxes of  labels and buttons

        for i in range(1, 50):
            object = QLabel("TextLabel: " + str(i))
            self.vbox.addWidget(object)

        self.widget.setLayout(self.vbox)

        # Scroll Area Properties
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.widget)

        self.setCentralWidget(self.scroll)

        self.setGeometry(600, 100, 1000, 900)
        self.setWindowTitle('Scroll Area Demonstration')
        self.show()

        return







def start():
    print("starting gui")
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    print("gui launched!")
    sys.exit(app.exec_())


if __name__ == "__main__":
    start()