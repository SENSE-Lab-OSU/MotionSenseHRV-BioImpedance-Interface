import os
import sys
import time
import atexit
import PyQt5.QtBluetooth
from PyQt5.QtCore import QRunnable, QThreadPool
from PyQt5.Qt import QLinearGradient
import PyQt5.QtCore
import multiprocessing
from PyQt5.QtCore import pyqtSlot


import datetime

print(str(datetime.datetime.now()))


'''This is the UI script for the OSU MotionSense Bluetooth
device. For more information on how to setup Custom devices, please see device_setup.py

This code is designed to be as dependency free as possible,
so that making new scripts for data collection can be easily be swapped out by
replacing singular function calls to collect_data'''

from data_collection import bluetooth_reciver

from data_collection import device_setup




import PyQt5
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFormLayout,
    QLineEdit,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QLabel,
    QScrollArea,
    QMainWindow

)
from PyQt5.QtGui import QPixmap, QFont

from PyQt5.QtCore import Qt
import PyQt5.QtWidgets

import PyQt5.Qt

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
        self.path = None
        self.update_modulus = 0
        #
        self.devices = []
        self.addresses = []

        self.timer = PyQt5.QtCore.QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.update_timer)

        self.timer.start()

        self.record_length = 180.0
        # initialize all of the components of the UI - The Checkboxs, Line Edits, Logo, Text, what have you not

        # Create an outer layout
        outerLayout = QVBoxLayout()

        # create a seperate layout just for logo
        picture_layout = QFormLayout()

        # Create a form layout for the label and line edit
        topLayout = QFormLayout()

        #try to add OSU logo image
        image = QLabel(self)
        pixmap = QPixmap('gui/newOSU2.png')
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
        self.file_line3.setText(str(1.0))
        topLayout.addRow("Save Location:", self.file_line)
        topLayout.addRow("Folder Name:", self.file_line2)
        topLayout.addRow("Stop Recording Data after: (min)", self.file_line3)

        self.enable_csv = QCheckBox()
        topLayout.addRow("Open LSL Connection during Streaming", self.enable_csv)


        # layout for connecting to motionsense UI, with a button to connect
        collections_layout = QFormLayout()
        self.button = PyQt5.QtWidgets.QPushButton("Connect to MotionSense")
        #button.setText("lol")
        self.gather_button = PyQt5.QtWidgets.QPushButton("Start data_collection")
        self.gather_button.setDisabled(True)

        logging_header = QLabel("Logging Options")
        logging_header.setFont(font)
        Option1 = QLoggingOptions(lambda: self.log("Pre-Bike Phase Started", True), "Log Pre-Bike Phase")
        Option2 = QLoggingOptions(lambda: self.log("Bike Started", True), "Log Bike Phase")
        Option3 = QLoggingOptions(lambda: self.log("Post Bike Phase Started", True), "Log Post-Bike Phase")

        Options = QHBoxLayout()
        collections_layout.addWidget(logging_header)
        Options.addWidget(Option1)
        Options.addWidget(Option2)
        Options.addWidget(Option3)
        collections_layout.addRow(Options)



        self.th_log = QLineEdit()
        self.log_button = PyQt5.QtWidgets.QPushButton("log custom message:")
        self.log_button.clicked.connect(self.send_note)
        collections_layout.addRow(self.log_button, self.th_log)
        collections_layout.addWidget(QLabel(" "))
        # progress bar
        self.progress_bar = PyQt5.QtWidgets.QProgressBar()
        self.progress_bar_label = QLabel("Collection Progress")
        data_label = QLabel("Data Collection:")
        data_label.setFont(font)
        collections_layout.addWidget(data_label)
        collections_layout.addRow(self.progress_bar_label, self.progress_bar)
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

        self.path = self.file_line.text() + "\\" + self.file_line2.text()
        print(self.path)

        # try and create the directory for storing data
        self.log("making directories..")
        try:
            os.mkdir(self.path)
        except FileExistsError:
            self.log("directory already exists, keeping old")
        except:
            self.log("Could not create the directory! " + str(self.path) + " is invalid.")
            return
        finally:
            self.log("moving on")
        self.log("sucessfully created directories!")
        self.logging_file = open(self.path + "\\" + "log.txt", "a")
        self.user_log_file = open(self.path + "\\" + "user_log.txt", "a")
        self.log("created and activated log file...")


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

        connect_addresses = bluetooth_reciver.non_async_connect(device_setup.all_sensors)
        if len(connect_addresses) != 0:
            self.log_disp.setText("collection will be saved to")
            self.button.setText("connected!")
            self.button.setDisabled(True)
            self.gather_button.setEnabled(True)
            self.addresses.extend(connect_addresses)

            # for every MotionSense ble device found add it to the list of devices
            for count, adress in enumerate(self.addresses):
                device = MotionSense_device_QWidget(count, adress)
                self.devices.append(device)
                self.optionsLayout.addWidget(device)
            self.path = self.file_line.text() + "\\" + self.file_line2.text()
            print(self.path)
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


    def update_timer(self):

        print("updating...")

        
        disconnected_devices = 0
        
        if self.threads is not None:
            for thread in self.threads:
                process = thread[0]
                if process.is_alive():
                    pass
                else:
                    disconnected_devices += 1
            debug_string = "Current Collecting Devices: " + str(len(self.threads)- disconnected_devices) + "\n"
            debug_string += "disconnected_devices = " + str(disconnected_devices)
            if self.currently_collecting:
                # this timer is updating every 2 seconds, so we multiply by 2 to get the true time
                progress_value = int(((self.update_modulus)/self.record_length)*100)
                
                if progress_value >= 100:
                    progress_value = 98
                    debug_string = "Collection Finished, Disconnecting Devices..."
                self.progress_bar.setValue(progress_value)
                self.progress_bar_label.setText("Collection Progress: " + str(datetime.timedelta(seconds=self.update_modulus)))
                if self.update_modulus % 2 == 0:
                    self.log(debug_string)
                self.update_modulus += 1
                
                
                if len(self.threads) != 0 and disconnected_devices == len(self.threads) and disconnected_devices > 0:

                    self.collect_data()
                    self.log("all devices have been disconnected, terminating collection")




    # this is device/implementation specific, and may require some modifications
    # for future versions to work
    def collect_data(self):
        self.log("collect button pressed")
        self.update_modulus = 0
        self.progress_bar.setValue(0)
        total_checks = 0
        if len(self.devices) == 0:
            return
        # we assume here that address is valid, because this button should be greyed out if there
        # is no connection

        if self.currently_collecting:
            # this means the button has been pressed before, and we need to stop it
            self.currently_collecting = False
            self.log("Stopped data collection")
            self.gather_button.setText("Saved!")
            self.log_disp.setText("Stopping Collection and saving to file...")
            self.log("trying to terminate existing bluetooth data collection...")



            print("trying to close app")
            self.refresh_log_file()
            for thread in self.threads:
                with thread[1].get_lock():
                    thread[1].value = -1
                thread[0].join()
            self.threads.clear()


            #print(datetime.datetime.now())
            self.gather_button.setText("Start")

            return
        else:
            # need to figure out how to handel async events here


            self.log("trying to start collection...")
            try:
                self.record_length = float(self.file_line3.text())*60
            except ValueError:
                self.record_length = 180.0
                self.log_disp.setText("record length input invalid, defaulting to 180.0")



            self.log("registering devices...")
            # for all MSense devices, get the characteristics that are checked and collect data from them

            for device in self.devices:
                self.log("registering device " + str(device.address))
                path = self.file_line.text() + "\\" + self.file_line2.text() + "\\" + device.name
                options = device.get_characteristics()
                if len(options) == 0:
                    # if the user didn't check any boxes we don't need to run any data
                    continue
                self.log("got options...")
                self.log("options for device: " + device.name)
                self.log(str(options))
                self.log("creating thread...") #data_collection.bluetooth_reciver.non_async_collect

                th_thread = Collection_Worker(bluetooth_reciver.non_async_collect, device.address,
                                              path, self.record_length, options)


                # a bit of a messy solution. For more compatibility options,
                # use PyQt threadpool
                self.log("creating child shared memory flag for end")
                exit_flag = multiprocessing.Value("i", 2)
                self.log("creating Process...")
                p = multiprocessing.Process(target=test_function, args=(device.address, path, self.record_length, options, exit_flag))
                self.log("attempting to start thread" + str(total_checks))
                p.start()

                total_checks += 1


                #threadpool.start(th_thread)
                #th_thread.run(self.addresses, path, record_length, options)
                self.log("adding thread to list")
                self.threads.append((p, exit_flag))
                #data_collection.bluetooth_reciver.non_async_collect(self.address, path, record_length, options)
            #start logging for files
            self.log("sucessfully registered all devices")
            if total_checks > 0:
                self.currently_collecting = True
                self.log("started data collection")
                # need to program this to get the data in the future
                # This is a class of bool values containing what options we would like
                self.log_disp.setText("Collecting Data and saving to file...")
                self.gather_button.setText("Stop")
                self.log("started for " + str(total_checks) + "devices")
            #self.log("returning to main menu...")

            #self.update()





    def log(self, text, user_message=False):
        self.log_disp.setText(text)
        print(text)
        if user_message:
            info_string = "[USER INFO]: "
            self.log_file_user(info_string + text)
        else:
            info_string = "[LOG]: "
        self.log_file(info_string + text)



    def log_file(self, text):
        if self.logging_file is not None and not self.logging_file.closed:
            logging_string = str(datetime.datetime.now()) + ":  " + text + "\n"
            self.logging_file.write(logging_string)
            
        else:
            print("error writing to log file")

    def log_file_user(self, text):
        if self.user_log_file is not None and not self.user_log_file.closed:
            logging_string = str(datetime.datetime.now()) + ":  " + text + "\n"
            self.user_log_file.write(logging_string)
        else:
            print("error writing to log file")


    def refresh_log_file(self):
        if self.logging_file is not None and self.path is not None:
            self.logging_file.close()
            self.logging_file = open(self.path + "\\" + "log.txt", "a")

    def send_note(self):
        text_to_send = self.th_log.text()
        self.th_log.clear()
        self.log(text_to_send, True)








class QLoggingOptions(PyQt5.QtWidgets.QPushButton):

    def __init__(self, function, text):
        super().__init__()
        self.clicked.connect(function)
        self.setText(text)






# using this class you can create custom functionality for individual sensors that may be different. For example, this class
# represents the MotionSense device
class MotionSense_device_QWidget(QWidget):

    def __init__(self, number, device:bluetooth_reciver.MSenseDevice):
        super().__init__()
        print("trying to register " + str(device.name))
        # Create a layout for the sensor option checkboxes
        optionsLayout = QFormLayout()
        self.options = []
        self.characteristics = []
        # every device must have an address, and name
        self.number = number
        self.address = device.address

        if device.name == None or device.name == "":
            print("error registering device")
            return -1

        self.name = device.name
        #bold font
        #remove this from here and into the log

        # Add the checkboxes to the layout
        device_name = QLabel(self.name + " Device " + str(self.address))
        device_name.setFont(bold_font)
        optionsLayout.addWidget(device_name)
        # here is where we customize the attributes

            # for every ble characteristic we want to collect from, we set up this 2 element array:
            # the first element of the Array contains a QCheckBox representing an enabled or disabled
            # state in the application, while the other element is a custom class.
        for characteristic in device.characteristics:
            current_checkbox = [QCheckBox(characteristic.name + " collection")]
            current_checkbox.append(characteristic)
            current_checkbox[0].setChecked(True)
            self.options.append(current_checkbox)


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
        print(err)

    print("I am done!")



class Window(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MotionSense Bluetooth GUI")
        self.initUI()
        p = self.palette()

        test_gradient = QLinearGradient()

        test_gradient.setStart(self.width() / 2, 0.0)
        test_gradient.setFinalStop(self.width() / 2, self.height())
        test_gradient.setColorAt(1.0, PyQt5.Qt.QColor(200, 200, 200,255))
        test_gradient.setColorAt(0.0, PyQt5.Qt.QColor(255, 150, 150, 255))

        p.setBrush(self.backgroundRole(), PyQt5.Qt.QBrush(test_gradient))
        self.setPalette(p)



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


    def closeEvent(self, a0) -> None:
        self.widget.log("closing app")
        try:
            self.widget.logging_file.close()
            self.widget.user_log_file.close()
        except Exception as e:
            print(e)



import bleak_winrt.windows.devices.bluetooth


def update(param):
    print("hello")


def start():
    #bleak_winrt.windows.devices.bluetooth.BluetoothLEPreferredConnectionParametersRequest
    print("starting gui")
    app = QApplication(sys.argv)
    window = Window()
    window.show()




    print("gui launched!")
    app.exec_()

    sys.exit()


if __name__ == "__main__":
    start()