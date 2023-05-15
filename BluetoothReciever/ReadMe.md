Using the Bluetooth script


Requirments to use this bluetooth script

If you would like to run the script without a virutal enviornment, you must have a python 3.7 interpreter with QT, numpy, and bleak installed (see requirements.txt for more information)
For the exact versions of these libraries that have been tested, see requirements.txt


To run this application, follow these steps below.


1. Clone the repository, or download the zip file (as located in the top right corner of Github) and extract to an apropriate location.
2.  Go into the BluetoothReciever folder, and double click the run.bat file. You can also run this file with command prompt
3. The MotionSense Application should launch.


# Manual Setup (For Windows, Linux and Mac)
1. Clone this repository
2. Navigate to this folder, and create a virtual environment
3. Install the python dependencies (can be done by doing pip install -r requirements.txt) - USE THE requirements.txt VERSION FIRST, if it does not work then you can switch to requirements_stable.txt
4. You can then launch this application by running 'python MotionSense.py' in the terminal.



If you have a new sensor with bluetooth low energy and would like to configure it to collect using this application, you can add it by setting up an instance of MotionSense_device_QWidget,
(see gui/gui.py line 301 for more information.)


in gui.py
# Add the checkboxes to the layout
        device_name = QLabel(name + " Device " + str(address))
        device_name.setFont(bold_font)
        optionsLayout.addWidget(device_name)
        # here is where we customize the attributes
        if name == "MotionSense":
            ...


To create a new device, simply add the name here and then specify what you would like to collect.





