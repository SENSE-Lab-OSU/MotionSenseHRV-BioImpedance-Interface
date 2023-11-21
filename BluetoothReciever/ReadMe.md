Using the Bluetooth script



Requirements for use

Before using, a python interpreter > 3.7 must be installed. The recommended version that has been verified to work as per 
requirements.txt is python 3.10.5

If you would like to run the script without a python virtual environment or requirements.txt (manual install),  QT, numpy, and bleak installed (see requirements.txt for more information)
For the exact versions of these libraries that have been tested, see requirements.txt

To run this application, follow these steps below.


1. Clone the repository, or download the zip file (as located in the top right corner of Github) and extract to an appropriate location.
2.  Go into the BluetoothReceiver folder, and double click the run.bat file. You can also run this file with command prompt
3. The MotionSense Application should launch.



# Manual Setup (For Windows, Linux and Mac)
1. Clone this repository
2. Navigate to this folder, and create a virtual environment
3. Install the python dependencies (can be done by doing pip install -r requirements.txt) - USE THE requirements.txt VERSION FIRST, if it does not work then you can switch to requirements_stable.txt
4. You can then launch this application by running 'python MotionSense.py' in the terminal.

# Connecting Devices

Once the application starts, you should see a window pop up that contains the main interface for collecting data.
To start collection, first make sure all desired sensors are powered up. Then click the 'Connect to MotionSense' button.
All sensors that are named "MotionSense" will be connected. Note that the Application can support up to 4 devices (More can also be added, but this is not recommended due to potential performance issues)

# Configuring External Devices for Collection

The bluetooth application is designed to be easily configurable, such that any new ble device that supports notify characteristics can be added with a minimum of 3 lines of code. If you have a new sensor with bluetooth low energy and would like to configure it to collect using this application, you can add it by setting up an instance of MSense Device,
(see data_collection/device_setup.py line for more information.)






