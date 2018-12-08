# -*- coding: utf-8 -*-
"""
Created on Thu Jun 28 01:06:26 2018

@author: neutron

Description: Battery Charging Auto Cutoff script
"""

import psutil
import serial
import time
import Tkinter as tk
import tkMessageBox
import ConfigParser

config = ConfigParser.RawConfigParser()
config.read('laptop_dock_controller.cfg')

SERIAL_PORT = 'COM8'
BAUDRATE = 57600
SERIAL_PORT_READ_TIMEOUT_MS  = 5000
DEV_INIT_SUCCESS_TEXT = "Initialization Complete\r\n"

CHARGER_CONNECT_RETRY_COUNT = 3

POWER_ON_CMD = '1'
POWER_OFF_CMD = '0'
POWER_ON_SUCCESS_TEXT = "POWER_ON"
POWER_OFF_SUCCESS_TEXT = "POWER_OFF"

INIT_TEXT = 'LAPTOP CHARGER CONTROLLER -- INITIALIZED\r\nSUPPLY COMES FROM NC OF RELAY, DEFAULT IS POWER ON\r\nSEND>\r\n1: POWER ON\r\n0: POWER OFF\r\nSEND>\r\nA: ENABLE_CURRENT_STREAM\r\nB: DISABLE_CURRENT_STREAM\r\nInitialization Complete\r\n'
INIT_TEXT_LENGTH = len(INIT_TEXT)

STREAMING_ON_CMD = 'A'
STREAMING_OFF_CMD = 'B'
STREAMING_ON_SUCCESS_TEXT = "STREAMING_ON"
STREAMING_OFF_SUCCESS_TEXT = "STREAMING_OFF"

#the below mentioned settings are used for battery re-calibration, when we want to recal battery we need to discharge to very lower value
#and then recharge to 100% but in this case we need to allow sometime for the batt firmware to recal itself in charging condition.
BATTERY_CHARGING_UPPER_CUTOFF_MAX = 99
BATTERY_CHARGING_LOWER_CUTOFF_MIN = 30
BATTERY_CHARGING_UPPER_CUTOFF_MAX_ALLOW_CAL_TIMEOUT_S = 300


BATTERY_CHARGING_UPPER_CUTOFF = 99 #was 90
BATTERY_CHARGING_LOWER_CUTOFF = 50 #was 60 Changed because my battery wear level has recently increase, now total capacity reduced to 37Wh from 39Wh

POLL_DELAY_SECS = 5

MY_DOCK_MONITOR_ID = 'DELA07A'
#Powershell command to get basic details of connected monitors is "Get-CimInstance -Namespace root\wmi -ClassName WmiMonitorBasicDisplayParams" or "Get-CimInstance -Namespace root\wmi -ClassName WmiMonitorID"
import wmi
def getMonitorCount():#for details regarding using this command please refer to document "Wmi Monitor Info access" in Onenote
    monitorsID_wmi = wmi.WMI(moniker="//./root/wmi:WmiMonitorID") #create a wmi object with namespace root\wmi and pointing to classname WmiMonitorID
    return len(monitorsID_wmi.query())  #returns the list of WmiMonitorID objects per Connected monitor, so if two monitors are connected it will return a list of two objects

def getMonitorName(monitorId):
    """
    getMonitorName(<monitorId>) starts from 0
    will return a string something like "DEL071A"
    if the specified is not present it will return an empty string.
    """
    monitorsID_wmi = wmi.WMI(moniker="//./root/wmi:WmiMonitorID")
    monitorsID_wmi_list = monitorsID_wmi.query()
    numberOfMonitors = len(monitorsID_wmi_list)
    if monitorId > numberOfMonitors-1 or monitorId < 0:#checking if specified monitorId is greater than number of connected monitors or if it is invalid number like -1
        return ""
    monitorIdString = monitorsID_wmi_list[monitorId].InstanceName
    return monitorIdString.split('\\')[1]

def isMonitorActive(monitorId):
    """
    isMonitorActive(<monitorId>) starts from 0
    will return a string something like True or False depending on the actual state
    """
    monitorsID_wmi = wmi.WMI(moniker="//./root/wmi:WmiMonitorID")
    monitorsID_wmi_list = monitorsID_wmi.query()
    numberOfMonitors = len(monitorsID_wmi_list)
    if monitorId > numberOfMonitors-1 or monitorId < 0:#checking if specified monitorId is greater than number of connected monitors or if it is invalid number like -1
        return False
    return monitorsID_wmi_list[monitorId].Active


def isPrimaryDockMonitorConnected():
    for monitors in range(0,getMonitorCount()):
        if getMonitorName(monitors) == MY_DOCK_MONITOR_ID:
            return True
    return False

def openSerial(serialPort=SERIAL_PORT):
    return serial.Serial(port=serialPort,baudrate=BAUDRATE,timeout=SERIAL_PORT_READ_TIMEOUT_MS)


def charger_power_state_retry_wrapper(serialPort,powerStateOn=False,retryCount=0):
    status = charger_power_state(serialPort,powerStateOn)
    while status==0 and retryCount!=0:
        status = charger_power_state(serialPort,powerStateOn)
        retryCount = retryCount-1
    
    return status

def charger_power_state(serialPort,powerStateOn=False):
    try:
        sp = serial.Serial(port=serialPort,baudrate=BAUDRATE,timeout=SERIAL_PORT_READ_TIMEOUT_MS)
        
        timeStamp = time.time()
        while time.time()-timeStamp < SERIAL_PORT_READ_TIMEOUT_MS/1000 and sp.inWaiting() < INIT_TEXT_LENGTH:
            pass
        
        buffered = sp.inWaiting()

        if buffered < INIT_TEXT_LENGTH:
            print ('Failed to reset Arduino!')
            sp.close()
            return 0
        
        data = sp.read(buffered)
        if data.find(DEV_INIT_SUCCESS_TEXT) == -1:
            print ('Could not find initialization text!, Please check the arduino fw.')
            sp.close()
            return 0
        
        token = ''
        if powerStateOn is True:
            sp.write(POWER_ON_CMD)
            token=POWER_ON_SUCCESS_TEXT
        else:
            sp.write(POWER_OFF_CMD)
            token=POWER_OFF_SUCCESS_TEXT
        
        timeStamp = time.time()
        while (time.time()-timeStamp < SERIAL_PORT_READ_TIMEOUT_MS/1000 and sp.inWaiting<len("POWER")):
            pass
        
        time.sleep(0.5)
        data = sp.read(sp.inWaiting())
        if data.find(token) == -1:
            print ('Incorrect Response!')
            sp.close()
            return 0
        
        print ('Command Successful.')
        sp.close()
        return 1
    
    except serial.SerialException as spException:
        print ('Unable to open the port or read the response, find more details below:')
        #print (str(spException.message))
        try:
            if sp.closed is False:
                sp.close()
            return 0
        except UnboundLocalError:#if port was not opened at all the sp is still not defined in such a case there will be an unboundlocalerror when trying to reference 'sp'
            return 0
    except Exception as e:
        print 'Unknown exception'
        print str(e)
        return 0

def getData(sp):#sp is serial port handle/object
    sp.write('A')
    while (sp.inWaiting()==0):
        pass
    return sp.readline()
        
def getBatteryPercentage():
    return psutil.sensors_battery().percent

def getChargerPluggedState():
    return psutil.sensors_battery().power_plugged


CHARGER_SET_TO_OFF_FLAG = False

#Tkinter must have a root window. If you don't create one, one will be created for you. If you don't want this root window, create it and then hide it:
#more details in onenote "TkInter message box"
def displayErrorMsg(title,msg):
    root = tk.Tk()
    root.withdraw()
    return tkMessageBox.showinfo(title, msg)

if __name__ == "__main__":
    try:
	#when the laptop comes out of sleep generally bluetooth connection is not established in first try,
	#in such case an exception can occur and program might end, we need to handle such situations
    #also sometimes I have observed that the Plug of the device itself is not properly connected to
    #mains supply, in such cases the bluetooth in device is unpowered, so we need to popup a message 
    #saying it tried to open bluetooth connection but device is not responding.
    #and when we try to enable the charging and command is successful but device is not on ac power 
    #we have to popup a message saying please check if you have plugged the charger in laptop
    #and when we disable the charger successfully but still it is charging, popup a message saying 
    #please check whether the charger is connected through the device or directly to mains.
        while True:
            try:
                if isPrimaryDockMonitorConnected() is True:#run the charge-discharge cycle only when Monitor is connected i.e. when it is docked in my room.
                    if getBatteryPercentage() > BATTERY_CHARGING_UPPER_CUTOFF and getChargerPluggedState() is True:
                        #if calibration setting has been specified then wait for CAL TIMEOUT PERIOD before disconnecting charger.
                        if BATTERY_CHARGING_UPPER_CUTOFF == BATTERY_CHARGING_UPPER_CUTOFF_MAX:
                            time.sleep(BATTERY_CHARGING_UPPER_CUTOFF_MAX_ALLOW_CAL_TIMEOUT_S)
                        
                        #now retry will be done 3 more times before reporting error.
                        if charger_power_state_retry_wrapper(SERIAL_PORT,False,CHARGER_CONNECT_RETRY_COUNT) == 0:
                            displayErrorMsg("LCP", "Unable to open the Bluetooth Serial Port")                
                    
                    if getBatteryPercentage() < BATTERY_CHARGING_LOWER_CUTOFF and getChargerPluggedState() is False:
                        if charger_power_state_retry_wrapper(SERIAL_PORT,True,CHARGER_CONNECT_RETRY_COUNT) == 0:
                            displayErrorMsg("LCP", "Unable to open the Bluetooth Serial Port")
                time.sleep(POLL_DELAY_SECS)
            
            except:
                print("There has been an exception")
    except KeyboardInterrupt:
        print('Program stopped, CTRL^C pressed')
    
