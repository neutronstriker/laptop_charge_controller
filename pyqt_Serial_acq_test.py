# Import libraries
from numpy import *
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import serial
import time

SERIAL_PORT = 'COM8'
BAUDRATE = 57600
SERIAL_PORT_READ_TIMEOUT_MS  = 5000

def openSerial(serialPort=SERIAL_PORT):
    return serial.Serial(port=serialPort,baudrate=BAUDRATE,timeout=SERIAL_PORT_READ_TIMEOUT_MS)

def getData(sp):#sp is serial port handle/object
    sp.write('A')
    while (sp.inWaiting()==0):
        pass
    return sp.readline()

INIT_TEXT = 'LAPTOP CHARGER CONTROLLER -- INITIALIZED\r\nSUPPLY COMES FROM NC OF RELAY, DEFAULT IS POWER ON\r\nSEND>\r\n1: POWER ON\r\n0: POWER OFF\r\nSEND>\r\nA: ENABLE_CURRENT_STREAM\r\nB: DISABLE_CURRENT_STREAM\r\nInitialization Complete\r\n'
INIT_TEXT_LENGTH = len(INIT_TEXT)

# Create object serial port
portName = SERIAL_PORT                    # replace this port name by yours!
baudrate = BAUDRATE

sp = openSerial()
timeStamp = time.time()
while time.time()-timeStamp < SERIAL_PORT_READ_TIMEOUT_MS/1000 and sp.inWaiting() < INIT_TEXT_LENGTH:
    pass

sp.read(sp.inWaiting())#flush buffer


### START QtApp #####
app = QtGui.QApplication([])            # you MUST do this once (initialize things)
####################

win = pg.GraphicsWindow(title="Signal from serial port") # creates a window
p = win.addPlot(title="Realtime plot")  # creates empty space for the plot in the window
curve = p.plot()                        # create an empty "plot" (a curve to plot)

windowWidth = 500                       # width of the window displaying the curve
Xm = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr = -windowWidth                      # set first x position

# Realtime data plot. Each time this function is called, the data display is updated
def update():
    global curve, ptr, Xm    
    Xm[:-1] = Xm[1:]                      # shift data in the temporal mean 1 sample left
    value = getData(sp).split('\t')[1]                # read line (single value) from the serial port
    Xm[-1] = float(value)                 # vector containing the instantaneous values      
    ptr += 1                              # update x position for displaying the curve
    curve.setData(Xm)                     # set the curve with this data
    curve.setPos(ptr,0)                   # set x position in the graph to 0
    QtGui.QApplication.processEvents()    # you MUST process the plot now

### MAIN PROGRAM #####    
# this is a brutal infinite loop calling your realtime data plot
while True: update()

### END QtApp ####
pg.QtGui.QApplication.exec_() # you MUST put this at the end
##################