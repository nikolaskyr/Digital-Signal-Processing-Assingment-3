# This is the application testing software for the ' Central Heating Boiler Smoke Indicator'
# Its purpose is to measure the density of the smoke coming out of the central heating boiler
# and provide a visual indication, in order to assist the maintenance technician to adjust the 
# air intake to the burner, and thus reduce the volume of smoke.

 
from pyfirmata2 import Arduino
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import signal as signal


# Class IIR2Filter computes the output of a 2nd order IIR filter. The filter coefficients
# are past as parameters during the instantiation of the filter as the constructors parameters.
# The actual filtering operation is carried out by the method 'filter2(x)' that has as input
# a single sample of the input signal and returns the filtered value.
class IIR2Filter:
    def __init__(self,_b0,_b1,_b2,_a1,_a2):   # Class constructor
        self.a1 = _a1      # set filter coefficients
        self.a2 = _a2      # set filter coefficients
        self.b0 = _b0      # set filter coefficients
        self.b1 = _b1      # set filter coefficients
        self.b2 = _b2      # set filter coefficients
        self.buffer1 = 0   # Step delay buffer 1
        self.buffer2 = 0   # Step delay buffer 2
    def filter2(self,x):   # 2nd order IIR filter implementation
        input_acc = x  - self.buffer1*self.a1 - self.buffer2*self.a2
        output_acc = input_acc*self.b0 + self.buffer1*self.b1 + self.buffer2*self.b2
        self.buffer2 = self.buffer1   # Forward buffers for next step delay
        self.buffer1 = input_acc      # Forward buffers for next step delay
        return output_acc   # Return filtered value      

# Class IIRFilter creates a chain of 2nd order IIR filters specified in class IIR2Filter.
# For each 2nd order filter it updates the filter coefficients and calls the 'filter2' method 
# to compute the filtered output.
class IIRFilter:
    def __init__(self,_coeffs):
        self.coeffs = _coeffs
        self.in_buff = np.zeros(100)
        self.out_buff = np.zeros(100)
        self.f=IIR2Filter(b0,b1,b2,a1,a2)
                
    def filter(self,x1):
        self.in_buff[0]=x1  # Assign new input data to the input buffer of the first 2nd order filter. 

# Set the b,a coefficients and filter the input sample.
        for i_order in range(int(len(self.coeffs)/5)):
            IIR2Filter.a1 = self.coeffs[i_order*5+3]
            IIR2Filter.a2 = self.coeffs[i_order*5+4]
            IIR2Filter.b0 = self.coeffs[i_order*5+0]
            IIR2Filter.b1 = self.coeffs[i_order*5+1]
            IIR2Filter.b2 = self.coeffs[i_order*5+2]
            self.out_buff[i_order] = self.f.filter2(self.in_buff[i_order])
            
# Forward the output buffer of each 2nd order filter to the input buffer of the next one.
        for i_order in range(int(len(self.coeffs)/5)):
            self.in_buff[i_order+1] = self.out_buff[i_order]
        
        return self.out_buff[i_order]

# RealtimePlotwindow is a realtime oscilloscope that displays the signal received 
# from an analogue input of the Arduino board.         
# This class is taken from the 'analog_realtime_scope.py' program
# Creates a scrolling data display

class RealtimePlotWindow:

    def __init__(self):
        # create a plot window
        self.fig, self.ax = plt.subplots()
        # that's our plotbuffer
        self.plotbuffer = np.zeros(500)
        # create an empty line
        self.line, = self.ax.plot(self.plotbuffer)
        # axis
        self.ax.set_ylim(0, 1)
        # That's our ringbuffer which accumluates the samples
        # It's emptied every time when the plot window below
        # does a repaint
        self.ringbuffer = []
        # start the animation
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100)

    # updates the plot
    def update(self, data):
        # add new data to the buffer
        self.plotbuffer = np.append(self.plotbuffer, self.ringbuffer)
        # only keep the 500 newest ones and discard the old ones
        self.plotbuffer = self.plotbuffer[-500:]
        self.ringbuffer = []
        # set the new 500 points of channel 9
        self.line.set_ydata(self.plotbuffer)
        return self.line,

    # appends data to the ringbuffer
    def addData(self, v):
        self.ringbuffer.append(v)


# Specify the sos filter coefficients and instantiate the IIR filter.
a1 = -1.81861152 # This part of the code is inserted here to avoid error messages 
                 # concerning the a,b coefficient definition 
a2 = 0.82929564
b0 = 0.08930701
b1 = -0.17342385
b2 = 0.08930701
coeffs = [0.08930701,-0.17342385,0.08930701,-1.81861152, 0.82929564,
         -1.93685575,0.9418292,1,-1.9921146,1,
        -1.98279602,0.98605365,1,-1.99577041,1]
f=IIRFilter(coeffs)


# Create an instance of an animated scrolling window
# To plot more channels just create more instances and add callback handlers below
realtimePlotWindow = RealtimePlotWindow()
#realtimePlotWindow2 = RealtimePlotWindow()

# sampling rate: 100Hz
samplingRate = 100
gain = 4
in_array = np.zeros(1000)
filtered_array = np.zeros(1000)
ar_index = 0
# our callback where we filter the data
def callBack(data):
    y = f.filter(data)
    y=gain*y
    realtimePlotWindow.addData(y)
    if y>=0.70: # Turn on/off the green and red leds according to the signal received.
        LED_green.write(y/1.5)
        LED_red.write(0)
    else:
        LED_green.write(0)
        LED_red.write((1-y)/1.5)
        
# Get the Ardunio board
board = Arduino('\\.\COM3')

# Define digital pins D5 and D9 as analog (PWM) outputs
LED_green = board.get_pin('d:5:p')
LED_red = board.get_pin('d:9:p')

# Set the sampling rate in the Arduino
board.samplingOn(1000 / samplingRate)

# Register the callback which adds the data to the animated plot
board.analog[0].register_callback(callBack)

# Enable the callback
board.analog[0].enable_reporting()

# show the plot and start the animation
plt.show()

print("finished")
