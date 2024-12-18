import time
import Adafruit_ADS1x15
import smbus
import RPi.GPIO as GPIO
import numpy as np
from flask import Flask, jsonify, render_template
from scipy.signal import cheby1, lfilter

# Initialize the I2C bus (usually bus 1 for most Raspberry Pi models)
i2c_bus_number = 1
i2c = smbus.SMBus(i2c_bus_number)

# Create an instance of the ADS1115 ADC
adc = Adafruit_ADS1x15.ADS1115(busnum=i2c_bus_number)

# Pin for the TDS sensor
TDS_SENSOR_CHANNEL = 1  # Channel A1

# GPIO Pin Definitions for the ultrasonic sensor
TRIG_PIN = 23  # Trigger Pin
ECHO_PIN = 24  # Echo Pin

# Setup GPIO Mode
GPIO.setmode(GPIO.BCM)  # Broadcom pin numbering
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Flask application setup
app = Flask(__name__)

# Parameters for filtering
BUFFER_SIZE = 5
buffer_distance = []
tds_buffer = []

# Design Chebyshev filter for distance smoothing
cheby_order = 4
cheby_ripple = 0.5
cheby_cutoff = 0.1  # Normalized cutoff frequency
cheby_b, cheby_a = cheby1(cheby_order, cheby_ripple, cheby_cutoff, btype='low', analog=False)

# Function to apply Chebyshev filter to distance data
def apply_chebyshev_filter(data, b, a):
    return lfilter(b, a, data)[-1]  # Return the most recent filtered value

# Function to apply least squares method for TDS smoothing
def least_squares_filter(buffer, new_value):
    buffer.append(new_value)
    if len(buffer) > BUFFER_SIZE:
        buffer.pop(0)
    if len(buffer) < 2:
        return np.mean(buffer)
    x = np.arange(len(buffer))
    A = np.vstack([x, np.ones(len(buffer))]).T
    m, c = np.linalg.lstsq(A, buffer, rcond=None)[0]
    return m * (len(buffer) - 1) + c

# Function to convert voltage to TDS value (adjust for your sensor calibration)
def read_tds(voltage):
    tds_value = voltage * 1000  # Example formula, customize based on calibration
    return tds_value

# Function to measure distance using the ultrasonic sensor
def measure_distance():
    # Send Trigger Pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 microsecond pulse
    GPIO.output(TRIG_PIN, False)

    # Wait for Echo to Start
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    # Wait for Echo to End
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    # Calculate Pulse Duration
    pulse_duration = pulse_end - pulse_start

    # Calculate Distance (Speed of Sound = 34300 cm/s)
    distance = pulse_duration * 17150
    distance = round(distance, 2)  # Round to 2 decimal places

    return distance

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_data():
    # Measure distance and apply Chebyshev filter
    distance = measure_distance()
    buffer_distance.append(distance)
    if len(buffer_distance) > BUFFER_SIZE:
        buffer_distance.pop(0)
    filtered_distance = apply_chebyshev_filter(buffer_distance, cheby_b, cheby_a)

    # Read and filter TDS data
    raw_adc = adc.read_adc(TDS_SENSOR_CHANNEL, gain=1)
    voltage = raw_adc * (4.096 / 32767.0)
    tds_value = read_tds(voltage)
    filtered_tds = least_squares_filter(tds_buffer, tds_value)

    # Prepare JSON response
    data = {
        "distance": {
            "raw": distance,
            "filtered": round(filtered_distance, 2)
        },
        "tds": {
            "raw": tds_value,
            "filtered": round(filtered_tds, 2)
        }
    }
    return jsonify(data)

if __name__ == '__main__':
    try:
        app.run(debug=True, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        GPIO.cleanup()
