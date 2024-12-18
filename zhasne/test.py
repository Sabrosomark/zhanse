import time
import Adafruit_ADS1x15
import smbus
import RPi.GPIO as GPIO

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

try:
    print("Starting TDS and Distance Measurements...")
    while True:
        # Read and display TDS sensor data
        raw_adc = adc.read_adc(TDS_SENSOR_CHANNEL, gain=1)
        voltage = raw_adc * (4.096 / 32767.0)  # Calculate voltage based on ADS1115 gain
        tds = read_tds(voltage)  # Convert voltage to TDS
        print(f"TDS: {tds:.2f} ppm")

        # Wait briefly before switching to distance measurement
        time.sleep(1)

        # Measure and display distance data
        dist = measure_distance()
        print(f"Distance: {dist} cm")

        # Wait briefly before looping again
        time.sleep(1)

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
    GPIO.cleanup()  # Cleanup GPIO pins
