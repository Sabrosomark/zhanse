import time
import Adafruit_ADS1x15
import smbus

# Initialize the I2C bus (usually bus 1 for most Raspberry Pi models)
i2c_bus_number = 1
i2c = smbus.SMBus(i2c_bus_number)

# Create an instance of the ADS1115 ADC
adc = Adafruit_ADS1x15.ADS1115(busnum=i2c_bus_number)

# Pin for the TDS sensor
TDS_SENSOR_CHANNEL = 1  # Channel A1

# Function to convert voltage to TDS value (adjust for your sensor calibration)
def read_tds(voltage):
    tds_value = voltage * 1000  # Example formula, customize based on calibration
    return tds_value

try:
    while True:
        # Read raw ADC value and convert to voltage
        raw_adc = adc.read_adc(TDS_SENSOR_CHANNEL, gain=1)
        voltage = raw_adc * (4.096 / 32767.0)  # Calculate voltage based on ADS1115 gain
        tds = read_tds(voltage)  # Convert voltage to TDS

        # Display the TDS value
        print(f"TDS: {tds:.2f} ppm")

        time.sleep(1)  # Delay between readings

except KeyboardInterrupt:
    print("\nProgram interrupted by user")
