#lib
import json
import sys
import time
import random
import serial
from iotticket.models import device
from iotticket.models import criteria
from iotticket.models import deviceattribute
from iotticket.models import datanodesvalue
from iotticket.client import Client

port = serial.Serial('COM26', baudrate = 9600, timeout=2, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE);
global received;
port.close();

#main
# data = json.load(open("config.json"))
# username = data["username"]
# password = data["password"]
# deviceId = data["deviceId"]
# baseurl = data["baseurl"]

# c = Client(baseurl, username, password)

# Sensor register mapping
REGISTERS = {
    "dht22_temp": 0x01,
    "dht22_hum": 0x02,
    "lmt84lp": 0x03,
    "nsl19m51": 0x04,
    "hih4000": 0x05,
    "grove_voc": 0x06,
    "grove_eco2": 0x07,
    "ac_voltage": 0x08,
    "energy_kwh": 0x09
}

def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return [crc & 0xFF, crc >> 8]

def modbus_request(register):
    # Modbus request: [slave_addr, func_code, reg_addr_high, reg_addr_low, num_regs_high, num_regs_low, crc_low, crc_high]
    request = [0x01, 0x04, 0x00, register, 0x00, 0x01]
    crc = calculate_crc(request)
    request.extend(crc)
    
    print(f"Request: {request}")
    port.open()
    port.write(bytes(request))
    response = port.read(7) # Expect 7 bytes: [addr, func, byte_count, data_high, data_low, crc_low, crc_high]
    port.close()
    
    if len(response) == 7 and response[0] == 0x01 and response[1] == 0x04:
        value = (response[3] << 8) | response[4]
        print(f"{time.strftime('%Y%m%d:%H%M%S')}: Register {register} = {value}")
        return value
    else:
        print(f"{time.strftime('%Y%m%d:%H%M%S')}: Error or empty response: {response}")
        return -9999

def send_data_to_iot_ticket(data):
    # Load IoT-Ticket credentials
    with open("config.json") as f:
        config = json.load(f)
    
    c = Client(config["baseurl"], config["username"], config["password"])
    listofvalues = []
    
    for key, value in data.items():
        nv = datanodesvalue()
        if key == "dht22_temp" or key == "lmt84lp":
            nv.set_name("Temperature")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("C")
        elif key == "dht22_hum" or key == "hih4000":
            nv.set_name("Humidity")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("%")
        elif key == "nsl19m51":
            nv.set_name("Light")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("lux")
        elif key == "grove_voc":
            nv.set_name("VOC")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("ppb")
        elif key == "grove_eco2":
            nv.set_name("CO2")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("ppm")
        elif key == "ac_voltage":
            nv.set_name("Voltage")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("V")
        elif key == "energy_kwh":
            nv.set_name("Energy")
            nv.set_path(key)
            nv.set_dataType("double")
            nv.set_unit("kWh")
        nv.set_value(value)
        listofvalues.append(nv)
    
    print(c.writedata(config["deviceId"], *listofvalues))

while True:
    sensor_data = {}
    for name, reg in REGISTERS.items():
        value = modbus_request(reg)
        if value != -9999:
            if name in ["dht22_temp", "dht22_hum", "lmt84lp"]: # Scale as needed
                value /= 10.0
            sensor_data[name] = value
        else:
            sensor_data[name] = -9999
    
    print(f"Sensor Data: {sensor_data}")
    # send_data_to_iot_ticket(sensor_data)
    time.sleep(5) # Poll every 5 seconds