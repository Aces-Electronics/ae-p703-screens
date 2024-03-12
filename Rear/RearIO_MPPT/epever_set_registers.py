import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.WARNING)

import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

client = ModbusClient(method='rtu', port='/dev/ttyUSB2', timeout=1, stopbits=1, bytesize=8, parity='N', baudrate=115200)
client.connect()

rr = client.read_holding_registers(address=0x9000, count=1, unit=1)
print(rr.registers)

rr = client.write_registers(address=0x9000, values=[
    0x0,  # (0x9000 = 0000H - User defined)
    0x22, # (0x9001 = 34AH - Battery Capacity)
    0x12c, # (0x9002 = 3.00mV/°C/2V - Temperature compensationcoefficient)
    1620, # (0x9003 = 16.20V - High Volt.disconnect)
    0x5dc, # (0x9004 = 15.00V - Charging limit voltage)
    0x5dc, # (0x9005 = 15.00V - Over voltage reconnect)
    0x5b4, # (0x9006 = 14.60V - Equalization voltage)
    0x5a0, # (0x9007 = 14.40V - Boost voltage)
    0x564, # (0x9008 = 13.80V - Float voltage)
    1630, # (0x9009 = 16.30V - Boost reconnect voltage)
    0x4ec, # (0x900A = 12.60V - Low voltage reconnect)
    0x4c4, # (0x900B = 12.20V - Under voltage recover)
    0x4b0, # (0x900C = 12.00V - Under voltage warning)
    0x456, # (0x900D = 11.10V - Low voltage disconnect)
    0x424, # (0x900E = 10.60V - Discharging limit voltage)
], unit=1)
print(rr)

rr = client.read_holding_registers(address=0x9009, count=1, unit=1)

if hasattr(rr, 'registers'):
    print(rr.registers)
else:
    print(rr)