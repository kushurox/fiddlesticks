import serial.tools.list_ports


found = False

port_name = None


while not found:
    for port, desc, _ in serial.tools.list_ports.comports():
        if desc.lower() == "fiddlesticks":
            port_name = port
            found = True
            break


print(f"Found Fiddlesticks on port {port_name}")
with serial.Serial(port_name, 115200, timeout=1) as ser:
    ser.write(b'CALIBRATE\n')
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(line)