#pip install pyserial

import serial.tools.list_ports

ports = serial.tools.list_ports.comports() 
serialInst = serial.Serial() #instance 
portsLists = [] #formats port to ports list

for one in ports:
    portsLists.append(str(one)) #finds all ports
    print(f"Device: {one.device}, Description: {one.description}")
    print(str(one)) #displays all ports available

com = input("Select Com Port for Arduino #: ")

#search and set up for port
for i in range(len(portsLists)):
    if portsLists[i].startswith(str(com)):
        use = str(com)
        print(use)


serialInst.baudrate = 9600
serialInst.port = use
serialInst.open()

#commands for input
while True:
    command = input("Arduino Command (Blue/Green/Red/exit): ")
    serialInst.write(command.encode('utf-8'))

    if command == "exit":
        exit()

