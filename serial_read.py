import serial
import datetime
from matplotlib import pyplot as plt
from drawnow import drawnow

# Inicializa el serial
aams = serial.Serial('COM4',
                     baudrate=115200,
                     bytesize=8,
                     parity='N',
                     stopbits=1,
                     timeout=1.5
                     )

aams.close()
aams.open()
aams.flush()

time = []
temp1 = []
temp2 = []
pres1 = []
pres2 = []
humi1 = []
humi2 = []


while True:
    line = aams.readline()
    print(line)
    # if (line[:4].decode("ascii") == str(datetime.date.today().year)):
    #     data = line[:-2].decode("ascii").split(sep=",")
    #     time.append(data[0])
    #     if data[1] == '1':
    #         temp1.append(data[2])
    #         pres1.append(data[3])
    #         humi1.append(data[4])
        
    #     if data[1] == '2':
    #         temp2.append(data[2])
    #         pres2.append(data[3])
    #         humi2.append(data[4])

    #drawnow(in_figure)