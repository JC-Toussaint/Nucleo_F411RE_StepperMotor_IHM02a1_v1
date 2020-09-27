import numpy as np
import serial
import serial.tools.list_ports as port_list
from skimage import io
import matplotlib.pyplot as plt

ports = list(port_list.comports())
STM32_port=[]
for p in ports:
    print(p)
    port_str=str(p)
    if 'STM32' in port_str:
        STM32_port=port_str.split()
    
print(STM32_port[0])    
s = serial.Serial(STM32_port[0], baudrate=115200)

print('*** RESET STM32 BOARD ***')

while True:
    line = s.readline()
    if line[0] != 35: 
        rgb_str=line.split()
        rgb=np.uint8(rgb_str)
        rgb_str='RGB {:03d} {:03d} {:03d}'.format(rgb[0], rgb[1], rgb[2])

        rgb=rgb.reshape(1, 1, 3)
        plt.imshow(rgb)
        plt.text(0, 0, rgb_str, bbox=dict(facecolor='white', alpha=1.0))
        plt.draw()
        plt.pause(1)
    else:
        print(line)



