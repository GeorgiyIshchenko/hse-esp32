# Для этого кода нужно подрубить main.c с фукнцией 
# adxl345_test_console_log (только она должна быть включена)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import serial

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x_vals, y_vals, z_vals = [0], [0], [0]
scatter = ax.scatter([], [], [], c='b')
lim_val = 2

ax.set_xlim([-lim_val, lim_val])
ax.set_ylim([-lim_val, lim_val])
ax.set_zlim([-lim_val, lim_val])
ax.set_xlabel('X')
# its ok, axis hack
ax.set_ylabel('Z')
ax.set_zlabel('Y')
ax.set_title('XYZ ONLINE')
ax.view_init(elev=30, azim=45)
ax.plot(x_vals, z_vals, y_vals, c='b')

ser = serial.Serial(
    port='/dev/tty.usbserial-0001',
    baudrate=115200,  
    timeout=1
)

try:
    i = 0
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            # print(f"Received: {line}")
            if line.startswith('>'):
                axis = line[1]
                value = float(line.split(':')[1].split('|')[0])
                if axis == 'x':
                    x_vals.append(value)
                elif axis == 'y':
                    y_vals.append(value)
                elif axis == 'z':
                    z_vals.append(value)
                else:
                    raise Exception(f"Parser: unknown axis {axis}")
                
                i += 1

                if i % 3 == 0 and i >= 3:
                    if (len(x_vals)) > 50:
                        x_vals.pop(0)
                        y_vals.pop(0)
                        z_vals.pop(0)
                    print(f"x: {x_vals[-1]}, y: {y_vals[-1]}, z: {z_vals[-1]}")
                    if i % 9 == 0: # optimization, cla is heavy op
                        ax.cla()
                        ax.set_xlim([-lim_val, lim_val])
                        ax.set_ylim([-lim_val, lim_val])
                        ax.set_zlim([-lim_val, lim_val])
                        ax.set_xlabel('X')
                        ax.set_ylabel('Z')
                        ax.set_zlabel('Y')
                        ax.set_title('XYZ ONLINE')
                        ax.view_init(elev=30, azim=45)
                        ax.plot(x_vals, z_vals, y_vals, c='b')
                        plt.draw()
                        plt.pause(0.05)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()
