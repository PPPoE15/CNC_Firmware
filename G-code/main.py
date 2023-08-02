import serial
import re
import time


buf = [[], [], []]
ppm = 2048/10  # pulse per millimeter
sleep = 0.01


def rnd(num):
    return int(num + (0.5 if num > 0 else -0.5))


print('Enter the g-code file full path \n')
path = input()
print('Enter COM-port number \n')
com_num = input()


with open(path) as gcode:
    for line in gcode:
        line = line.strip()
        command = re.findall(r'[MG].?\d+.?\d+', line)
        command = str(command).strip("'[]'")
        buf[0].append(command)

        x_coord = re.findall(r'X+(\d*\.\d+|\d+)?', line)
        x_coord = str(x_coord).strip("'[GMXY]'")
        if x_coord != '':
            x_coord = rnd(float(x_coord) * ppm)  # conv to float + conv from millimeters to num of pulses + rounded
        x_coord = str(x_coord)
        if len(x_coord) != 6:
            x_coord = '{:0>6}'.format(x_coord)
        x_coord = bytes(x_coord, 'ascii')

        buf[1].append(x_coord)

        y_coord = re.findall(r'Y+(\d*\.\d+|\d+)?', line)
        y_coord = str(y_coord).strip("'[GMXY]'")
        if y_coord != '':
            y_coord = rnd(float(y_coord) * ppm)  # conv to float + conv from millimeters to num of pulses + rounded
        y_coord = str(y_coord)
        if len(y_coord) != 6:
            y_coord = '{:0>6}'.format(y_coord)
        y_coord = bytes(y_coord, 'ascii')

        buf[2].append(y_coord)

        print(command + str(x_coord)  + str(y_coord))

print('parsing done')


ser = serial.Serial(com_num, 115200)  # make connection with stm32
print('connected')
time.sleep(sleep)

ser.write(b'1')  # send status request
time.sleep(sleep)
print('send status request')
if int(ser.read_until('1', 1)) == 1:  # wait for getting ready-message from STM
    for i in range(len(buf[0])):
            match buf[0][i]:
                case "G21":
                    print(str(i) + ' Режим работы в метрической системе')
                    ser.write(b'0123123123123')
                    print('done')
                    time.sleep(sleep)

                case "G90":
                    print(str(i) + ' Задание абсолютных координат')
                    ser.write(b'0000000000000')
                    time.sleep(sleep)

                case "G00":  # send 1
                    print(str(i) + ' Холостой ход')
                    ser.write(b'1' + buf[1][i] + buf[2][i])  # send command + x-coord + y-coord
                    time.sleep(sleep)

                case "M09":  # send 2
                    print(str(i) + ' Подача воздуха')

                    ser.write(b'2000000000000')

                    time.sleep(sleep)

                case "G01":  # send 3
                    print(str(i) + ' Рабочее перемещение')
                    ser.write(b'3' + buf[1][i] + buf[2][i])  # send command + x-coord + y-coord
                    time.sleep(sleep)

                case "M10":  # send 4
                    print(str(i) + ' Прекращение подачи воздуха')
                    ser.write(b'4000000000000')
                    time.sleep(sleep)

                case "M02":  # send 5
                    print(str(i) + ' Конец программы')
                    ser.write(b'5000000000000')
                    time.sleep(sleep)

                case _:
                    print(str(i) + ' Ошибка! Неизвестная команда')

print('Loading is done!')







time.sleep(5)