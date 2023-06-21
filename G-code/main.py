import serial
import re
import time


buf = [[], [], []]
ppm = 2048/10  # pulse per millimeter
sleep = 0.1


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

        x_coord = re.findall(r'X.?\d+.?\d+', line)
        x_coord = str(x_coord).strip("'[GMXY]'")
        if x_coord != '':
            x_coord = rnd(float(x_coord) * ppm)  # conv to float + conv from millimeters to num of pulses + rounded
        x_coord = str(x_coord)
        if len(x_coord) != 6:
            x_coord = bytes('{:0>6}'.format(x_coord), 'ascii')

        buf[1].append(x_coord)

        y_coord = re.findall(r'Y.?\d+.?\d+', line)
        y_coord = str(y_coord).strip("'[GMXY]'")
        if y_coord != '':
            y_coord = rnd(float(y_coord) * ppm)  # conv to float + conv from millimeters to num of pulses + rounded
        y_coord = str(y_coord)
        if len(y_coord) != 6:
            y_coord = bytes('{:0>6}'.format(y_coord), 'ascii')
        buf[2].append(y_coord)

        # print(command)
        # print(x_coord)
        # print(y_coord)



print('parsing done')


ser = serial.Serial(com_num, 115200)  # make connection with stm32
print('connected')
time.sleep(sleep)

ser.write(b'1')  # send status request
time.sleep(sleep)
print('send status request')

for i in range(len(buf[0])):

    if int(ser.read_until('1', 1)) == 1:  # wait for getting ready-message from STM
        match buf[0][i]:
            case "G21":
                print('Режим работы в метрической системе')
                ser.write(b'0')
                time.sleep(sleep)

            case "G90":
                print('Задание абсолютных координат')
                ser.write(b'0')
                time.sleep(sleep)

            case "G00":  # send 1
                print('Холостой ход')

                ser.write(b'1')
                time.sleep(sleep)
                if int(ser.read_until('2', 1)) == 2:
                    ser.write(buf[1][i])  # send x-coord
                    time.sleep(sleep)

                if int(ser.read_until('3', 1)) == 3:
                    ser.write(buf[2][i])  # send y-coord
                    time.sleep(sleep)

            case "M09":  # send 2
                print('Подача воздуха')
                ser.write(b'2')
                time.sleep(sleep)

            case "G01":  # send 3
                print('Рабочее перемещение')
                ser.write(b'3')
                time.sleep(sleep)
                if int(ser.read_until('2', 1)) == 2:
                    ser.write(buf[1][i])  # send x-coord
                    time.sleep(sleep)

                if int(ser.read_until('3', 1)) == 3:
                    ser.write(buf[2][i])  # send y-coord
                    time.sleep(sleep)

            case "M10":  # send 4
                print('Прекращение подачи воздуха')
                ser.write(b'4')
                time.sleep(sleep)

            case "M02":  # send 5
                print('Конец программы')
                ser.write(b'5')
                time.sleep(sleep)

            case _:
                print('Ошибка! Неизвестная команда')

        print(int(i))
        # print(buf[2][i])
        # print(buf[2][i])
