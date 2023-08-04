import re
import time
import PySimpleGUI as sg
import serial

buf = [[], [], []]
ppm = 2048/10  # pulse per millimeter
sleep = 0.01
text_info = 'Инструкция:\n' \
            '0. Включаем все оборудование и подключаем контроллер к ПК\n' \
            '1. Выбираем файл с g-кодом на ПК (поддерживаемые расширения .txt и .gcode)\n' \
            '2. Задаем скорость перемещения\n' \
            '3. Выбираем COM-порт к которому подключен контроллер\n' \
            '4. Нажимаем кнопку загрузить\n' \
            '5. Нажимаем кнопку просмотреть g-code, программа отобразится на экране\n' \
            '6. Нажимаем кнопку подключения, ПК откроет COM-порт и подключится к контроллеру ЧПУ станка\n' \
            '7. Нажимаем кнопку старт, ПК загрузит программу в память контроллера и станок ее начнет выполнять\n' \
            '8. Дожидаемся окончания выполнения программы, получаем сообщение "Программа выполнена", закрываем программу\n' \
            'Поддержка:\n экстренно: art.viktorov12@gmail.com\n по рабочим вопросам и предложениям: viktorov@gksnab.ru\n\n' \
            '                 © ГК Снабжение   2023                    '

def rnd(num):
    return int(num + (0.5 if num > 0 else -0.5))


def serial_ports():
    ports = ['COM%s' % (i + 1) for i in range(256)]
    com_port_list = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            com_port_list.append(port)
        except (OSError, serial.SerialException):
            pass
    return com_port_list


com_port_list = serial_ports()

layout = [
    [sg.Text('Выберете файл, содержащий g-code:'), sg.Button('Справка')],
    [sg.Text('Путь:'), sg.InputText(key='-PATH-'), sg.FileBrowse('Выбрать')],
    [sg.Text('Задайте скорость перемещения в мм/с:')],
    [sg.InputText(key='-SPEED-')],
    [sg.Text('Выберете номер COM-порта:')],  # [sg.InputText(key='-COM-')],
    [sg.Combo(com_port_list, default_value='COM4', key='-COM-', size=(15, 1))],
    [sg.Submit('Загрузить параметры')],
    [sg.Text(' ')],
    [sg.Button('Просмотреть G-code'), sg.Button('Подключение') , sg.Button('Старт') , sg.Cancel('Выход')],
    [sg.Output(size=(100, 20))]
]


window = sg.Window('ЧПУ станок для нанесения герметика', layout)

while True:                             # The Event Loop
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Выход':
        break
    #print(event, values) #debug
    path = values['-PATH-']
    speed = values['-SPEED-']
    com_num = values['-COM-']

    if event == 'Справка':
        sg.popup(text_info)
    if event == 'Загрузить параметры':
        print('Загружено')
        print('Скорость = ' + speed + ' мм/c')
        print('COM-порт: ' + com_num)

    if event == 'Просмотреть G-code':
        print('G-code:\n')
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

                #print(command + str(x_coord) + str(y_coord))
                print(line)

        print('Обработка завершена\n')

    if event == 'Подключение':
        ser = serial.Serial(com_num, 115200)  # make connection with stm32
        time.sleep(sleep)
        if ser.is_open:
            print('Подключено\n')

    if event == 'Старт':
        ser.write(b'1')  # send status request
        time.sleep(sleep)
        print('send status request')
        if int(ser.read_until('1', 1)) == 1:  # wait for getting ready-message from STM
            for i in range(len(buf[0])):
                match buf[0][i]:
                    case "G21":
                        #print(str(i) + ' Режим работы в метрической системе')
                        ser.write(b'0123123123123')
                        #print('done')
                        time.sleep(sleep)

                    case "G90":
                        #print(str(i) + ' Задание абсолютных координат')
                        ser.write(b'0000000000000')
                        time.sleep(sleep)

                    case "G00":  # send 1
                        #print(str(i) + ' Холостой ход')
                        ser.write(b'1' + buf[1][i] + buf[2][i])  # send command + x-coord + y-coord
                        time.sleep(sleep)

                    case "M09":  # send 2
                        #print(str(i) + ' Подача воздуха')

                        ser.write(b'2000000000000')

                        time.sleep(sleep)

                    case "G01":  # send 3
                        #print(str(i) + ' Рабочее перемещение')
                        ser.write(b'3' + buf[1][i] + buf[2][i])  # send command + x-coord + y-coord
                        time.sleep(sleep)

                    case "M10":  # send 4
                        #print(str(i) + ' Прекращение подачи воздуха')
                        ser.write(b'4000000000000')
                        time.sleep(sleep)

                    case "M02":  # send 5
                        #print(str(i) + ' Конец программы')
                        ser.write(b'5000000000000')
                        time.sleep(sleep)

                    case _:
                        print(str(i) + ' Ошибка! Неизвестная команда')

        print('\nПрограмма завершена\n')
        #window.update()

        time.sleep(5)
        program = ser.read_until('4', 1)  # wait for getting ready-message from STM
        if program == '4':
            print('\nПрограмма завершена\n')

window.close()
