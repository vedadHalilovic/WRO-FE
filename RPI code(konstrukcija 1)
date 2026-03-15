import rplidar
import serial
import time

PORT_LIDAR = "/dev/ttyUSB0"
PORT_SERIAL = "/dev/serial0"
BAUDRATE = 9600

lidar = rplidar.RPLidar(PORT_LIDAR)
ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=1)
time.sleep(2)

kp = 0.5# smanji jer su distance velike
kd = 0.005
offset =0
last_error = 0

def send_command(angle):
    angle = int(max(-180, min(180, angle)))
    frame = f"{angle}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
def turning(lorr):
    frame = f"{lorr}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
print("START")

#logika za skretanja tj. stranu
#47cm je osrednja udaljenost lidara od zida unutar kocke
#105cm je otp maksimalna udaljenost lidara od zida unutar kocke
#240cm je maks duzina kada izgubi kocku

#kd konstante

# 180 lidar pravo, 90 lijevo, 270 desno
#prije odredjenog turna kretanje

while turnl==False and turnr==False:
    for scan in lidar.iter_scans():
        suml = 0
        countl = 0
        sumr = 0
        countr = 0
        for (_, angle, distance) in scan:
            if angle > 255 and angle < 285: #od 240 do 270 - ismar
                suml += distance
                countl+=1
                
            if angle > 75 and angle < 105: #od 90 do 120 - ismar
                sumr += distance
                countr+=1
        avgr = sumr / countr
        avgl = suml / countl
        error = avgr - avgl + offset #+x ako hoces x mm offset udesno, - za ulijevo
        res = kp * error + kd * (last_Error - error) # saljes rezultat kolko se treba ispraviti
        last_Error = error
        send_command(res)
    if avgl >= 140 :
        turnl == True
        turning('l')
    if avgr >= 140 :
        turnr == True # saljes informaciju na koju stranu je turn
        turning('r')

#ako je odredjeno skretanje lijevo      
if turnl == True:
    while count_turns<11:
        for scan in lidar.iter_scans():
            suml = 0
            countl = 0
            sumr = 0
            countr = 0
            for (_, angle, distance) in scan:
                if angle > 255 and angle < 285: #line 46 - ismar
                    suml += distance
                    countl+=1
                if angle > 75 and angle < 105: #line 50 - ismar
                    sumr += distance
                    countr+=1
            avgl = suml / countl
            avgr = sumr / countr
            if avgl >= 140:
                turning("l")
            else :
                error = avgr - avgl + offset #+x ako hoces x mm offset udesno, - za ulijevo
                res = kp * error + kd * (last_Error - error)
                last_Error = error
                send_command(res)

#ako je odredjeno desno
else :
    while count_turns<11:
        for scan in lidar.iter_scans():
            suml = 0
            countl = 0
            sumr = 0
            countr = 0
            for (_, angle, distance) in scan:
                if angle > 255 and angle < 285: #line 46 - ismar
                    suml += distance
                    countl+=1
                if angle > 75 and angle < 105: #line 50 - ismar
                    sumr += distance
                    countr+=1
            avgl = suml / countl
            avgr = sumr / countr
            if avgr >= 140:
                turning("r")
            else :
                error = avgr - avgl + offset #+x ako hoces x mm offset udesno, - za ulijevo
                res = kp * error + kd * (last_Error - error)
                last_Error = error
                send_command(res)
turning("e")       
lidar.disconnect()
