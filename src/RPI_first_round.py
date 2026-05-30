#datoteke
import rplidar
import serial
import time

#konfiguracija
PORT_LIDAR = "/dev/ttyUSB0"
PORT_SERIAL = "/dev/serial0"
BAUDRATE = 9600

lidar = rplidar.RPLidar(PORT_LIDAR, 256000)
ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=1)
time.sleep(2)

#bazne konstante za lidar i kameru
kp = 0.05  # smanji jer su distance velike
kd = 0.0015
offset = 0
last_error = 0

button=False
while button==False:
    line = ser.readline().decode('utf-8').strip()
    if line=="a":
        button==True
#komande za komunikaciju izmedju rpi i esp i varijabla bool skreno koja se mijenja naspram primljene info sa esp
def send_command(angle):
    angle = int(max(-90, min(90, angle)))
    frame = f"{angle}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
def turning(lorr):
    frame = f"{lorr}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
skreno=True
print("START")

def get_clean_lidar():
    print("pre-cleaning serial port")
    raw_ser = serial.Serial(PORT_LIDAR, baudrate=256000, timeout=1)
    raw_ser.setDTR(False)
    time.sleep(0.1)
    raw_ser.reset_input_buffer()
    raw_ser.reset_output_buffer()
    raw_ser.close()
    obj = rplidar.RPLidar(PORT_LIDAR)
    try:
        print("Sending hardware reset command")
        obj.reset()
        time.sleep(2)
        obj.get_health()
        return(obj)
    except rplidar.RPLidarException:
        print("Hardware reset failed, trying one more time")
        obj.disconnect()
        return rplidar.RPLidar(PORT_LIDAR)
lidar=get_clean_lidar()

def safe_scans(lidar):
    while True:
        try:
            for scan in lidar.iter_scans():
                yield scan
        except rplidar.RPLidarException as e:
            print(f"Lidar greska: {e}, ciscenje i restart")
            turning('t')
            try:
                lidar.stop()
                time.sleep(0.5)
                lidar.clean_input()
                time.sleep(0.5)
            except:
                pass


#logika za skretanja tj. stranu
#47cm je osrednja udaljenost lidara od zida unutar kocke
#105cm je otp maksimalna udaljenost lidara od zida unutar kocke
#240cm je maks duzina kada izgubi kocku
# 180 lidar pravo, 90 lijevo, 270 desno

#osnovne varijable koje se koriste u petljama za lidar
turnl=False
turnr=False
ispravljen = False
count_turns=0
#varijable za lijevi i desni cone lidara
minl1 = 345
maxl1=360
minl2 =0
maxl2= 15
minr=165
maxr=195
#kod prije nego sto znamo odredjenu stranu na koju je robot okrenut
lidar.stop()
time.sleep(1)
lidar.clean_input()
time.sleep(0.5)
#uzimanje informacija sa lidara
for scan in safe_scans(lidar):
    #vrijednosti koje se moraju nulirati nakon svakog skeniranja lidara za kd
    suml = 0
    countl = 1
    sumr = 0
    countr = 1
    #prolazenje kroz vrijednosti samoga skeniranja lidara
    for (_, angle, distance) in scan:
        #sracunavanje vrijednosti sa lijeve strane
        if ((angle > minl1 and angle < maxl1)or(angle > minl2 and angle < maxl2)):
            suml += distance
            countl+=1
        #sracunavanje vrijednosti sa desne strane
        if angle > minr and angle < maxr:
            sumr += distance
            countr+=1
    #proracun ugla kojim se treba ispravljati da se drzi neke ravni najvise
    avgr = sumr / countr
    avgl = suml / countl
    error = avgr - avgl 
    res = kp * error + kd * (error - last_error) # saljes rezultat kolko se treba ispraviti
    last_error = error
    send_command(res)
    #gledanje da li ima stranu tj. da li mora skrenuti
    if avgl >= 1000 :
        lidar.stop()
        turnl = True
        turning('l')
        count_turns+=1
        while skreno==True :
            line = ser.readline().decode('utf-8').strip()
            print(line)
            if line == "h":
                skreno = False
                lidar.clean_input()
        time.sleep(0.2)
        break
    if avgr >= 1000 :
        lidar.stop()
        turnr = True # saljes informaciju na koju stranu je turn
        turning('r')
        count_turns+=1
        while skreno==True:
            line = ser.readline().decode('utf-8').strip()
            print(line)
            if line == "h":
                skreno = False
                lidar.clean_input()
        time.sleep(0.2)
        break
#ako je odredjeno lijevo skretanje
if turnl == True:
    #ponavlja se 11 puta za 11 skretanja koja mora obaviti za 3 puna kruga
        #resotovanje informacije sa esp i citanje informacija sa lidara
    while count_turns<12:
        skreno=True
        for scan in safe_scans(lidar):
            #resetovanje varijabli za svaki krug
            suml = 0
            countl = 1
            sumr = 0
            countr = 1
            #prolazenje kroz informacije sa lidara
            for (_, angle, distance) in scan:
                if  ((angle > minl1 and angle < maxl1)or(angle > minl2 and angle < maxl2)):
                    suml += distance
                    countl+=1
                if angle > minr and angle < maxr:
                    sumr += distance
                    countr+=1
            #u slucaju da je skretanje zanemaruje proracune te salje informaciju za skretanje
            avgl = suml / countl
            avgr = sumr / countr
            if avgl >= 1000:
                lidar.stop()
                turning("l")
                count_turns+=1
                while skreno==True:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
                    if line == "h":
                        skreno = False
                        lidar.clean_input()
                skreno=True
                avgl=0
                time.sleep(0.2)
                break
            else :
                error = avgr - avgl - offset #+x ako hoces x mm offset udesno, - za ulijevo
                res = kp * error + kd * (error - last_error)
                last_error = error
                send_command(res)
    
#ako je odredjeno desno skretanje
else :
    while count_turns<12:
        skreno=True
        for scan in safe_scans(lidar):
            suml = 0
            countl = 1
            sumr = 0
            countr = 1
            for (_, angle, distance) in scan:
                if  ((angle > minl1 and angle < maxl1)or(angle > minl2 and angle < maxl2)):
                    suml += distance
                    countl+=1
                if angle > minr and angle < maxr:
                    sumr += distance
                    countr+=1
            avgl = suml / countl
            avgr = sumr / countr
            if avgr >= 1000:
                lidar.stop()
                turning("r")
                count_turns+=1
                while skreno==True:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
                    if line == "h":
                        skreno = False
                        lidar.clean_input()
                skreno=True
                avgr=0
                time.sleep(0.2)
                break
            else :
                error = avgr - avgl + offset #+x ako hoces x mm offset udesno, - za ulijevo
                res = kp * error + kd * (error - last_error)
                last_error = error
                send_command(res)
#nakon 3 runde salje informaciju robotu da stane u tom kvadrantu
turning("e")       
lidar.disconnect()
