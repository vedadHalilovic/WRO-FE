import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import rplidar
#konfiguracija
PORT_LIDAR = "/dev/ttyUSB0"
PORT_SERIAL = "/dev/serial0"
BAUDRATE = 9600   

lidar = rplidar.RPLidar(PORT_LIDAR, baudrate=256000)
ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=1)
time.sleep(2)

#bazne konstante za lidar i kameru
kp = 0.02  # smanji jer su distance velike
kd = 0.0005
offsetmin = 170
offsetmax = 270
last_error = 0

button=False
while button==False:
    line = ser.readline().decode('utf-8').strip()
    if line=="a":
        button==True
#komande za komunikaciju izmedju rpi i esp i varijabla bool skreno koja se mijenja naspram primljene info sa esp
gyro=False
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
skreno=True
#varijable za lijevi i desni cone lidara
minl1 = 345
maxl1=360
minl2 =0
maxl2= 15
minr=165
maxr=195
minc=75
maxc=105
prva_kocka, druga_kocka, br_kocki, distanca =0
# ─────────────────────────────────────────────
# CONFIGURATION — adjust these to your setup
# ─────────────────────────────────────────────

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
mid_x = FRAME_WIDTH // 2

picam2 = Picamera2()            # hardware, global
picam2.configure(picam2.create_preview_configuration(
main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
))
# Real-world block height in centimeters (used for distance estimation)
BLOCK_REAL_HEIGHT_CM = 15.0

# Camera focal length in pixels — calibrate with: f = (pixel_height * distance) / real_height
# Example: if a 5 cm block appears 100px tall at 50 cm → f = (100 * 50) / 5 = 1000
FOCAL_LENGTH_PX = 1000.0

# Fixed distance offset between front and back block (cm) when one is behind the other
STACKED_BLOCK_OFFSET_CM = 90.0

# Minimum contour area to be considered a block (filters out noise)
MIN_BLOCK_AREA = 800

# How "rectangular" a contour must be (0–1). Higher = stricter rectangle check.
RECT_SOLIDITY_THRESHOLD = 0.80

# HSV colour ranges
# Red wraps around in HSV, so two ranges are needed
RED_LOWER_1  = np.array([0,   120, 70]) #sa 120 stavi na 150 radi roze
RED_UPPER_1  = np.array([10,  255, 255])
RED_LOWER_2  = np.array([170, 120, 70]) #sa 120 stavi na 150 radi roze
RED_UPPER_2  = np.array([180, 255, 255])

GREEN_LOWER  = np.array([35,  60,  60])
GREEN_UPPER  = np.array([85,  255, 255])

blocks = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
blocknumber=0
blocksdone=0
temp =[[0,0,0],[0,0,0]]
# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

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
            try:
                lidar.stop()
                time.sleep(0.5)
                lidar.clean_input()
                time.sleep(0.5)
            except:
                pass

            
def send_command(angle):
    angle = int(max(-90, min(90, angle)))
    frame = f"{angle}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
    
def turning(lorr):
    frame = f"{lorr}\n"
    ser.write(frame.encode())
    print("Sent:", frame.strip())
print("START")

def estimate_distance(pixel_height: float) -> float:
    """Return distance in cm using the pinhole camera model."""
    if pixel_height <= 0:
        return float('inf')
    return (BLOCK_REAL_HEIGHT_CM * FOCAL_LENGTH_PX) / pixel_height


def is_rectangular(contour, threshold: float = RECT_SOLIDITY_THRESHOLD) -> bool:
    """Return True if contour is solid enough to be a rectangle."""
    area = cv2.contourArea(contour)
    if area < MIN_BLOCK_AREA:
        return False
    x, y, w, h = cv2.boundingRect(contour)
    rect_area = w * h
    if rect_area == 0:
        return False
    solidity = area / rect_area
    return solidity >= threshold


def find_blocks(mask, x_offset: int = 0):
    """
    Find rectangular blobs in a binary mask.

    Returns a list of dicts:
        { 'bbox': (x, y, w, h),   # coordinates in FULL frame
          'center': (cx, cy),
          'distance_cm': float }
    sorted by distance (nearest first).
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blocks = []

    for cnt in contours:
        if not is_rectangular(cnt):
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        dist = estimate_distance(h)
        blocks.append({
            'bbox': (x + x_offset, y, w, h),   # shift back to full-frame coords
            'center': (x + x_offset + w // 2, y + h // 2),
            'distance_cm': dist,
        })

    # Sort nearest-first; limit to 2 per side
    blocks.sort(key=lambda b: b['distance_cm'])
    return blocks[:2]


def classify_stacking(blocks):
    """
    If two blocks are detected in a segment, determine whether they are
    side-by-side (unlikely given the setup) or front-to-back (stacked).
    Returns a label string for display.
    """
    if len(blocks) < 2:
        return None
    d1, d2 = blocks[0]['distance_cm'], blocks[1]['distance_cm']
    gap = abs(d2 - d1)
    # If the distance gap is close to the known fixed offset, flag as stacked
    tolerance = STACKED_BLOCK_OFFSET_CM * 0.35   # ±35 %
    if abs(gap - STACKED_BLOCK_OFFSET_CM) <= tolerance:
        return f"STACKED (gap ≈ {gap:.1f} cm)"
    return f"SEPARATE (gap = {gap:.1f} cm)"


def build_colour_mask(hsv_frame):
    """Return separate red and green binary masks."""
    red_mask = (
        cv2.inRange(hsv_frame, RED_LOWER_1, RED_UPPER_1) |
        cv2.inRange(hsv_frame, RED_LOWER_2, RED_UPPER_2)
    )
    green_mask = cv2.inRange(hsv_frame, GREEN_LOWER, GREEN_UPPER)

    # Morphological clean-up
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    red_mask   = cv2.morphologyEx(red_mask,   cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    return red_mask, green_mask


def draw_blocks(frame, blocks, colour_bgr, colour_label: str, side_label: str):
    """Draw bounding boxes and distance labels onto the frame."""
    for i, blk in enumerate(blocks):
        x, y, w, h = blk['bbox']
        cv2.rectangle(frame, (x, y), (x + w, y + h), colour_bgr, 2)
        label = f"{side_label} {colour_label}#{i+1}: {blk['distance_cm']:.1f} cm"
        cv2.putText(frame, label, (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour_bgr, 1, cv2.LINE_AA)


def draw_ui(frame, left_info, right_info):
    """Draw dividing line, segment labels, and stacking info."""
    mid_x = FRAME_WIDTH // 2

    # Centre divider
    cv2.line(frame, (mid_x, 0), (mid_x, FRAME_HEIGHT), (200, 200, 200), 1)

    # Segment headings
    cv2.putText(frame, "LEFT",  (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2, cv2.LINE_AA)
    cv2.putText(frame, "RIGHT", (mid_x + 10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2, cv2.LINE_AA)

    # Stacking info per side
    for side_label, info, x_start in [("L", left_info, 10), ("R", right_info, mid_x + 10)]:
        y_pos = FRAME_HEIGHT - 40
        for colour, blocks in info.items():
            stacking = classify_stacking(blocks)
            if stacking:
                text = f"{side_label}-{colour}: {stacking}"
                cv2.putText(frame, text, (x_start, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
                y_pos -= 16


def print_summary(left_info, right_info):
    """Print a structured console summary each frame."""
    print("\n── Frame ──────────────────────────────────")
    for side_label, info in [("LEFT ", left_info), ("RIGHT", right_info)]:
        for colour, blocks in info.items():
            if not blocks:
                continue
            distances = [f"{b['distance_cm']:.1f} cm" for b in blocks]
            stacking  = classify_stacking(blocks) or "single"
            print(f"  {side_label} | {colour:5s} | {stacking:30s} | distances: {distances}")

            
def scan_frame():
     """
    Captures one frame and returns exactly 2 block entries.
    Format: [colour, side, distance_cm]
        colour:   1 = green, 2 = red,  0 = empty
        side:     1 = left,  2 = right, 0 = empty
        distance: float cm,             0 = empty
    Always returns exactly 2 entries, zeros fill empty spots.
    """
    frame_rgb = picam2.capture_array()
    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    # ── Split into segments ──────────────────
    left_frame  = frame[:, :mid_x]
    right_frame = frame[:, mid_x:]

    # ── Build colour masks per segment ───────
    left_hsv  = cv2.cvtColor(left_frame,  cv2.COLOR_BGR2HSV)
    right_hsv = cv2.cvtColor(right_frame, cv2.COLOR_BGR2HSV)

    left_red_mask,  left_green_mask  = build_colour_mask(left_hsv)
    right_red_mask, right_green_mask = build_colour_mask(right_hsv)

    # ── Detect blocks ────────────────────────
    left_info = {
        "RED":   find_blocks(left_red_mask,  x_offset=0),
        "GREEN": find_blocks(left_green_mask, x_offset=0),
    }
    right_info = {
        "RED":   find_blocks(right_red_mask,  x_offset=mid_x),
        "GREEN": find_blocks(right_green_mask, x_offset=mid_x),
    }

    # ── Draw results ─────────────────────────
    draw_blocks(frame, left_info["RED"],    (0, 0, 255), "RED",   "L")
    draw_blocks(frame, left_info["GREEN"],  (0, 200, 0), "GREEN", "L")
    draw_blocks(frame, right_info["RED"],   (0, 0, 255), "RED",   "R")
    draw_blocks(frame, right_info["GREEN"], (0, 200, 0), "GREEN", "R")

    draw_ui(frame, left_info, right_info)
    print_summary(left_info, right_info)

    cv2.imshow("Block Detector", frame)
    cv2.waitKey(1)

    # ── Convert to simple lists ──────────────
    detected = [[0, 0, 0], [0, 0, 0]]
    index = 0

    colour_map = {"GREEN": 1, "RED": 2}

    for colour_label, blocks in left_info.items():
        for blk in blocks:
            if index >= 2:
                break
            detected[index] = [colour_map[colour_label], 1, round(blk['distance_cm'], 1)]
            index += 1

    for colour_label, blocks in right_info.items():
        for blk in blocks:
            if index >= 2:
                break
            detected[index] = [colour_map[colour_label], 2, round(blk['distance_cm'], 1)]
            index += 1

    return detected

#color side distance

def sacuvaj(detected):
    for block in detected:
        blocks[blocknumber][0]=block[0]
        blocks[blocknumber][1]=block[1]
        if round(block[2], 1)<49:
            blocks[blocknumber][2]=1
        else if round(block[2], 1)>49 and round(block[2], 1)<97:
            blocks[blocknumber][2]=2
        else:
            blocks[blocknumber][2]=3
        blocknumber+=1

def obilazenje(offsetmin, offsetmax):
    br_kocki=0 # ako je jadna kockica
    prva_kocka=0#+ ili - offsetmin ili offsetmax
    distanca =0 # distanca ako je samo jedna kockica
    druga_kocka=0#+ ili - offsetmin ili offsetmax
    tren=0
    if blocksdone-2>0:
        if blocks[blocksdone-2][0]!=0 and blocks[blocksdone-1][0]==0 :
            br_kocki+=1
            if blocks[blocksdone][0]=1:
                if blocks[blocksdone][1]=1:     
                    druga_kocka=-offsetmax#error = avgr - avgl - offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
                else druga_kocka=-offsetmin#error = avgr - avgl - offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
            else:
                if blocks[blocksdone][1]=1:
                     druga_kocka=offsetmin#error = avgr - avgl + offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
                else druga_kocka=offsetmax#error = avgr - avgl + offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
    if blocknumber>=blocksdone and blocknumber!=0:
        if blocks[blocksdone][0]!=0:
            if blocks[blocksdone][0]=1:
                if blocks[blocksdone][1]=1:     
                    prva_kocka=-offsetmax #error = avgr - avgl - offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
                else prva_kocka=-offsetmin#error = avgr - avgl - offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
            else:
                if blocks[blocksdone][1]=1:
                    prva_kocka=offsetmin#error = avgr - avgl + offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
                else prva_kocka=offsetmax#error = avgr - avgl + offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
            distanca=blocks[blocksdone][2]
        blocksdone+=1
        if blocks[blocksdone-1][0]!=0 and blocks[blocksdone][0]!=0:
            br_kocki+=1
            if blocks[blocksdone][0]=1:
                if blocks[blocksdone][1]=1:     
                    druga_kocka=-offsetmax#error = avgr - avgl - offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
                else druga_kocka=-offsetmin#error = avgr - avgl - offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
            else:
                if blocks[blocksdone][1]=1:
                     druga_kocka=offsetmin#error = avgr - avgl + offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
                else druga_kocka=offsetmax#error = avgr - avgl + offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
        else if blocks[blocksdone][0]!=0:
            if blocks[blocksdone][0]=1:
                if blocks[blocksdone][1]=1:     
                    prva_kocka=-offsetmax#error = avgr - avgl - offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
                else prva_kocka=-offsetmin#error = avgr - avgl - offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
            else:
                if blocks[blocksdone][1]=1:
                    prva_kocka=offsetmin#error = avgr - avgl + offsetmin #+x ako hoces x mm offset udesno, - za ulijevo
                else prva_kocka=offsetmax#error = avgr - avgl + offsetmax #+x ako hoces x mm offset udesno, - za ulijevo
            distanca=blocks[blocksdone][2]
    if blocks[blocksdone][2]<blocks[blocksdone-1][2]:
        tren=prva_kocka
        prva_kocka=druga_kocka
        druga_kocka=tren
    if count_turns!=0:
        blocknumber=count_turns*2
    blocksdone+=1
    if blocksdone==8 blocksdone=0
    return prva_kocka, druga_kocka, br_kocki, distanca
#skeniraj jednom sliku i sa koje su strane kockice tu je i skretanje
lidar.stop()
picam2.start()
time.sleep(1.0) #0.5 ako je lighting konstantan, a 1 ako se malo mijenja
lidar.clean_input()
time.sleep(0.2)
temp =scan_frame()
sacuvaj(temp)
time.sleep(0.1)
picam2.stop()
if blocks[blocknumber-1][1] =1 or blocks[blocknumber-2][1] =1:
    turnl=True
    prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
else if blocks[blocknumber-1][1] =2 or blocks[blocknumber-2][1] =2:
    turnr=True
    prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
else:
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
        gyro=False
        if count_turns<=4:
            picam2.start()
            time.sleep(0.5)
            temp =scan_frame()
            sacuvaj(temp)
            time.sleep(0.1)
            picam2.stop()
        prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
        for scan in safe_scans():
            #resetovanje varijabli za svaki krug
            suml = 0
            countl = 1
            sumr = 0
            countr = 1
            sumc=0
            countc=1
            #prolazenje kroz informacije sa lidara
            for (_, angle, distance) in scan:
                if  ((angle > minl1 and angle < maxl1)or(angle > minl2 and angle < maxl2)):
                    suml += distance
                    countl+=1
                if angle > minr and angle < maxr:
                    sumr += distance
                    countr+=1
                if angle > minc and angle < maxc:
                    sumc += distance
                    countc+=1


            #u slucaju da je skretanje zanemaruje proracune te salje informaciju za skretanje
            avgl = suml / countl
            avgr = sumr / countr
            avgc=sumc/countc
            if avgl >= 1000:
                lidar.stop()
                picam2.start()
                turning("l")
                count_turns+=1
                while skreno==True:
                    line = ser.readline().decode('utf-8').strip()
                    if line == "h":
                        skreno = False
                        lidar.clean_input()
                skreno=True
                if count_turns<=4:
                    temp =scan_frame()
                    sacuvaj(temp)
                    picam2.stop()
                break
            else :
                if br_kocki==1:
                    if avgc<1500 :
                        line = ser.readline().decode('utf-8').strip()
                        value = int(line)
                        if value==0 or value==1 gyro=line
                        if gyro==True:
                            prva_kocka = druga_kocka
                else:
                    if avgc<distanca*495 and avgc>(distanca-1)*495:
                        line = ser.readline().decode('utf-8').strip()
                        value = int(line)
                        if value==0 or value==1 gyro=line
                        if gyro==True:
                            picam2.start()
                            time.sleep(0.5)
                            temp =scan_frame()
                            sacuvaj(temp)
                            time.sleep(0.1)
                            picam2.stop()
                            prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
                            if druga_kocka!=0 prva_kocka=druga_kocka
                error = avgr - avgl - prva_kocka
                res = kp * error + kd * (last_Error - error)
                last_Error = error
                send_command(res)
  
#ako je odredjeno desno skretanje
else :
    while count_turns<12:
        skreno=True
        gyro=False
        if count_turns<=4:
            picam2.start()
            time.sleep(0.5)
            temp =scan_frame()
            sacuvaj(temp)
            time.sleep(0.1)
            picam2.stop()
        prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
        for scan in safe_scans():
            suml = 0
            countl = 1
            sumr = 0
            countr = 1
            sumc=0
            countc=1
            for (_, angle, distance) in scan:
                if  ((angle > minl1 and angle < maxl1)or(angle > minl2 and angle < maxl2)):
                    suml += distance
                    countl+=1
                if angle > minl and angle < maxr:
                    sumr += distance
                    countr+=1
                if angle > minc and angle < maxc:
                    sumc += distance
                    countc+=1
            avgl = suml / countl
            avgr = sumr / countr
            avgc = sumc / countc
            if avgr >= 1000:
                lidar.stop()
                picam2.start()
                turning("r")
                count_turns+=1
                while skreno==True:
                    line = ser.readline().decode('utf-8').strip()
                    if line == "h":
                        lidar.clean_input()
                        skreno = False
                skreno=True
                if count_turns<=4:
                    temp=scan_frame()
                    sacuvaj(temp)
                    picam2.stop()
                break

            else :
                if br_kocki==1:
                    if avgc<1500 :
                        line = ser.readline().decode('utf-8').strip()
                        value = int(line)
                        if value==0 or value==1 gyro=line
                        if gyro==True:
                            prva_kocka = druga_kocka
                else:
                    if avgc<distanca*495 and avgc>(distanca-1)*495:
                        line = ser.readline().decode('utf-8').strip()
                        value = int(line)
                        if value==0 or value==1 gyro=line
                        if gyro==True:
                            picam2.start()
                            time.sleep(0.5)
                            temp =scan_frame()
                            sacuvaj(temp)
                            time.sleep(0.1)
                            picam2.stop()
                            prva_kocka, druga_kocka, br_kocki, distanca=obilazenje()
                            if druga_kocka!=0 prva_kocka=druga_kocka
                error = avgr - avgl - offset
                res = kp * error + kd * (last_Error - error)
                last_Error = error
                send_command(res)
#nakon 3 runde salje informaciju robotu da stane u tom kvadrantu
turning("e")       
lidar.disconnect()
