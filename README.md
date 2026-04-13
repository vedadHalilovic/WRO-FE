## WRO-FE

### **1.1 Team overview**

Team name: MAD Robots

**Team members:**

- Vedad Halilovic-3rd grade of Electrotechnical school in Tuzla

Responsible for construction, electronics, ESP32 code (implementation of said electronics), documentation.

- Ahmed Imsiragic-3rd grade of Mesa Selimovic gymnasium in Tuzla

Responsible for RPI code(python) development and debugging, LIDAR and camera implementation.

- Emir Mujezinovic-3rd grade of Mesa Selimovic gymnasium in Tuzla

**Team mentors:**

- Dejan Bojic- electrical engineer

Responsible for communication with competition organizers.

- Ivan Bosankic-electrical engineer

Responsible for supplying parts and tools.

### **1.2 Mission objectives**

The robot we will be talking about is built to regulations provided by WRO association, as a part of the WRO Future Engineers category.

Our task was to make a self-driving car that will be tested through two rounds, first round where it navigates around a clear track, also known as a free run/open track and a second round where there are added obstacles that need to be avoided accordingly. We achieved this by using a suite of sensors and of course the accompanying code.

The robot's uniqueness lies in it's guidance system, where it uses LIDAR for driving straight and avoiding obstacles as well as using a gyro for 90 degree turns and a camera for detecting obstacle colours. We combined this with RPI 4 model B which takes the raw data from LIDAR and camera and generates navigation data that is further executed by the ESP32 that receives the data and using it controls the steer and drive components (servo and DC motor).

## **Car description**

### **2.1 Challenge 1-open track**

- LIDAR detects walls so the robot stays centerlined and detects an incoming turn.
- RPI generates navigation data in degrees and sends it to the ESP32 and ensures that the robot completes 3 laps and stops in the right spot.
- ESP steers the car according to the data from RPI to keep the robot centerlined and when needed initiate a 90-degree turn
- ICM-20948 tracks the yaw and ensures that the robot turned 90 degrees left or right.

### **Challenge 2-obstacle course**

- The robot starts from a parking spot using the logic implemented in the code to navigate out of the said spot.
- Uses LIDAR to keep centreline and to detect obstacles and turns.
- To differentiate two types of obstacles we use a Pi cam module, based on whose data robot knows to go left or right of the obstacle.

### **2.3 Hardware components**

Electronic components that were needed are as follows:

| Component           | Function                                                                                                                   | Model                                                                     | Mounting/notes                                                                                                         |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| LIDAR               | Keeping the robot centerlined, detecting obstacles                                                                         | RPLIDAR A1                                                                | Mounted using the provided standoffs and securing it to the base of the construction with M3 bolts                     |
| Raspberry Pi 4      | Main controller                                                                                                            | Pi 4 model B 8GB                                                          | Mounted to the top deck of the construction using two M3 bolts                                                         |
| AZ-Delivery ESP32   | Gyro/motor controller                                                                                                      | ESP32 NodeMCU Module WLAN WiFi Development Board Dev Kit C V2 with CP2102 | Mounted to the custom PCB using solder connections                                                                     |
| Servo motor         | Steering the robot by turning the steer wheels                                                                             | Hitec HS-422                                                              | Mounted using a custom made bracket, for fasteners M3 and M4 bolts with corresponding nuts were used                   |
| 12V DC drive motor  | Driving/propelling the robot                                                                                               | 25GA-370                                                                  | Mounted using PETG 3D printed mount and secured to the lower deck/base of the construction using M3 bolts and adhesive |
| Motor driver        | Controls the motors using PWM signals received from the ESP32 and powering the drive motor from the 11-12V source(battery) | Adafruit TB6612 motor driver                                              | Mounted to the custom PCB using solder connections                                                                     |
| ICM-20948           | Used to determine when the 90-degree turn is achieved                                                                      | GY-ICM20948V2                                                             | Mounted to the custom PCB using solder connections                                                                     |
| Buck converter      | Stepping the voltage from 11-12V to 5V suitable for 5V logic electronics                                                   | 5A high current                                                           | Mounted using adhesive to the top deck of the construction                                                             |
| 11.1V Li-Po battery | Powering the robot                                                                                                         | GOLDBAT 11.1V 5000mAh 3S 50C LiPo RC Battery                              | Mounted using a 3D printed bracket                                                                                     |
| Pi camera           | Detecting the obstacle colour                                                                                              | Raspberry Pi Camera Module 2                                              | Mounted above the LIDAR using PETG 3D printed beam                                                                     |

### **2.4** **Mechanical design**

- **Base/lower deck:** PETG 3D printed with honeycomb pattern for weight savings.
- **Top deck:** PETG 3D printed with honeycomb pattern for weight savings, attached to lower deck using four 3D printed pillars.
- **Camera mount:** 3D printed beam attached to the top deck using adhesive, camera is pointed down at a 15-degree angle for more accurate readings with less ambient light interference.
- **Cooling system:** heatsinks on RPI and buck converter.
- **Fasteners:** M3 bolts with corresponding nuts.
- **Drivetrain:** connected to drive wheels using ⌀4mm D-shaped axle(shaft) with a 3D printed spur gear (module 1, 14 teeth) with the same being connected to the motor with a 28 teeth module 1 spur gear. Driveshaft side movement limited using ⌀4mm shaft collars.
- **Steering system:** Steering system uses a combination of 3D printed parts and pre-made aluminium profiles as well as Lego spur gears. Axles are 3D printed, servo is connected to one of the gears, multiple gears are used as to clear the LIDAR. Everything is mounted using M3, M4 bolts and 4mm shaft collars.

### **2.5 Electrical design**

Our robot uses a custom PCB design to avoid unnecessary wires and breadboards. The PCB connects everything on the ESP side, in other terms it enables us to almost completely push jumper wires out on ESP side, with a few exceptions being connection RPI↔ESP32 and servo PWM signal wire.  
By using a single high current buck converter we save a considerable amount of space on the top deck, which in turn allows for a more compact design.

#### **2.5.1 Buck converter**

We are using a 5A buck converter that powers all electronics with the exception being the drive motor.  
It's role is to step the voltage down, so it is safe for our controllers and sensors.  
A heatsink was installed to provide adequate cooling because of high currents.

#### **2.5.2 Buttons**

Buttons provide a way to initiate our programs without accessing the RPI remotely per the regulations. First button is used to initiate the code for the first challenge, with the second button being used for the second challenge.

 **2.5.3 Motor driver**  
 
Adafruit dual H-bridge TB-6612 motor driver was chosen because it best fit our use case, with it being compact, supporting up to 1.2A on 5-13V DC.  
It controls the motor movement direction by applying 5V to AIN1 or AIN2.  
Motor speed is controlled via a PWM signal from the ESP32 to the driver's PWMA pin.

(electrical schemes are in the schemes folder)

**3 Software design**

**Language:** Python (RPI side), C++ (ESP32 side)  
**Image processing:** OpenCV

**Modules:**

- **Position tracking:** LIDAR
- **Turning:** Gyro
- **Drive control:** ESP32
- **Corrections** are made using PD-based steering

**Flowcharts:**

- **System startup:** Power on → Initialize sensors → Calibration → Ready
- **Challenge 1:** LIDAR keeping robot centred driving forward→ LIDAR detecting a turn→ gyro executing a 90-degree turn → robot completes 3 laps → stops in segment it started from
- **Challenge 2:** LIDAR keeping robot centred driving forward → LIDAR detecting an obstacle → camera turning on and detecting the colour → robot passing the obstacle on the correct side → loop until finish → stop

**Uploading the code (for reproduction purposes)**

**IDE-s used:**

- **Arduino IDE (for ESP32)**
- **Thonny (for RPI)**

**Libraries (ESP32):**

- &lt;Arduino.h&gt; - used to enable Arduino API
- &lt;ESP32Servo.h&gt; - used for servo control
- &lt;Wire.h&gt; - enables I2C communication
- &lt;Adafruit_ICM20948.h&gt; - used for interacting with/controlling the gyro

**Libraries (RPI):**

- rplidar - needed to use the LIDAR
- serial - enables communication between ESP32 and RPI
- time - enables time.sleep() function

**Uploading the code to ESP:**

After installing libraries for ESP32 you need to download ESP32 in Arduino IDE to be able to upload your code, as well as CP210X drivers.

Open Arduino IDE → connect ESP to your computer → select your ESP32 board in board manager → click the upload → after the upload has finished press the RST button on ESP32

This file in combination with the codes from src folder and schemes from schemes folder can be used to replicate our robot(feel free to try)

<br/><br/>



<br/>\*\*
