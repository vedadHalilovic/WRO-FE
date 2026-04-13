# WRO-FE

**1.1 Team overview**

Team name: MAD Robots

**Team members:**

-   Vedad Halilovic-3^rd^ grade of Electrotechnical school in Tuzla

> Responsible for construction, electronics, ESP32 code (implementation
> of said electronics), documentation.

-   Ahmed Imsiragic-3^rd^ grade of Mesa Selimovic gymnasium in Tuzla

> Responsible for RPI code(python) development and debugging, LIDAR and
> camera implementation.

-   Emir Mujezinovic-3^rd^ grade of Mesa Selimovic gymnasium in Tuzla

**Team mentors:**

-   Dejan Bojic- electrical engineer

> Responsible for communication with competition organizers and suying
> parts.

-   Ivan Bosankic-electrical engineer

Responsible for suplying the tools and parts

**1.2 Mission objectives**

The robot we will be talking about is built to regulations provided by
WRO association, as a part of the WRO Future Engineers category.

Our task was to make a self-driving car that will be tested through two
rounds, first round where it navigates around a clear track, also known
as a free run/open track and a second round where there are added
obstacles that need to be avoided accordingly. We achieved this by using
a suite of sensors and of course the accompanying code.

The robot's uniqueness lies in it's guidance system, where it uses LIDAR
for driving straight and avoiding obstacles as well as using a gyro for
90 degree turns and a camera for detecting obstacle colours. We combined
this with RPI 4 model B which takes the raw data from LIDAR and camera
and generates navigation data that is further executed by the ESP32 that
receives the data and using it controls the steer and drive components
(servo and DC motor).

1.  **Car description**

> **2.1 Challenge 1-open track**

-   LIDAR detects walls so the robot stays centerlined and detects an
    > incoming turn.

    > RPI generates navigation data in degrees and sends it to the ESP32
    > and ensures that the robot completes 3 laps and stops in the right
    > spot.

    > ESP steers the car according to the data from RPI to keep the
    > robot centerlined and when needed initiate a 90-degree turn

    > ICM-20948 tracks the yaw and ensures that the robot turned 90
    > degrees left or right.

1.  1.  **Challenge 2-obstacle course**

-   The robot starts from a parking spot using the logic implemented in
    > the code to navigate out of the said spot.

    > Uses LIDAR to keep centreline and to detect obstacles and turns.

    > To differentiate two types of obstacles we use a Pi cam module,
    > based on whose data robot knows to go left or right of the
    > obstacle.

**2.3 Hardware components**

Electronic components that were needed are as follows:

  --------------- --------------- --------------- ------------------------
                                                  

                                                  

                                                  

                                                  

                                                  

                                                  

                                                  

                                                  

                                                  

                                                  

                                                  
  --------------- --------------- --------------- ------------------------

**2.4** **Mechanical design**

-   **Base/lower deck:** PETG 3D printed with honeycomb pattern for
    > weight savings.

    > **Top deck:** PETG 3D printed with honeycomb pattern for weight
    > savings, attached to lower deck using four 3D printed pillars.

    > **Camera mount:** 3D printed beam attached to the top deck using
    > adhesive, camera is pointed down at a 15-degree angle for more
    > accurate readings with less ambient light interference.

    > **Cooling system:** heatsinks on RPI and buck converter.

    > **Fasteners:** M3 bolts with corresponding nuts.

    > **Drivetrain:** connected to drive wheels using â4mm D-shaped
    > axle(shaft) with a 3D printed spur gear (module 1, 14 teeth) with
    > the same being connected to the motor with a 28 teeth module 1
    > spur gear. Driveshaft side movement limited using â4mm shaft
    > collars.

    > **Steering system:** Steering system uses 3D printed steering
    > axles, with LEGO spur gears and make block hubs that connect to
    > the wheels. Servo is connected to one of the gears, multiple gears
    > are used as to clear to LIDAR. Everything is mounted using M3, M4
    > bolts and 4mm shaft collars.

**2.5 Electrical design**

Our robot uses a custom PCB design to avoid unnecessary wires and
breadboards. The PCB connects everything on the ESP side, in other terms
it enables us to almost completely push jumper wires out on ESP side,
with a few exceptions being connection RPIâESP32 and servo PWM signal
wire.â¨By using a single high current buck converter we save a
considerable amount of space on the top deck, which in turn allows for a
more compact design.

**2.5.1 Buck converter**

We are using a 5A buck converter that powers all electronics with the
exception being the drive motor.â¨It's role is to step the voltage down,
so it is safe for our controllers and sensors.â¨A heatsink was installed
to provide adequate cooling because of high currents.

**2.5.2 Buttons**

Buttons provide a way to initiate our programs without accessing the RPI
remotely per the regulations. First button is used to initiate the code
for the first challenge, with the second button being used for the
second challenge.

**2.5.3 Motor driverâ¨**Adafruit dual H-bridge TB-6612 motor driver was
chosen because it best fit our use case, with it being compact,
supporting up to 1.2A on 5-13V DC.â¨It controls the motor movement
direction by applying 5V to AIN1 or AIN2. â¨Motor speed is controlled via
a PWM signal from the ESP32 to the driver's PWMA pin.

(electrical schemes are in the schemes folder)

**3 Software design**

**Language:** Python (RPI side), C++ (ESP32 side)â¨**Image processing:**
OpenCV

â¨**Modules:**

-   **Position tracking:** LIDAR

    > **Turning:** Gyro

    > **Drive control:** ESP32

```{=html}
<!-- -->
```
-   **Corrections** are made using PD-based steering

**Flowcharts:**

-   **System startup:** Power on â Initialize sensors â Calibration â
    > Ready

    > **Challenge 1:** LIDAR keeping robot centred driving forwardâ
    > LIDAR detecting a turnâ gyro executing a 90-degree turn â robot
    > completes 3 laps â stops in segment it started from

    > **Challenge 2:** LIDAR keeping robot centred driving forward â
    > LIDAR detecting an obstacle â camera turning on and detecting the
    > colour â robot passing the obstacle on the correct side â loop
    > until finish â stop

**Uploading the code (for reproduction purposes)**

**IDE-s used:**

-   **Arduino IDE (for ESP32)**

    > **Thonny (for RPI)**

**Libraries (ESP32):**

-   \<Arduino.h\> - used to enable Arduino API

    > \<ESP32Servo.h\> - used for servo control

    > \<Wire.h\> - enables I2C communication

    > \<Adafruit_ICM20948.h\> - used for interacting with/controlling
    > the gyro

**Libraries (RPI):**

-   rplidar -- needed to use the LIDAR

    > serial -- enables communication between ESP32 and RPI

    > time -- enables time.sleep() function

**Uploading the code to ESP:**

After installing libraries for ESP32 you need to download ESP32 in
Arduino IDE to be able to upload your code, as well as CP210X drivers.

Open Arduino IDE â connect ESP to your computer â select your ESP32
board in board manager â click the upload â after the upload has
finished press the RST button on ESP32

â¨â¨â¨

> **â¨â¨**