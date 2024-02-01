## Kinect Azure Framework - kv3
If you like my little framework, be a friend and donate little money

e.g. for a cup of coffee (Paypal: spende@angermayer.de).

## What the framework support
    - access to the data streams (images, mircophone array, and IMU sensor data)
    - ODAS support - detect the direction of the noise
    - ODAS cofiguration files for Kinect Azure micrphone array (kv3/cfg/...)
    - access and image handling via opencv 4
    - better support and intigration from PCL Lib (https://www.pcl.org/)
    - save RGBD,registered RGB pointcloud, mesh fro PCL and kinect sensor raw data with a running number
    - convert data between opencv MAT format and RGB Pointcloud


## Motivation 
My motivation was to use the microsoft kinect azure driver / API and make all more comfortbable to use.

## Install Instructions
### Precondition
    connect the Kinect Azure system via USB and connect the power supply.


### todo...

    sudo apt update
    sudo apt upgrade
    sudo apt install -y cmake git

    
#### Install..

    cd ~
    git clone https://github.com/busybeaver42/kv3.git
    cd kv3
    mkdir build
    cd build
    cmake ..
    make -j4
    
    optional:
    cmake-gui ..
    make -j4
    
    
#### execute

    ./bin/kv3

    
#### API documentation
    inside git repository:
   ../kv3/doxyDocu/genDocu/html/index.html
    
    or use: the doxygen config file "Doxyfile", inside path ./kv3/doxyDocu/...
    How to:
    cd /kv3/doxyDocu/
    doxywizard Doxyfile
    And create your documentation (press lower down corner the <next> button several time and
    at the end press the <Run doxygen> button in the upper left corner )
    
#### ODAS (Open embeddeD Audition System)
    based on following github repro, I have create an example to use the 
    ODAS information from the ODAS live system.
    https://github.com/introlab/odas
        
    You have to install the ODAS repro before. 
    Afterwards, my application can read and show the ODAS position data.
    The ODAS system need to know on which card and device the kinect azure is located.
    This example shows you how to use odas live system information in your
    own application, together with the datastream and rendering from the Kinect Azure.
    
    git checkout odas
    ./bin/kv3
    cd /odas/build/bin
    sudo ./odaslive -vc /odas/config/odaslive/kv3Socket.cfg

    perhaps you have to modify before the configuration file "kv3Socket.cfg". 
    use: arecord -l to get information you need to modify the card and device id
    inside the configuration file (modify it at line 20 - card and 21 - device).
    You will found a lot of example configuration files below /kv3/cfg/... 

    hint: perhaps you have to modify the source code for the default sound device
    equal to card and device of the kinect azure inside file
    ../kv3/sup/audio/include/kv3audio.h
    71 #define DEFAULT_DEVICE      "plughw:2,0" // plughw:card,device
       
    
### kv3 GUI - with pointcloud visualisation
![Alt-Text](/assets/kv3gui01.png "kv3 GUI - pointcloud")

### kv3 GUI - with mesh visualisation
![Alt-Text](/assets/kv3gui02.png "kv3 GUI - mesh")

### kv3 GUI - with mesh with normals visualisation
![Alt-Text](/assets/kv3gui03.png "kv3 GUI - smooth mesh via normals")

### kv3 GUI ODAS
![Alt-Text](/assets/kv3guiOdas.png "kv3 GUI ODAS")

#### Control Keys
    key <1> - Mode 3: RGB 1920x1080 ; Depth 1024x1024 
    key <2> - Mode 2: RGB 4096x3072 ; Depth  640x 576 
    key <3> - Mode 1: RGB 1280x 720 ; Depth  640x 576 
    key <4> - Mode 4: RGB 1280x 720 ; Depth  320x 288 
    key <5> - Mode 5: RGB 4096x3072 ; Depth 1024x1024 
    key <ESC> Exit
    key <s> try to save all data (incl. mesh as .stl-file) to "resultPath" (must be modified inside source code - default: /var/www/ramdev/...)
    key <j> switch to mjpeg format
    key <b> switch to BGRA format (default)
    key <l> live meshing B-Spline fitting (could take a while) and save the result as .obj-file.
    key <p> enable and disable the pointcloud/mesh visualisation
    key <n> smooth normals to smooth the mesh
    
    
    
    
