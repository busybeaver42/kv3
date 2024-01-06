# kv3
## Kinect Azure Framework
If you like my little framework, be a friend and donate little money

e.g. for a cup of coffee (Paypal: spende@angermayer.de).

## What the framework support
    - access to the data streams (images, mircophone array, and IMU sensor data)
    - ODAS support - detect the direction of the noise
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
https://github.com/busybeaver42/kv3/tree/main/doxyDocu/genDocu/html/index.html


or use: doxygen config file inside path ./kv3/doxyDocu/...

inside CLI:

cd /kv3/doxyDocu/
doxywizard Doxyfile

And create your documentation (press lower down corner the <next> button several time and

at the end press the <Run doxygen> button in the upper left corner )

    
### kv3 GUI - with pointcloud visualisation
![Alt-Text](/assets/kv3gui01.png "kv3 GUI - pointcloud")

### kv3 GUI - with mesh visualisation
![Alt-Text](/assets/kv3gui02.png "kv3 GUI - mesh")

### kv3 GUI - with mesh with normals visualisation
![Alt-Text](/assets/kv3gui03.png "kv3 GUI - smooth mesh via normals")

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
    
    
    
    
