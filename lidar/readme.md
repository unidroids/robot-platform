# build sequence, result in bin
mkdir build
cd build
cmake ..
make -j$(nproc)


chmod +x ../bin/robot_lidar_tcp
# cp ../bin/robot_lidar_tcp ../../server/robot_lidar_tcp


# set mode
Bit	Function                                              - Value 0                     Value 1
0	Switch between standard FOV and wide-angle FOV	      - Standard FOV (180°)         Wide-angle FOV (192°)
1	Switch between 3D and 2D measurement mode	          - 3D measurement mode         2D measurement mode
2	Enable or disable IMU                                 - Disable IMU                 Enable IMU            
3	Switch between network port mode and serial port mode - Network port mode           Serial port mode
4	Switch the default startup mode of the laser radar    - AutoStart when powered on   Keep it from rotating and wait for the startup command when powered on
5-31	Reserved	Reserved	Reserved