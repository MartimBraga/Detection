# Detection
Repository for code and files developed for Master Thesis
## INSTALLATION
Follow the installation instructions of the systems in DURABLE SIMULATION ARDUPILOT:https://github.com/durable-ist/Multi_Robot_Simulation

After this add the detection folder to your catkin_ws and run the following:
```
cd ~/catkin_ws/
catkin build
```
Install DroneKit followind the instruction at https://dronekit-python.readthedocs.io/en/latest/develop/installation.html, or run the following commands:
```
pip install dronekit
sudo apt-get install python-pip python-dev
sudo pip install dronekit
```
modify the content of the corresponding folders with the ones supplyed.

## USAGE
To run the simulation, 4 terminals are required.
On the first lauch the gazebo simulator :
```
roslaunch durable_gazebo_simulation durable_sim.launch 
```
On the second one run the Ardupilot simulator, run this command from the ArduCopter folder in the ardupilot package
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0 -L Evora

```
On the third, launch the depth camera detection on the detection folder on the catkin_ws.
```
cd catkin_ws/src/detection/src
python talker.py 
```
On the last one, run the Dronekit landing routine on the detection folder on the catkin_ws.
```
cd catkin_ws/src/detection/src/pid
python autoland.py --connect 127.0.0.1:14550

```

## Modification

To make modifications to the landing routine change the parameters on the autoland.py. 