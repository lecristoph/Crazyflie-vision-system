# Crazyflie-vision-system

This project allows the Crazyflie 2.0 by Bitcraze to detect an ArUco marker and navigate towards it

## Getting Started

The main program is named 'run.py', but before it can be run, it is required to replace the file 'motion_commander.py' in cflib with the modified version provided in this repo.

### Prerequisites

The following libraries needs to be installed in order to run the file 'run.py': <br />
• logging <br />
• numpy <br />
• OpenCV > version 3.0 <br />
• yaml <br />
• simple_pid <br />
• filterpy <br />
• cflib <br />
• transforms3d <br />
<br />

It is also required to have the Crazyradio PA, a camera and video transmitter connected on-board on the quadcopter itself and a wireless video receiver which will act as the webcam that receives the video feed from the on-board camera. For installation and configuration, referr to 'report.pdf'.

## Bitcraze AB

Please read https://github.com/bitcraze/crazyflie-lib-python for more information about the Python library which allows for communication with the Crazyflie 2.0 thorugh Python.

## Authors

* **Christoffer Karlsson** - *This project is part of my Bachelor's thesis in electrical engineering at Karlstad University, 2019* - 

## More information

For complete information about this project, please read 'report.pdf' in this repository.

## License

This project is licensed under the GPL License.

