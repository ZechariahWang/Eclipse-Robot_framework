![newLogo](https://github.com/ZechariahWang/210Z-Over-Under-Worlds/assets/97078224/db011725-d4a7-4acb-8b84-9804be6e7e04)

# Eclipse 2023-2024 Library

Code Framework developed by 210Z during the 2023-2024 Vex Robotics Competition garnered throughout three years of development.

- Div Top 16 World Championships
- 7x World Championship Qualified
- 7x Regional Tournament Champions
- 6x Regional Tournament Finalists
- 9x Design, Skills, Sportsmanship, Judges, Innovate Awards
- 2x International Tournament Champion
- 1x Provincial Champion
- 2x Excellence Award
- 1x Think Award
- Western Mechatronics 2024 Excellence in Programming Award
- Calgary Stampede Agriculture Technology 2nd Place 

Contains Proportional Integral Derivative (PID) Controllers, Feedback Control MTP and MTRP, Kalman Filters, Bezier Curve Path Planners, Pure Pursuit Path Trackers, PID Constant Tuners, Odometry Position Localization, Holomonic Feedback Control MTP, Linear Motion Profilers with Inverse Kinematics, LVGL Integrated Graphics, Autonomous Selector, Metrics Modular Control.

## Project Usage
1: Install the package from this repository using ```git clone https://github.com/ZechariahWang/210Z_SPINUP_V2.0.git```

2: To set up the components of the robot, navigate to
``` Globals.cpp ```. It will be found within the ``` Miscellaneous ``` folder under ```CoreSystemVitals```. From there, manipulate the ports of your robot accordingly to the corresponding parts within the code.

3: For driver control, go to ```main.cpp``` and locate the ```opControl``` function. This is the main driver function for all opControl related functions within the robot. If you wish to change core driver vital systems, go to ```CoreAssets.cpp``` under ```DriveHandler``` in ```CoreSystemVitals```. For specific in game functions such as flywheels, rollers, etc, go to ```GameAssets.cpp``` under the same corresponding folder.

4: For autonomous control, there are three main components: PID Controller, Odometry Logic, and Algorithm Logic. These are all divided into their own seperate components within the project. Note that certain pieces of logic may require external module and utility functions, which can be found in the ```Modules``` and ```Utility``` folders.

5: For LVGL Embedded System Graphics, all logic will be found in ```main.cpp``` in the ```initialize``` function. Note that declaration for specific components may be found in other paths of the project. The current setup of the interface is in 4 components: Game, Sensor, Auton, Misc. Each section displays data about it's corresponding section, but may connect to other parts of the project. The biggest notable example of this is the autonomous selector. To edit or change logic within the selector, navigate to ```InitializeModule.cpp``` in the ```OnStartupModule``` folder.

## Authors

- [@ZechariahWang](https://github.com/ZechariahWang)

