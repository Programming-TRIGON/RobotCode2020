# RobotCode2020

This is TRIGON 5990 robot code for the 2020 FRC season: INFINITE RECHARGE.

The code is written in Java and is based of WPILib Java control system.


## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details

## Package Functions
- [`frc.robot`](src/main/java/frc/robot)

    Contains the robot's central functions and runs the robot main loop. It also contains control for the drivers.

- [`frc.robot.subsystems`](src/main/java/frc/robot/subsystems)

    Contains code for subsystems. The subsystem are used by commands and the command scheduler for convenient control of the robot.
- [`frc.robot.commands`](src/main/java/frc/robot/commands)

    Contains code for general commands that do not require a specific subsystem.
- [`frc.robot.components`](src/main/java/frc/robot/components)

    Contains code for sensors and solenoids used by the robot. 

- [`frc.robot.motion_profiling`](src/main/java/frc/robot/motion_profiling)

    Contains all motion profiling code used for autonomous driving. Trapezoidal motion profiles are used for smooth acceleration and minimal slip using ramsete controllers.

- [`frc.robot.util`](src/main/java/frc/robot/utils)

    Contains a collection of assorted utilities classes used in the robot code. Check each file for more information.

- [`frc.robot.vision`](src/main/java/frc/robot/vision)

    Contains various classes that help with tracking and following vision targets using limelight 2.
![logo](images/TrigonLogo.png)