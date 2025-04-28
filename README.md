# Challenge 2025 1

## Challenge 

In this challenge, your robot gets dropped onto the field at a random location. The location changes every time you run the simulation,
and your code does not have access to information on where it is dropped. In fact, the robot thinks it is at the 0 position of the 
field. 

You have to program the robot to
1. Find out its actual position on the field.
2. Go to a specific location on the field.

To find the location of the robot in the field, we will be using a simulated Limelight which can see April Tags.
The limelight will report the position of an April Tag to the robot, using which, the robot code can calculate its position.

Once the robot's location is discovered, it can then be moved to the wanted position on the field. Do not use PathPlanner,
make your own algorithm. You may use PID, or not, the choice is yours.

## Robot

The robot only has one system - the chassis, which is a simple tank drive. It also has a limelight
located on it, which can see April Tags in the field. 

`RobotMap` already contains all relevant information about the robot, do not change existing constants, is it may break functionality, 
but you may add new ones.

### Drive System

The drive system is a simple tank drive made up of 4 _Falcon500_ motors controlled by 4 _TalonFX_ motor controllers, 
and 4 _3-inch_ wheels. 

All the motors have functioning integrated encoders, accessible via the motor controllers. A _Pigeon2_ is also located
on the chassis.

IDs for the devices are present in `RobotMap`.

In order to complete the challenge, you will be required to implement this system, so the robot may be moved.

If you do not recall how to use some of the devices, see the following snippets
- [TalonFX Encoder](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#encoder-access-talon-fx-integrated-encoder)
- [TalonFX Basic Control](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-basic-control-talon-fx)
- [Pigeon2 Yaw](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#gyro-access-pigeon-2-yaw)

TODO: TALON FX POSITION LOOP

Implement the drive system in whatever manner you wish, with whatever methods and commands you need to complete the challenge. 
You may want to test the system code before trying to complete the actual challenge.  

At the end of the constructor for your system, call
```java
SimSystems.registerDrive(frontLeft, backLeft, frontRight, backRight, pigeon);
```
This will register your system with the simulation and allow it to function. Note that all device configurations must be done beforehand.

### Limelight

A limelight camera is placed on the robot, at `(0.1, 0, 0.8)` relative to the robot center, oriented to look forward.
This camera will report information on one visible April Tag (the best seen), in accordance with the actual April Tag
positions on the field.

Usage of Limelight is similar to how it is used on a real robot, but much simplified.

#### NetworkTables

_NetworkTables_ (_NT_) is a communication library and part of _WPILib_. It is used to transfer information between
robot devices. For example, it is used to send information to dashboard from the RoboRIO, or from Limelight to the RoboRIO.
As such, we will need to use it to read information from the limelight.

Information is generally structured following a pseudo file-system, with files (called _Entries_) containing data, and
folders (called _Tables_) storing files (_Entries_) and used for organization.

Each `NetworkTableEntry` (_Entry_) may contain a single data set, of a specific type. This may be a `double`, `boolean`, `String`, 
array or more.

Each `NetworkTable` (_Table_) may contain a number of entries and a numbers of sub tables, each identified by a _name_.

For example, when using `SmartDashboard.putNumber("Hello", 12)`, the entry named `Hello` is set with the value `12` in table 
`SmartDasboard`.

Paths are any easy way to identify tables and entries, especially when placed deep into the table tree. 
For example, the entry `Hello` mentioned above is identified by the path `/SmartDashboard/Hello`.

To access data, we will want to retrieve the entries which contain the data and either read or set them.
```java
private NetworkTableEntry ourEntry;
private NetworkTableEntry ourEntry2;

@Override
public void robotInit() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable('TableName');
    ourEntry = table.getEntry("EntryName");
    ourEntry2 = table.getEntry("EntryName2");
}

@Override
public void robotPeriodic() {
    double entryValue = ourEntry.getNumber(0);
    if (entryValue >= 0) {
        // do stuff

        ourEntry2.setNumber(entryValue / 2);
    }
}
```

#### Getting Limelight Information

The limelight information is divided into 3 different entries under the limelight table (name is `RobotMap.LIMELIGHT_NAME`).
- `tv`: `boolean`, `true` indicates and April Tag is seen, `false` otherwise.
- `tid`: `int`, id of the seen April Tag, or `-1` if not seen
- `targetpose_camspace`: `double[6]`, contains data about the april tag position relative to the limelight. 
    Array has the following data: `[x, y, z, roll, pitch, yaw]` (position is in meters, rotation is in radians).
    Be clear that this information is the "distance" of the April Tag from the *limelight*. It is not a position in the field.

> [!NOTE]
> this is not exactly the information and entries supplied by the real limelight, more on that below.

So to read information from the limelight, obtain the 3 specified entries, and periodically sample them. If
`tv` is `false`, do not sample the others since no April Tag is seen, and their values are unusable.
If it is `true` you may sample `tid` and `targetpose_camspace`. 

Example on reading the `targetpose_camspace` entry and converting it to a `Transform3d`:
```java
Number[] arr = entry.getNumberArray(new Number[0]);
Transform3d transform = new Transform3d(
        arr[0].doubleValue(), 
        arr[1].doubleValue(), 
        arr[2].doubleValue(), 
        new Rotation3d(
                arr[3].doubleValue(), 
                arr[4].doubleValue(), 
                arr[5].doubleValue()
        )
);
```

#### Field Coordinates

TODO

#### April Tag Layout

WPILib comes with information about the position of April Tag on the field. You can use this information to find April Tags
or to calculate positioning relative to april tags.

The following shows and example of using this API:
```java
private AprilTagFieldLayout fieldLayout;

@Override
public void robotInit() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // this is a must! all coordinates in the simulation follow this origin
    fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
}

@Override
public void robotPeriodic() {
    // returns 3D position info for April Tag ID 5
    // will crash the code if given ID is not of an existing April Tag.
    Pose3d aprilTagPose = fieldLayout.getTagPose(5).orElseThrow();
}
```

## Simulation

The following sections describe specifics about the behaviour and use of the simulation, as it
has some differences to a real robot.

> [!WARNING]
> Do not touch any of the simulation code (located under package `sim`)
> under any circumstances. In changes are likely to completely break the simulation.

### Field

For the purposes of this exercise, the field has the same layout as the real 2025
field, with the same April Tags and locations, but cleared of all ground obstacles.
That is, the robot can move anywhere, and the limelight will not be obstructed by any objects.

### Limitations

Although functional, the simulation has a few limitations and does not replicate real world behaviour exactly.
However, this is fine for the needs of this challenge.

#### TalonFX

For the most part, `TalonFX` functions normally in the typical uses we have for it.
However, there are few limitation in what support was implemented:
- Changing configuration of a device *after* registering to the simulation will not actually update the function of the device
    in the simulation. So configure everything ahead of registration, in the constructor of the subsystem.
- Setting sensor values is not supported, and will likely break sensor simulation.
- Only the following control modes are supported: 
  - `DutyCycleOut`
  - `NeutralOut`
  - `PositionDutyCycle`
  - `Follower`
- Only the motor's integrated sensor (`RotorSensor`) is supported.
- Only _Slot 0_ is supported for Closed Loop controls.
- Only the following configuration settings have effects:
  - `configuration.MotorOutput.Inverted`
  - `configuration.Feedback.SensorToMechanismRatio`
  - `configuration.Slot0.kP`
  - `configuration.Slot0.kI`
  - `configuration.Slot0.kD`

Motor following is supported, via the `Follower` control mode. Recursive following between motors is also
possible, but not circular.

When setting a control mode for a device, create a new instance of the control mode for each different device. That is,
do not use the same control mode instance (like `DutyCycleOut`) for two devices with different values.
The example below showcases bad use:
```java
public void move() {
    DutyCycleOut control = new DutyCycleOut(0);

    control.Output = 1;
    motor1.setControl(control);

    control.Output = 0.5;
    motor2.setControl(control);
}
```
This will make both motors run at `0.5`. Instead, do:
```java
public void move() {
    motor1.setControl(new DutyCycleOut(1));
    motor2.setControl(new DutyCycleOut(0.5));
}
```

> [!WARNING]
> This is only a problem in the simulation. The provided approach is bad on a real robot.
> So do not take this use to real code, just use it for this simulation.

#### Pigeon2

The pigeon only has basic support for reading Yaw information. No configuration
or special features are implemented in the simulation. Just access the Yaw signal to 
see orientation.

#### Limelight

The Limelight simulation does not actually simulation a full limelight, or any vision for that matter. As
such, its performance and behaviour in the simulation have no bearing on the real thing.

There are no configurable pipelines, camera control, fine-tuning and any other basic feature one can expect from a real
limelight. It is simply provides basic generated information about April Tags in its simulated Field of View.

This is also way the access information via _NT_ provided above is not the same as seen
on a real limelight, but a simplified version chosen for the needs of this challenge.

Do note that the limelight camera simulated has a limited field of view, and can only see things with in it.
Consider it like a real camera, which has a very specific FOV. The camera also has a distance limit, beyond which
it will not be able to see an April Tag. The specific specs are of no interest for this challenge.

### Running the Simulation

The project has several pre-made run configurations for your convenience. These configurations will be loaded automatically
when you open the project.

To use a run configuration, select it from the configurations drop down box and click on _run_

Select the `Simulate` run configuration and click on _run_. This will build and run the code in the simulator.

![image](https://github.com/user-attachments/assets/c6d534f5-fd32-4ad9-a6ab-7feff2d39753)

You may also run the simulation in debug mode and use the debugger by pressing the _Bug_ button next to _Run_.

### Testing My Code

In order to see if your code is correct, you may consult some information provided by the simulation
to the simulation GUI.

TODO INSERT SOME IMAGE

There are two sets of information provided by the simulation to the GUI
- In the network tables view, you may find the table `Sim`. This table contains a few sub-tables and entries
    showing the state of the simulated systems. This includes information on the actual state of the motors/devices
    and such. You may compare this information to what your code is reporting and see if it is the same.
- Under the `NetworkTables->SmartDashboard` tab, you may find `SimDebugField` This is a field view provided by the simulation.
    it provides both the robot's real position, the limelight's real position and all April Tags visible by the camera on the field.

Use either options to check if your code is current or not.

Read [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html) to see more on the simulation GUI.
