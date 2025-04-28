package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.Sim;
import frc.robot.subsystems.DriveSystem;

public class Robot extends TimedRobot {

    // todo: start robot at an unknown position on the field each time
    //      have to find real position with an april tag
    //      and go to a specific position in the field (maybe to an april tag?)

    private DriveSystem driveSystem;
    private GenericHID controller;

    private AprilTagFieldLayout fieldLayout;
    private Field2d field2d = new Field2d();

    @Override
    public void robotInit() {
        //noinspection ResultOfMethodCallIgnored
        Sim.getInstance(); // will initialize sim stuff

        driveSystem = new DriveSystem();
        controller = new GenericHID(0);

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        SmartDashboard.putData("field22222", field2d);
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Sim.getInstance().update();

        NetworkTable table = NetworkTableInstance.getDefault().getTable(RobotMap.LIMELIGHT_NAME);
        boolean tv = table.getEntry("tv").getBoolean(false);
        if (tv) {
            int id = table.getEntry("tid").getNumber(0).intValue();
            Number[] arr = table.getEntry("targetpose_camspace").getNumberArray(new Number[0]);
            Transform3d llToTag = new Transform3d(arr[0].doubleValue(), arr[1].doubleValue(), arr[2].doubleValue(), new Rotation3d(arr[3].doubleValue(), arr[4].doubleValue(), arr[5].doubleValue()));

            Pose3d aprilTagPose = fieldLayout.getTagPose(id).orElseThrow();
            Pose3d llPose = aprilTagPose.plus(llToTag.inverse());
            Pose3d robotPose = llPose.plus(RobotMap.LIMELIGHT_OFFSET_FROM_ROBOT_CENTER.inverse());

            field2d.setRobotPose(robotPose.toPose2d());
            field2d.getObject("apriltag").setPose(aprilTagPose.toPose2d());
            field2d.getObject("ll").setPose(llPose.toPose2d());
        }
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
