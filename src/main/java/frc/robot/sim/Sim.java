package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class Sim {

    private static final Sim INSTANCE = new Sim();

    public static Sim getInstance() {
        return INSTANCE;
    }

    private final AprilTagFieldLayout fieldLayout;
    private final DifferentialDriveOdometry odometry;

    private final DriveSim driveSim;
    private final LimelightSim limelightSim;

    private final NetworkTableEntry robotPoseEntry;
    private final Double[] robotPoseArr;

    private final Field2d simDebugField;

    public Sim() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Sim");

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        odometry = new DifferentialDriveOdometry(Rotation2d.kZero, 0, 0, Pose2d.kZero);

        driveSim = new DriveSim(table.getSubTable("Drive"));
        limelightSim = new LimelightSim(fieldLayout, RobotMap.LIMELIGHT_OFFSET_FROM_ROBOT_CENTER, table.getSubTable("Limelight"));

        Pose3d startingRobotPosition = createRandomRobotPose();
        odometry.resetPose(startingRobotPosition.toPose2d());

        robotPoseEntry = table.getEntry("RobotPose");
        robotPoseArr = new Double[6];
        Arrays.fill(robotPoseArr, 0.0);
        robotPoseEntry.setNumberArray(robotPoseArr);
        publishRobotPose(startingRobotPosition);

        simDebugField = new Field2d();
        SmartDashboard.putData("SimDebugField", simDebugField);
    }

    public void update() {
        DriveSim.State driveState = driveSim.update();
        odometry.update(driveState.heading, driveState.leftDistanceMeters, driveState.rightDistanceMeters);

        Pose3d robotPose = new Pose3d(odometry.getPoseMeters());
        LimelightSim.State limelightState = limelightSim.update(robotPose);

        simDebugField.setRobotPose(robotPose.toPose2d());
        List<Pose2d> seenAprilTagPoses = limelightState.seenAprilTags.stream()
                .map((ap -> ap.pose.toPose2d()))
                .collect(Collectors.toList());
        simDebugField.getObject("AprilTags").setPoses(seenAprilTagPoses);

        if (limelightState.selectedAprilTagId >= 0) {
            Pose2d pose = fieldLayout.getTagPose(limelightState.selectedAprilTagId).orElseThrow().toPose2d();
            simDebugField.getObject("SeenAprilTag").setPose(pose);
        } else {
            simDebugField.getObject("SeenAprilTag").setPoses();
        }

        simDebugField.getObject("Limelight").setPose(limelightState.limelightPose.toPose2d());

        publishRobotPose(robotPose);
    }

    DriveSim getDriveSim() {
        return driveSim;
    }

    private Pose3d createRandomRobotPose() {
        Random random = new Random();
        double randomX = random.nextDouble(fieldLayout.getFieldLength());
        double randomY = random.nextDouble(fieldLayout.getFieldWidth());
        double randomRot = random.nextDouble(360);

        return new Pose3d(randomX, randomY, 0, new Rotation3d(0, 0, Math.toRadians(randomRot)));
    }

    private Pose3d createFixedRobotPose(int referenceAprilTagId, Transform3d transformFromTag) {
        Pose3d aprilTag = fieldLayout.getTagPose(referenceAprilTagId).orElseThrow();
        return aprilTag.plus(transformFromTag).plus(RobotMap.LIMELIGHT_OFFSET_FROM_ROBOT_CENTER.inverse());
    }

    private void publishRobotPose(Pose3d robotPose) {
        robotPoseArr[0] = robotPose.getX();
        robotPoseArr[1] = robotPose.getY();
        robotPoseArr[2] = robotPose.getZ();
        robotPoseArr[3] = Math.toDegrees(robotPose.getRotation().getX());
        robotPoseArr[4] = Math.toDegrees(robotPose.getRotation().getY());
        robotPoseArr[5] = Math.toDegrees(robotPose.getRotation().getZ());
        robotPoseEntry.setNumberArray(robotPoseArr);
    }
}
