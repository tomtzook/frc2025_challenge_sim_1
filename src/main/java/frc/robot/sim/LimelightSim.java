package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotMap;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;

public class LimelightSim {

    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d limelightPoseInRobot;

    private final NetworkTableEntry hasTargetEntry;
    private final NetworkTableEntry targetIdEntry;
    private final NetworkTableEntry targetPoseCamSpaceEntry;
    private final Double[] targetPoseCamSpaceArr;

    public LimelightSim(AprilTagFieldLayout fieldLayout, Transform3d limelightPoseInRobot) {
        this.fieldLayout = fieldLayout;
        this.limelightPoseInRobot = limelightPoseInRobot;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(RobotMap.LIMELIGHT_NAME);
        hasTargetEntry = table.getEntry("tv");
        targetIdEntry = table.getEntry("tid");
        targetPoseCamSpaceEntry = table.getEntry("targetpose_camspace");

        hasTargetEntry.setBoolean(false);
        targetIdEntry.setNumber(-1);
        targetPoseCamSpaceArr = new Double[6];
        Arrays.fill(targetPoseCamSpaceArr, 0.0);
        targetPoseCamSpaceEntry.setNumberArray(targetPoseCamSpaceArr);
    }

    public void update(Pose3d robotPose) {
        OptionalInt seenTag = findSeenAprilTag(robotPose);
        if (seenTag.isEmpty()) {
            publishNoData();
        } else {
            int tagId = seenTag.getAsInt();

            Optional<Pose3d> optionalAprilTag = fieldLayout.getTagPose(tagId);
            if (optionalAprilTag.isEmpty()) {
                // see nothing
                throw new RuntimeException("got non-existent tag, not possible");
            }

            Pose3d aprilTagPose = optionalAprilTag.get();
            Pose3d targetPoseCamSpace = calculateTargetPose(robotPose, aprilTagPose);
            publishNewData(tagId, targetPoseCamSpace);
        }
    }

    private OptionalInt findSeenAprilTag(Pose3d robotPose) {
        // todo: implement
        return OptionalInt.of(5);
    }

    private Pose3d calculateTargetPose(Pose3d robotPose, Pose3d aprilTagPose) {
        Pose3d llPose = robotPose.plus(limelightPoseInRobot);
        Transform3d llToTag = aprilTagPose.minus(llPose);

        // add white noise
        Matrix<N6, N1> stdDevs = MatBuilder.fill(Nat.N6(), Nat.N1(), 0.01, 0.01, 0.01, 0.0001, 0.0001, 0.0001);
        Matrix<N6, N1> whiteNoise = StateSpaceUtil.makeWhiteNoiseVector(stdDevs);
        return new Pose3d(
                new Translation3d(
                        llToTag.getX() + whiteNoise.get(0, 0),
                        llToTag.getY() + whiteNoise.get(1, 0),
                        llToTag.getZ() + whiteNoise.get(2, 0)
                ),
                new Rotation3d(
                        llToTag.getRotation().getX() + whiteNoise.get(3, 0),
                        llToTag.getRotation().getY() + whiteNoise.get(4, 0),
                        llToTag.getRotation().getZ() + whiteNoise.get(5, 0)
                )
        );
    }

    private void publishNoData() {
        hasTargetEntry.setBoolean(false);
    }

    private void publishNewData(int targetId, Pose3d targetPoseCamSpace) {
        hasTargetEntry.setBoolean(true);
        targetIdEntry.setNumber(targetId);

        targetPoseCamSpaceArr[0] = targetPoseCamSpace.getX();
        targetPoseCamSpaceArr[1] = targetPoseCamSpace.getY();
        targetPoseCamSpaceArr[2] = targetPoseCamSpace.getZ();
        targetPoseCamSpaceArr[3] = targetPoseCamSpace.getRotation().getX();
        targetPoseCamSpaceArr[4] = targetPoseCamSpace.getRotation().getY();
        targetPoseCamSpaceArr[5] = targetPoseCamSpace.getRotation().getZ();
        targetPoseCamSpaceEntry.setNumberArray(targetPoseCamSpaceArr);
    }
}
