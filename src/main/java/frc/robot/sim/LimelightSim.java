package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.*;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotMap;

import java.util.*;
import java.util.stream.Collectors;

public class LimelightSim {

    public static class State {
        public final int selectedAprilTagId;
        public final List<AprilTag> seenAprilTags;
        public final Pose3d limelightPose;

        public State(int selectedAprilTagId, List<AprilTag> seenAprilTags, Pose3d limelightPose) {
            this.selectedAprilTagId = selectedAprilTagId;
            this.seenAprilTags = seenAprilTags;
            this.limelightPose = limelightPose;
        }
    }

    private static final Number[] EMPTY_NUMBER_ARRAY = new Number[0];

    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d limelightPoseInRobot;
    private final CameraSim camera;

    private final NetworkTableEntry seenAprilTagIdsEntryData;

    private final NetworkTableEntry hasTargetEntryTarget;
    private final NetworkTableEntry targetIdEntryTarget;
    private final NetworkTableEntry targetPoseCamSpaceEntryTarget;
    private final Double[] targetPoseCamSpaceArr;

    public LimelightSim(AprilTagFieldLayout fieldLayout, Transform3d limelightPoseInRobot, NetworkTable dataTable) {
        this.fieldLayout = fieldLayout;
        this.limelightPoseInRobot = limelightPoseInRobot;
        this.camera = new CameraSim(20, 20, 10); // todo: check for good values

        seenAprilTagIdsEntryData = dataTable.getEntry("SeenAprilTags");
        seenAprilTagIdsEntryData.setNumberArray(EMPTY_NUMBER_ARRAY);

        NetworkTable targetDataTable = NetworkTableInstance.getDefault().getTable(RobotMap.LIMELIGHT_NAME);
        hasTargetEntryTarget = targetDataTable.getEntry(RobotMap.LIMELIGHT_ENTRY_HAS_TARGET);
        targetIdEntryTarget = targetDataTable.getEntry(RobotMap.LIMELIGHT_ENTRY_TARGET_ID);
        targetPoseCamSpaceEntryTarget = targetDataTable.getEntry(RobotMap.LIMELIGHT_ENTRY_TARGET_POSE);

        hasTargetEntryTarget.setBoolean(false);
        targetIdEntryTarget.setNumber(-1);
        targetPoseCamSpaceArr = new Double[6];
        Arrays.fill(targetPoseCamSpaceArr, 0.0);
        targetPoseCamSpaceEntryTarget.setNumberArray(targetPoseCamSpaceArr);
    }

    public State update(Pose3d robotPose) {
        Pair<OptionalInt, List<AprilTag>> tagInfo = findSeenAprilTag(robotPose);
        publishAprilTagInfo(tagInfo.getSecond());

        OptionalInt seenTag = tagInfo.getFirst();
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

        Pose3d llPose = robotPose.plus(limelightPoseInRobot);
        return new State(seenTag.orElse(-1), Collections.unmodifiableList(tagInfo.getSecond()), llPose);
    }

    private Pair<OptionalInt, List<AprilTag>> findSeenAprilTag(Pose3d robotPose) {
        Pose3d llPose = robotPose.plus(limelightPoseInRobot);
        List<AprilTag> visibleTags = getAllVisibleTags(fieldLayout.getTags(), llPose);
        if (visibleTags.isEmpty()) {
            return Pair.of(OptionalInt.empty(), visibleTags);
        }

        AprilTag aprilTag = findBestTag(visibleTags, llPose);
        if (aprilTag == null) {
            return Pair.of(OptionalInt.empty(), visibleTags);
        }

        return Pair.of(OptionalInt.of(aprilTag.ID), visibleTags);
    }

    private List<AprilTag> getAllVisibleTags(List<AprilTag> tags, Pose3d limelightPose) {
        List<AprilTag> results = new ArrayList<>();

        for (AprilTag aprilTag : tags) {
            Translation3d translation = aprilTag.pose.getTranslation();
            translation = translation.minus(limelightPose.getTranslation()); // remove distance from cam

            Vector<N3> point = toVector(translation);
            Rotation3d pointRotation = aprilTag.pose.getRotation();
            Rotation3d camRotation = limelightPose.getRotation();

            if (camera.isPointInFov(point, camRotation) && camera.isFacingFront(pointRotation, camRotation)) {
                results.add(aprilTag);
            }
        }

        return results;
    }

    private AprilTag findBestTag(List<AprilTag> aprilTags, Pose3d limelightPose) {
        AprilTag best = null;
        double closestDistance = -1;

        for (AprilTag aprilTag : aprilTags) {
            double distanceToLl = aprilTag.pose.getTranslation().getDistance(limelightPose.getTranslation());
            if (closestDistance < 0 || closestDistance > distanceToLl) {
                best = aprilTag;
                closestDistance = distanceToLl;
            }
        }

        return best;
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
        hasTargetEntryTarget.setBoolean(false);
    }

    private void publishNewData(int targetId, Pose3d targetPoseCamSpace) {
        hasTargetEntryTarget.setBoolean(true);
        targetIdEntryTarget.setNumber(targetId);

        targetPoseCamSpaceArr[0] = targetPoseCamSpace.getX();
        targetPoseCamSpaceArr[1] = targetPoseCamSpace.getY();
        targetPoseCamSpaceArr[2] = targetPoseCamSpace.getZ();
        targetPoseCamSpaceArr[3] = targetPoseCamSpace.getRotation().getX();
        targetPoseCamSpaceArr[4] = targetPoseCamSpace.getRotation().getY();
        targetPoseCamSpaceArr[5] = targetPoseCamSpace.getRotation().getZ();
        targetPoseCamSpaceEntryTarget.setNumberArray(targetPoseCamSpaceArr);
    }

    private void publishAprilTagInfo(List<AprilTag> seenAprilTags) {
        Number[] idsArr = seenAprilTags.stream()
                .map((ap)-> ap.ID)
                .collect(Collectors.toList())
                .toArray(EMPTY_NUMBER_ARRAY);
        seenAprilTagIdsEntryData.setNumberArray(idsArr);
    }

    private static Vector<N3> toVector(Translation3d translation) {
        return VecBuilder.fill(translation.getX(), translation.getY(), translation.getZ());
    }
}
