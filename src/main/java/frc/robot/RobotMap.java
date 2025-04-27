package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    private RobotMap() {}

    // general
    public static final double ROBOT_WEIGHT_KG = 20;
    public static final double ROBOT_WIDTH_M = 4.0;
    public static final double ROBOT_LENGTH_M = 5;

    // limelight
    public static final String LIMELIGHT_NAME = "limelight";
    public static final Transform3d LIMELIGHT_OFFSET_FROM_ROBOT_CENTER = new Transform3d(0, 0, 0, Rotation3d.kZero);

    // drive
    public static final int DRIVE_FRONT_RIGHT = 1;
    public static final int DRIVE_BACK_RIGHT = 2;
    public static final int DRIVE_FRONT_LEFT = 3;
    public static final int DRIVE_BACK_LEFT = 4;
    public static final int DRIVE_PIGEON = 5;
    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 5.71; // 5.71 : 1 (driver/driven)
    public static final double DRIVE_WHEEL_RADIUS_M = Units.inchesToMeters(3);
    public static final double DRIVE_WHEEL_CIRCUMFERENCE_M = 2 * Math.PI * DRIVE_WHEEL_RADIUS_M;
    public static final int DRIVE_SIDE_MOTOR_COUNT = 2;
    public static final double DRIVE_TRACK_WIDTH_M = ROBOT_WIDTH_M;
    public static final double DRIVE_MOMENT_OF_INERTIA = (Math.pow(ROBOT_LENGTH_M, 3) * ROBOT_WIDTH_M) / 12;
}
