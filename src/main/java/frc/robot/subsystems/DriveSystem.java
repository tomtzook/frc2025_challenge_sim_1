package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.SimSystems;

public class DriveSystem extends SubsystemBase {

    private final TalonFX frontLeft;
    private final TalonFX backLeft;
    private final TalonFX frontRight;
    private final TalonFX backRight;
    private final Pigeon2 pigeon;

    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    private final DutyCycleOut dutyCycleOutL = new DutyCycleOut(0);
    private final DutyCycleOut dutyCycleOutR = new DutyCycleOut(0);
    private final NeutralOut neutralOut = new NeutralOut();

    // todo:
    //      note: can't use the same control obj for two different requests in the same loop
    //      note: setting sensor values is not supported
    //      note: supported modes: NeutralOut, DutyCycleOut, PositionDutyCycle, Follower
    //      work: formalize exercise
    //      work: limelight sim. need to use perspective matrix to check what it sees
    //      work: dont divide task too much, let talya come up with it together we me on zoom, to practice breaking up tasks
    //              do task analysis together.
    //              most doc should be on how things work and what is required

    public DriveSystem() {
        frontLeft = new TalonFX(RobotMap.DRIVE_FRONT_LEFT);
        backLeft = new TalonFX(RobotMap.DRIVE_BACK_LEFT);
        frontRight = new TalonFX(RobotMap.DRIVE_FRONT_RIGHT);
        backRight = new TalonFX(RobotMap.DRIVE_BACK_RIGHT);
        pigeon = new Pigeon2(RobotMap.DRIVE_PIGEON);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        configuration.Slot0.kP = 0.1;
        frontLeft.getConfigurator().apply(configuration);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        configuration.Slot0.kP = 0.1;
        backLeft.getConfigurator().apply(configuration);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        configuration.Slot0.kP = 0.1;
        frontRight.getConfigurator().apply(configuration);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        configuration.Slot0.kP = 0.1;
        backRight.getConfigurator().apply(configuration);

        Pigeon2Configuration pigeonConfiguration = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(pigeonConfiguration);

        odometry = new DifferentialDriveOdometry(getHeading(), getLeftDistancePassed(), getRightDistancePassed(), Pose2d.kZero);
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        SimSystems.registerDrive(frontLeft, backLeft, frontRight, backRight, pigeon);
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public double getLeftDistancePassed() {
        double rotationsWheel = frontLeft.getPosition(true).getValue().in(Units.Rotations);
        return rotationsWheel * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M;
    }

    public double getRightDistancePassed() {
        double rotationsWheel = frontRight.getPosition(true).getValue().in(Units.Rotations);
        return rotationsWheel * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M;
    }

    public void set(double left, double right) {
        dutyCycleOutL.Output = left;
        frontLeft.setControl(dutyCycleOutL);
        backLeft.setControl(dutyCycleOutL);

        dutyCycleOutR.Output = right;
        frontRight.setControl(dutyCycleOutR);
        backRight.setControl(dutyCycleOutR);
    }

    public void stop() {
        frontLeft.setControl(neutralOut);
        backLeft.setControl(neutralOut);
        frontRight.setControl(neutralOut);
        backRight.setControl(neutralOut);
    }

    public void setToPosition(double positionMeters) {
        double positionRotations = positionMeters / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M;
        PositionDutyCycle positionDutyCycleL = new PositionDutyCycle(0);
        positionDutyCycleL.Position = positionRotations;

        frontLeft.setControl(positionDutyCycleL);
        backLeft.setControl(new Follower(frontLeft.getDeviceID(), false));
        frontRight.setControl(new Follower(frontLeft.getDeviceID(), true));
        backRight.setControl(new Follower(frontLeft.getDeviceID(), true));
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getLeftDistancePassed(), getRightDistancePassed());
        field.setRobotPose(odometry.getPoseMeters());
    }
}
