package frc.robot.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.RobotMap;

import java.util.Map;

public class DriveSim {

    public static class State {
        public final Rotation2d heading;
        public final double leftDistanceMeters;
        public final double rightDistanceMeters;

        public State(Rotation2d heading, double leftDistanceMeters, double rightDistanceMeters) {
            this.heading = heading;
            this.leftDistanceMeters = leftDistanceMeters;
            this.rightDistanceMeters = rightDistanceMeters;
        }
    }

    private final NetworkTableEntry leftOutEntry;
    private final NetworkTableEntry rightOutEntry;
    private final NetworkTableEntry leftPosEntry;
    private final NetworkTableEntry leftVelEntry;
    private final NetworkTableEntry rightPosEntry;
    private final NetworkTableEntry rightVelEntry;
    private final NetworkTableEntry headingEntry;

    private final DifferentialDrivetrainSim sim;

    private Hardware hardware;

    public DriveSim(NetworkTable table) {
        leftOutEntry = table.getEntry("LeftOutVolts");
        rightOutEntry = table.getEntry("RightOutVolts");
        leftPosEntry = table.getEntry("LeftPositionM");
        leftVelEntry = table.getEntry("LeftVelocityMps");
        rightPosEntry = table.getEntry("RightPositionM");
        rightVelEntry = table.getEntry("RightVelocityMps");
        headingEntry = table.getEntry("HeadingDeg");

        sim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(RobotMap.DRIVE_SIDE_MOTOR_COUNT),
            RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO,
            RobotMap.DRIVE_MOMENT_OF_INERTIA,
            RobotMap.ROBOT_WEIGHT_KG,
            RobotMap.DRIVE_WHEEL_RADIUS_M,
            RobotMap.DRIVE_TRACK_WIDTH_M,
            // [x, y, heading, left velocity, right velocity, left distance, right distance]
            MatBuilder.fill(Nat.N7(), Nat.N1(), 0, 0, 0.0001, 0.05, 0.05, 0.005, 0.005)
        );

        this.hardware = null;
    }

    public void setHardware(Hardware hardware) {
        this.hardware = hardware;
        this.hardware.initialize();
    }

    public State update() {
        if (hardware == null) {
            throw new IllegalStateException("hardware for DriveSim not defined");
        }

        Voltage busVoltage = Units.Volts.of(RobotController.getBatteryVoltage());

        Pair<Double, Double> outputs = hardware.updateOutput(busVoltage);
        double leftOutput = outputs.getFirst();
        double rightOutput = outputs.getSecond();
        sim.setInputs(leftOutput, rightOutput);

        leftOutEntry.setNumber(leftOutput);
        rightOutEntry.setNumber(rightOutput);

        sim.update(0.02);

        Angle leftPosition = positionMetersToRotorPosition(sim.getLeftPositionMeters());
        AngularVelocity leftVelocity = velocityMpsToRotorVelocity(sim.getLeftVelocityMetersPerSecond());
        Angle rightPosition = positionMetersToRotorPosition(sim.getRightPositionMeters());
        AngularVelocity rightVelocity = velocityMpsToRotorVelocity(sim.getRightVelocityMetersPerSecond());
        hardware.updateState(leftPosition, leftVelocity, rightPosition, rightVelocity, sim.getHeading());

        leftPosEntry.setNumber(sim.getLeftPositionMeters());
        leftVelEntry.setNumber(sim.getLeftVelocityMetersPerSecond());
        rightPosEntry.setNumber(sim.getRightPositionMeters());
        rightVelEntry.setNumber(sim.getRightVelocityMetersPerSecond());
        headingEntry.setNumber(sim.getHeading().getDegrees());

        return new State(sim.getHeading(), sim.getLeftPositionMeters(), sim.getRightPositionMeters());
    }

    private static Angle positionMetersToRotorPosition(double positionMeters) {
        return Units.Rotations.of(
                (positionMeters * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO) / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M
        );
    }

    private static AngularVelocity velocityMpsToRotorVelocity(double velocityMps) {
        return Units.RotationsPerSecond.of(velocityMps * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M);
    }

    public static class Hardware {
        private final MotorSim frontLeft;
        private final MotorSim backLeft;
        private final MotorSim frontRight;
        private final MotorSim backRight;
        private final GyroSim gyro;

        private final MotorsContainer motorsContainer;

        public Hardware(TalonFX frontLeft, TalonFX backLeft, TalonFX frontRight, TalonFX backRight, Pigeon2 pigeon2) {
            this.frontLeft = new TalonFxMotorSim(frontLeft);
            this.backLeft = new TalonFxMotorSim(backLeft);
            this.frontRight = new TalonFxMotorSim(frontRight);
            this.backRight = new TalonFxMotorSim(backRight);
            this.gyro = new GyroPigeon2Sim(pigeon2);

            motorsContainer = new MotorsContainer();
            motorsContainer.register(this.frontLeft);
            motorsContainer.register(this.backLeft);
            motorsContainer.register(this.frontRight);
            motorsContainer.register(this.backRight);
        }

        void initialize() {
            this.frontLeft.setInverted(false);
            this.frontLeft.setPosition(Units.Degrees.zero());
            this.frontLeft.setVelocity(Units.RPM.zero());

            this.backLeft.setInverted(false);
            this.backLeft.setPosition(Units.Degrees.zero());
            this.backLeft.setVelocity(Units.RPM.zero());

            this.frontRight.setInverted(true);
            this.frontRight.setPosition(Units.Degrees.zero());
            this.frontRight.setVelocity(Units.RPM.zero());

            this.backRight.setInverted(true);
            this.backRight.setPosition(Units.Degrees.zero());
            this.backRight.setVelocity(Units.RPM.zero());
        }

        Pair<Double, Double> updateOutput(Voltage busVoltage) {
            Map<Integer, Double> outputs = motorsContainer.updateOutput(busVoltage);

            double leftOutput = outputs.get(this.frontLeft.getId()) + outputs.get(this.backLeft.getId());
            // right is inverted
            double rightOutput = -(outputs.get(this.frontRight.getId()) + outputs.get(this.backRight.getId()));

            return Pair.of(leftOutput, rightOutput);
        }

        void updateState(Angle leftPosition, AngularVelocity leftVelocity,
                         Angle rightPosition, AngularVelocity rightVelocity,
                         Rotation2d rotation) {
            motorsContainer.updatePositions(Map.of(
                    frontLeft.getId(), leftPosition,
                    backLeft.getId(), leftPosition,
                    frontRight.getId(), rightPosition,
                    backRight.getId(), rightPosition
            ));

            motorsContainer.updateVelocities(Map.of(
                    frontLeft.getId(), leftVelocity,
                    backLeft.getId(), leftVelocity,
                    frontRight.getId(), rightVelocity,
                    backRight.getId(), rightVelocity
            ));

            gyro.setYaw(rotation.getDegrees());
        }
    }
}
