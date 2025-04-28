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
    //      note: when docing ex, make sure to explain how to show poses on field

    public DriveSystem() {
        // TODO: UNCOMMENT
        // SimSystems.registerDrive(frontLeft, backLeft, frontRight, backRight, pigeon);
    }

    @Override
    public void periodic() {

    }
}
