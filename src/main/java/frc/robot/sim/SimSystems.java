package frc.robot.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public class SimSystems {


    public static void registerDrive(TalonFX frontLeft, TalonFX backLeft, TalonFX frontRight, TalonFX backRight, Pigeon2 pigeon2) {
        Sim.getInstance().getDriveSim().setHardware(new DriveSim.Hardware(frontLeft, backLeft, frontRight, backRight, pigeon2));
    }
}
