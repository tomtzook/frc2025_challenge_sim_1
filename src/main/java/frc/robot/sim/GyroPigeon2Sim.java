package frc.robot.sim;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.wpilibj.RobotController;

public class GyroPigeon2Sim implements GyroSim {

    private final Pigeon2SimState sim;

    public GyroPigeon2Sim(Pigeon2 pigeon) {
        sim = pigeon.getSimState();

        sim.setRawYaw(0);
        sim.setPitch(0);
        sim.setRoll(0);
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    @Override
    public void setYaw(double angleDegrees) {
        sim.setRawYaw(angleDegrees);
    }
}
