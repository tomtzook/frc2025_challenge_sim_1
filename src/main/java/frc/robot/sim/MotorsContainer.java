package frc.robot.sim;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.HashMap;
import java.util.Map;

public class MotorsContainer {

    private final Map<Integer, MotorSim> motors;

    public MotorsContainer() {
        motors = new HashMap<>();
    }

    public void register(MotorSim motor) {
        motors.put(motor.getId(), motor);
    }

    public Map<Integer, Double> updateOutput(Voltage busVoltage) {
        Map<Integer, Double> outputs = new HashMap<>();

        for (Map.Entry<Integer, MotorSim> entry : motors.entrySet()) {
            if (outputs.containsKey(entry.getKey())) {
                continue;
            }

            MotorSim.Output output = entry.getValue().updateOutput(busVoltage);
            switch (output.type) {
                case Voltage:
                    outputs.put(entry.getKey(), output.voltage);
                    break;
                case Follow: {
                    int id = output.followerId;
                    if (outputs.containsKey(id)) {
                        double volts = output.followerInverted ? -outputs.get(id) : outputs.get(id);
                        outputs.put(entry.getKey(), volts);
                    } else {
                        MotorSim master = motors.get(id);
                        MotorSim.Output outputMaster = master.updateOutput(busVoltage);
                        if (outputMaster.type != MotorSim.OutputType.Voltage) {
                            throw new RuntimeException("recursive follow not supported");
                        }

                        outputs.put(id, outputMaster.voltage);

                        double volts = output.followerInverted ? -outputMaster.voltage : outputMaster.voltage;
                        outputs.put(entry.getKey(), volts);
                    }
                    break;
                }
            }
        }

        return outputs;
    }

    public void updatePositions(Map<Integer, Angle> positions) {
        for (Map.Entry<Integer, MotorSim> entry : motors.entrySet()) {
            if (positions.containsKey(entry.getKey())) {
                entry.getValue().setPosition(positions.get(entry.getKey()));
            }
        }
    }

    public void updateVelocities(Map<Integer, AngularVelocity> velocities) {
        for (Map.Entry<Integer, MotorSim> entry : motors.entrySet()) {
            if (velocities.containsKey(entry.getKey())) {
                entry.getValue().setVelocity(velocities.get(entry.getKey()));
            }
        }
    }
}
