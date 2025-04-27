package frc.robot.sim;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

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
            evaluate(entry.getValue(), busVoltage, outputs);
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

    private void evaluate(MotorSim motorSim, Voltage busVoltage, Map<Integer, Double> outputs) {
        if (outputs.containsKey(motorSim.getId())) {
            return;
        }

        MotorSim.Output output = motorSim.updateOutput(busVoltage);

        double volts;
        if (output.type == MotorSim.OutputType.Follow) {
            Set<Integer> visitedMotors = new HashSet<>(); // circular referencing protection
            visitedMotors.add(motorSim.getId());
            volts = evaluateMaster(output, busVoltage, outputs, visitedMotors);
        } else {
            volts = output.voltage;
        }

        outputs.put(motorSim.getId(), volts);
    }

    private double evaluateMaster(MotorSim.Output output, Voltage busVoltage, Map<Integer, Double> outputs, Set<Integer> visitedMotors) {
        if (visitedMotors.contains(output.followerId)) {
            throw new RuntimeException("Already visited this motor, there is a circular reference");
        }

        if (!outputs.containsKey(output.followerId)) {
            MotorSim master = motors.get(output.followerId);
            MotorSim.Output outputMaster = master.updateOutput(busVoltage);

            double volts;
            if (outputMaster.type == MotorSim.OutputType.Follow) {
                // recursion sucks, but it is the easiest solution here
                visitedMotors.add(output.followerId);
                volts = evaluateMaster(outputMaster, busVoltage, outputs, visitedMotors);
            } else {
                volts = outputMaster.voltage;
            }

            outputs.put(output.followerId, volts);
        }

        double volts = outputs.get(output.followerId);
        if (output.followerInverted) {
            volts = -volts;
        }

        return volts;
    }
}
