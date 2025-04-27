package frc.robot.sim;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorSim {

    enum OutputType {
        Voltage,
        Follow
    }

    class Output {
        public final OutputType type;
        public final double voltage;
        public final int followerId;
        public final boolean followerInverted;

        Output(OutputType type, double voltage, int followerId, boolean followerInverted) {
            this.type = type;
            this.voltage = voltage;
            this.followerId = followerId;
            this.followerInverted = followerInverted;
        }

        public static Output voltage(double voltage) {
            return new Output(OutputType.Voltage, voltage, 0, false);
        }

        public static Output follow(int id, boolean inverted) {
            return new Output(OutputType.Follow, 0, id, inverted);
        }
    }

    int getId();

    void setInverted(boolean inverted);
    void setPosition(Angle position);
    void setVelocity(AngularVelocity velocity);

    Output updateOutput(Voltage busVoltage);
}
