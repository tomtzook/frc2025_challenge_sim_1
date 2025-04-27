package frc.robot.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.lang.reflect.Field;
import java.util.Objects;

public class TalonFxMotorSim implements MotorSim {

    enum ControlType {
        Percent,
        Position,
        Velocity,
        Follow
    }

    private final TalonFX motor;
    private final TalonFXConfiguration configuration;
    private final TalonFXSimState simState;
    private final int id;

    private final Field controlReqField;
    private ControlRequest lastControlRequest;
    private ControlType controlType;
    private double controlValue;
    private double arbFeedForwardVolts;
    private int followedMotor;
    private boolean followedInvert;

    private final PIDController pidController;

    private double currentPosition;
    private double positionOffset;
    private double currentVelocity;

    public TalonFxMotorSim(TalonFX motor) {
        this.motor = motor;
        simState = motor.getSimState();
        this.configuration = new TalonFXConfiguration();
        this.controlReqField = getControlRequestField();
        this.id = motor.getDeviceID();

        simState.setSupplyVoltage(12);
        simState.setRotorVelocity(0);
        simState.setRawRotorPosition(0);
        simState.setRotorAcceleration(0);
        simState.setReverseLimit(false);
        simState.setForwardLimit(false);

        lastControlRequest = null;
        controlType = null;
        controlValue = 0;
        arbFeedForwardVolts = 0;
        followedMotor = -1;
        followedInvert = false;
        pidController = new PIDController(0, 0, 0);
        currentPosition = 0;
        positionOffset = 0;
        currentVelocity = 0;

        reloadConfiguration();
    }

    @Override
    public int getId() {
        return id;
    }

    @Override
    public void setInverted(boolean inverted) {
        simState.Orientation = inverted ? ChassisReference.CounterClockwise_Positive : ChassisReference.Clockwise_Positive;
    }

    @Override
    public void setPosition(Angle position) {
        double positionRotations = position.in(Units.Rotations);
        simState.setRawRotorPosition(positionRotations + positionOffset);
        currentPosition = positionRotations;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        double velocityRps = velocity.in(Units.RotationsPerSecond);
        simState.setRotorVelocity(velocityRps);
        currentVelocity = velocityRps;
    }

    @Override
    public Output updateOutput(Voltage busVoltage) {
        ControlRequest controlRequest = getCurrentControlRequest(motor, controlReqField);
        setControlByControlRequest(controlRequest);

        // todo: reload config in loop too slow, we won't see config changes
        //reloadConfiguration();

        double outputVoltage = 0;
        double busVoltageVolts = busVoltage.in(Units.Volt);

        if (controlType != null) {
            double outputPercent = 0;
            switch (controlType) {
                case Percent:
                    outputPercent = runInDutyCycle();
                    break;
                case Position:
                case Velocity:
                    outputPercent = runInPid();
                    break;
                case Follow:
                    return Output.follow(followedMotor, followedInvert);
                default:
                    break;
            }

            double ff = arbFeedForwardVolts;
            if (configuration.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive) {
                outputPercent = -outputPercent;
                ff = -arbFeedForwardVolts;
            }

            outputVoltage = (outputPercent * busVoltageVolts) + ff;
            outputVoltage = MathUtil.clamp(outputVoltage, -busVoltageVolts, busVoltageVolts);
        }

        return Output.voltage(outputVoltage);
    }

    private void setWantedPositionValue(Angle position) {
        double positionRotations = position.in(Units.Rotations) * configuration.Feedback.SensorToMechanismRatio;
        positionOffset = positionRotations - currentPosition;

        simState.setRawRotorPosition(positionRotations);
    }

    private void reloadConfiguration() {
        motor.getConfigurator().refresh(this.configuration);

        // todo: update according to used slot
        pidController.setPID(this.configuration.Slot0.kP, this.configuration.Slot0.kI, this.configuration.Slot0.kD);
    }

    private void setControlByControlRequest(ControlRequest controlRequest) {
        if (controlRequest == null || controlRequest instanceof EmptyControl) {
            lastControlRequest = null;
            return;
        }

        if (lastControlRequest != null && lastControlRequest.getControlInfo().equals(controlRequest.getControlInfo())) {
            // already executing
            return;
        }

        if (controlRequest instanceof NeutralOut) {
            setControl(0, ControlType.Percent, 0);
        } else if (controlRequest instanceof DutyCycleOut) {
            DutyCycleOut dutyCycleOut = (DutyCycleOut) controlRequest;
            setControl(dutyCycleOut.Output, ControlType.Percent, 0);
        } else if (controlRequest instanceof PositionDutyCycle) {
            PositionDutyCycle positionDutyCycle = (PositionDutyCycle) controlRequest;
            setControl(positionDutyCycle.Position, ControlType.Position, positionDutyCycle.FeedForward * 12);
        } else if (controlRequest instanceof Follower) {
            Follower follower = (Follower) controlRequest;
            setControlFollow(follower.MasterID, follower.OpposeMasterDirection);
        } else {
            throw new UnsupportedOperationException("unsupported control mode: " + controlRequest);
        }

        lastControlRequest = controlRequest;
    }

    private void setControl(double controlValue, ControlType controlType, double arbFeedForwardVolts) {
        this.controlType = controlType;
        this.controlValue = controlValue;
        this.arbFeedForwardVolts = arbFeedForwardVolts;

        pidController.reset();
    }

    private void setControlFollow(int id, boolean invert) {
        this.controlType = ControlType.Follow;
        this.followedMotor = id;
        this.followedInvert = invert;
    }

    private double runInDutyCycle() {
        return MathUtil.clamp(controlValue, -1, 1);
    }

    private double runInPid() {
        double processVariable;
        double setPoint;
        switch (controlType) {
            case Position:
                processVariable = (currentPosition + positionOffset) / configuration.Feedback.SensorToMechanismRatio;
                setPoint = controlValue;
                break;
            case Velocity:
                processVariable = currentVelocity / configuration.Feedback.SensorToMechanismRatio;
                setPoint = controlValue;
                break;
            default:
                throw new UnsupportedOperationException();
        }

        double output = pidController.calculate(processVariable, setPoint);
        return MathUtil.clamp(output, -1, 1);
    }

    public static ControlRequest getCurrentControlRequest(TalonFX motor, Field controlReqField) {
        try {
            controlReqField.setAccessible(true);
            ControlRequest request = (ControlRequest) controlReqField.get(motor);
            Objects.requireNonNull(request);

            return request;
        } catch (IllegalAccessException e) {
            throw new Error("unable to access control request", e);
        }
    }

    public static void setCurrentControlRequest(TalonFX motor, Field controlReqField, ControlRequest controlRequest) {
        try {
            controlReqField.setAccessible(true);
            controlReqField.set(motor, controlRequest);
        } catch (IllegalAccessException e) {
            throw new Error("unable to access control request", e);
        }
    }

    private static Field getControlRequestField() {
        try {
            Field field = ParentDevice.class.getDeclaredField("_controlReq");
            Objects.requireNonNull(field);

            return field;
        } catch (NoSuchFieldException e) {
            throw new Error("unable to locate control request field", e);
        }
    }
}

