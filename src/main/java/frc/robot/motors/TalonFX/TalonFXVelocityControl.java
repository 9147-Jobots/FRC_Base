package frc.robot.motors.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.motors.IMotorVelocityControl;

public class TalonFXVelocityControl extends TalonFXBase implements IMotorVelocityControl {

    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final static double DEFAULT_TOLERANCE = 0.05;

    /**
     * Creates a new VelocityControlMotorTalonFX with constants P, S and V.
     * @param deviceID
     * CAN ID of the TalonFX motor controller.
     * @param isInverted
     * Whether the motor output should be inverted.
     * @param positionConversionFactor
     * The factor to convert the motor's native units to the desired position units.
     * @param kP
     * The proportional gain for the PID controller.
     * @param kS
     * The static gain for the feedforward controller.
     * @param kV
     * The velocity gain for the feedforward controller.
     * @return
     */
    public static IMotorVelocityControl CreateTalonFXVelocityControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kS,
        double kV
    ) {
        return new TalonFXVelocityControl(deviceID, isInverted, positionConversionFactor, kP, 0, 0, kS, kV);
    }

    /**
     * DO NOT USE THIS UNLESS YOU KNOW WHAT YOU ARE DOING. Creates a new VelocityControlMotorTalonFX with constants P, I, D, S and V.
     * This constructor allows you to specify all of the PID and feedforward constants, but it is not recommended to use it unless you have a specific reason to do so, as it can lead to confusion and errors if the constants are not set correctly.
     * @param deviceID
     * CAN ID of the TalonFX motor controller.
     * @param isInverted
     * Whether the motor output should be inverted.
     * @param positionConversionFactor
     * The factor to convert the motor's native units to the desired position units.
     * @param kP
     * The proportional gain for the PID controller.
     * @param kI
     * The integral gain for the PID controller.
     * @param kD
     * The derivative gain for the PID controller.
     * @param kS
     * The static gain for the feedforward controller.
     * @param kV
     * The velocity gain for the feedforward controller.
     * @return
     */
    public static IMotorVelocityControl OverloadCreateTalonFXVelocityControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV
    ) {
        return new TalonFXVelocityControl(deviceID, isInverted, positionConversionFactor, kP, kI, kD, kS, kV);
    }

    /**
     * Creates a new VelocityControlMotorTalonFX with constants P, I, D, S and V.
     * @param deviceID
     * @param isInverted
     * @param positionConversionFactor
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     */
    private TalonFXVelocityControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV
    ) {
        super(new TalonFX(deviceID));

        // Configure the TalonFX with the provided parameters.
        configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kV = kV;
        configs.Feedback.SensorToMechanismRatio = positionConversionFactor; // Convert from sensor units to desired output units.
        
        talonFX.getConfigurator().apply(configs);
    }

    @Override
    public double getVelocity() {
        return talonFX.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return talonFX.getPosition().getValueAsDouble();
    }

    @Override
    public void runVelocity(double setpoint) {
        talonFX.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(0));
    }

    @Override
    public void runVelocity(double speedSetpoint, double ffVolts) {
        talonFX.setControl(velocityRequest.withVelocity(speedSetpoint).withFeedForward(ffVolts));
    }

    @Override
    public double getSetpoint() {
        return talonFX.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public boolean isAtTarget() {
        return isAtTarget(DEFAULT_TOLERANCE);
    }

    @Override
    public boolean isAtTarget(double tolerance) {
        return Math.abs(getVelocity() - getSetpoint()) < tolerance;
    }

    @Override
    public void stop() {
        talonFX.setControl(neutralRequest);
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kV = kV;
        configs.Slot0.kA = kA;
        talonFX.getConfigurator().apply(configs.Slot0);
    }

}
