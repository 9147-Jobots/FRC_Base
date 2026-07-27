package frc.robot.motors.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.motors.IMotorPositionControl;

public class TalonFXPositionControl extends TalonFXBase implements IMotorPositionControl {

    private final TalonFXConfiguration configs;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private final static double DEFAULT_TOLERANCE = 0.05;

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, I, D, S.
     * @param deviceID
     * CAN ID of the TalonFX motor controller.
     * @param isInverted
     * Whether the motor output should be inverted.
     * @param positionConversionFactor
     * Conversion factor from sensor units to desired output units.
     * @param kP
     * Proportional gain.
     * @param kI
     * Integral gain.
     * @param kD
     * Derivative gain.
     * @param kS
     * Static gain.
     * @return
     */
    public static IMotorPositionControl CreateTalonFXPositionControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS
    ) {
        return new TalonFXPositionControl(deviceID, isInverted, positionConversionFactor, kP, kI, kD, kS, 0, 0);
    }

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, I, D, S and G, including gravity compensation.
     * @param deviceID
     * CAN ID of the TalonFX motor controller.
     * @param isInverted
     * Whether the motor output should be inverted.
     * @param positionConversionFactor
     * Conversion factor from sensor units to desired output units.
     * @param kP
     * Proportional gain.
     * @param kI
     * Integral gain.
     * @param kD
     * Derivative gain.
     * @param kS
     * Static gain.
     * @param kG
     * Gravity compensation gain.
     * @return
     */
    public static IMotorPositionControl CreateLinearFFTalonFXPositionControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS,
        double kG
    ) {
        return new TalonFXPositionControl(deviceID, isInverted, positionConversionFactor, kP, kI, kD, kS, kG, 0);
    }

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, I, D, S and kCosG, including gravity compensation for pivoting arm.
     * @param deviceID
     * CAN ID of the TalonFX motor controller.
     * @param isInverted
     * Whether the motor output should be inverted.
     * @param positionConversionFactor
     * Conversion factor from sensor units to desired output units.
     * @param kP
     * Proportional gain.
     * @param kI
     * Integral gain.
     * @param kD
     * Derivative gain.
     * @param kS
     * Static gain.
     * @param kCos
     * Cosine gain for gravity compensation.
     * @return
     */
    public static IMotorPositionControl CreatePivotFFTalonFXPositionControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS,
        double kCos
    ) {
        return new TalonFXPositionControl(deviceID, isInverted, positionConversionFactor, kP, kI, kD, kS, 0, kCos);
    }

    /**
     * 
     * @param deviceID
     * @param isInverted
     * @param positionConversionFactor
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kG
     * @param kCos
     */
    private TalonFXPositionControl(
        int deviceID,
        boolean isInverted,
        double positionConversionFactor,
        double kP,
        double kI,
        double kD,
        double kS,
        double kG,
        double kCos
    ) {
        super(new TalonFX(deviceID));
        configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        if (kG != 0) {
            configs.Slot0.kG = kG;
            configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        } else {
            configs.Slot0.kG = kCos;
            configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        }

        configs.Feedback.SensorToMechanismRatio = positionConversionFactor;
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
    public void runPosition(double setpoint) {
        talonFX.setControl(positionRequest.withPosition(setpoint));
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
        return Math.abs(getPosition() - getSetpoint()) < tolerance;
    }

    @Override
    public void zeroPosition(double position) {
        talonFX.setPosition(position);
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double kCos) {
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kV = kV;
        configs.Slot0.kA = kA;
        // kG and kCos share the same Slot0.kG field — apply the one matching the gravity type set at construction
        configs.Slot0.kG = (configs.Slot0.GravityType == GravityTypeValue.Elevator_Static) ? kG : kCos;
        talonFX.getConfigurator().apply(configs.Slot0);
    }

}
