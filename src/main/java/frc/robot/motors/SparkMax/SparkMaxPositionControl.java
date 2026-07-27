package frc.robot.motors.SparkMax;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.motors.IMotorPositionControl;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class SparkMaxPositionControl extends SparkMaxBase implements IMotorPositionControl {

    private final SparkMaxConfig config;;

    private final RelativeEncoder encoder;

    private final SparkClosedLoopController pid;

    private static final double DEFAULT_TOLERANCE = 0.05; // Default tolerance for isAtTarget, can be overridden by isAtTarget(double tolerance)

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, S, V and A.
     * @param deviceID
     * The CAN ID of the Spark Max motor controller: This should be a unique ID assigned to the motor controller on the CAN bus, typically set using a hardware configuration tool or through code
     * @param isInverted
     * Whether the motor output should be inverted: Setting this to true will reverse the direction of the motor, which can be useful for certain mechanisms or to correct for wiring issues
     * @param positionConversionFactor
     * The factor to convert the motor's native units to the desired position units: Increasing it will make the controller more sensitive to position changes, but too high may cause instability
     * @param velocityConversionFactor
     * The factor to convert the motor's native units to the desired velocity units: Increasing it will make the controller more sensitive to velocity changes, but too high may cause instability
     * @param kCruiseVelocity
     * The cruise velocity for motion profiling: Increasing it will allow the controller to reach higher speeds, but too high may cause overshooting
     * @param kMaxAcceleration
     * The maximum acceleration for motion profiling: Increasing it will allow the controller to accelerate faster, but too high may cause instability
     * @param kAllowedProfileError
     * The allowed error for the motion profile: Increasing it will make the controller more tolerant to error, but too high may cause overshooting
     * @param KP
     * The proportional gain for the PID controller: Increasing it will increase the responsiveness of the controller, but too high may cause oscillation
     * @param kS
     * The static gain for the feedforward controller: helps overcome a constant resistance like friction in a gearbox
     * @param kV
     * The velocity gain for the feedforward controller: Increasing it will increase the voltage output proportionally to the velocity target
     * @param kA
     * The acceleration gain for the feedforward controller: Increasing it will help track acceleration more closely
     * @return
     */
    public static IMotorPositionControl CreateLinearSparkMaxPositionController(
        int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double KP,
        double kS,
        double kV,
        double kA
        ) {
        return new SparkMaxPositionControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, KP, 0, 0, kS, kV, kA, 0, 0);
    }

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, S, V, A and G, including gravity compensation.
     * @param deviceID
     * The CAN ID of the Spark Max motor controller: This should be a unique ID assigned to the motor controller on the CAN bus, typically set using a hardware configuration tool or through code
     * @param type
     * The type of motor being used: This should be set to the appropriate motor type (e.g. brushless or brushed) to ensure proper control and feedback from the motor controller
     * @param isInverted
     * Whether the motor output should be inverted: Setting this to true will reverse the direction of the motor, which can be useful for certain mechanisms or to correct for wiring issues
     * @param positionConversionFactor
     * The factor to convert the motor's native units to the desired position units: Increasing it will make the controller more sensitive to position changes, but too high may cause instability
     * @param velocityConversionFactor
     * The factor to convert the motor's native units to the desired velocity units: Increasing it will make the controller more sensitive to velocity changes, but too high may cause instability
     * @param kCruiseVelocity
     * The cruise velocity for motion profiling: Increasing it will allow the controller to reach higher speeds, but too high may cause overshooting
     * @param kMaxAcceleration
     * The maximum acceleration for motion profiling: Increasing it will allow the controller to accelerate faster, but too high may cause instability
     * @param kAllowedProfileError
     * The allowed error for the motion profile: Increasing it will make the controller more tolerant to error, but too high may cause overshooting
     * @param KP
     * The proportional gain for the PID controller: Increasing it will increase the responsiveness of the controller, but too high may cause oscillation
     * @param kS
     * The static gain for the feedforward controller: helps overcome a constant resistance like friction in a gearbox
     * @param kV
     * The velocity gain for the feedforward controller: Increasing it will increase the voltage output proportionally to the velocity target
     * @param kA
     * The acceleration gain for the feedforward controller: Increasing it will help track acceleration more closely
     * @param kG
     * The gravity gain for the feedforward controller: helps compensate for gravity when moving vertically, increasing it will help hold position against gravity better but too high may cause overshooting when moving downwards
     */
    public static IMotorPositionControl CreateLinearFFSparkMaxPositionController(
        int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double KP,
        double kS,
        double kV,
        double kA,
        double kG
    ) {
        return new SparkMaxPositionControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, KP, 0, 0, kS, kV, kA, kG, 0);
    }

    /**
     * Creates a new PositionControlMotorSparkMax with constants P, S, V, A and kCosG, including gravity compensation for pivoting arm.
     * @param deviceID
     * The CAN ID of the Spark Max motor controller: This should be a unique ID assigned to the motor controller on the CAN bus, typically set using a hardware configuration tool or through code
     * @param type
     * The type of motor being used: This should be set to the appropriate motor type (e.g. brushless or brushed) to ensure proper control and feedback from the motor controller
     * @param isInverted
     * Whether the motor output should be inverted: Setting this to true will reverse the direction of the motor, which can be useful for certain mechanisms or to correct for wiring issues
     * @param positionConversionFactor
     * The factor to convert the motor's native units to the desired position units: Increasing it will make the controller more sensitive to position changes, but too high may cause instability
     * @param velocityConversionFactor
     * The factor to convert the motor's native units to the desired velocity units: Increasing it will make the controller more sensitive to velocity changes, but too high may cause instability
     * @param kCruiseVelocity
     * The cruise velocity for motion profiling: Increasing it will allow the controller to reach higher speeds, but too high may cause overshooting
     * @param kMaxAcceleration
     * The maximum acceleration for motion profiling: Increasing it will allow the controller to accelerate faster, but too high may cause instability
     * @param kAllowedProfileError
     * The allowed error for the motion profile: Increasing it will make the controller more tolerant to error, but too high may cause overshooting
     * @param KP
     * The proportional gain for the PID controller: Increasing it will increase the responsiveness of the controller, but too high may cause oscillation
     * @param kS
     * The static gain for the feedforward controller: helps overcome a constant resistance like friction in a gearbox
     * @param kV
     * The velocity gain for the feedforward controller: Increasing it will increase the voltage output proportionally to the velocity target
     * @param kA
     * The acceleration gain for the feedforward controller: Increasing it will help track acceleration more closely
     * @param kCos
     * The cosine gain for gravity compensation based on position: helps reduce overshooting when moving downwards by reducing gravity compensation as you approach vertical, increasing it will help reduce overshooting but too high may cause instability near vertical positions
     */
    public static IMotorPositionControl CreatePivotFFSparkMaxPositionController(
        int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double KP,
        double kS,
        double kV,
        double kA,
        double kCos
    ) {
        return new SparkMaxPositionControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, KP, 0, 0, kS, kV, kA, 0, kCos);
    }

    /**
     * DO NOT USE THIS UNLESS YOU KNOW WHAT ALL OF THESE PARAMETERS DO AND WHY YOU NEED TO SPECIFY ALL OF THEM.
     * Note: This should only be used on rare occasions when ou need to specify all parameters. It is recommended to use one of the other CreateSparkMaxPositionController methods instead, as they have more reasonable defaults for the parameters that are not specified.
     * @param deviceID
     * @param isInverted
     * @param positionConversionFactor
     * @param velocityConversionFactor
     * @param kCruiseVelocity
     * @param kMaxAcceleration
     * @param kAllowedProfileError
     * @param KP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     * @param kA
     * @param kG
     * @param kCos
     * @return
     */
    public static IMotorPositionControl OverloadCreateSparkMaxPositionController(
        int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        double kCos
    ) {
        return new SparkMaxPositionControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, kP, kI, kD, kS, kV, kA, kG, kCos);
    }

    /**
     * 
     * @param deviceID
     * @param isInverted
     * @param positionConversionFactor
     * @param kCruiseVelocity
     * @param kMaxAcceleration
     * @param kAllowedProfileError
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     * @param kA
     * @param kG
     * @param kCos
     */
    private SparkMaxPositionControl(int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG,
        double kCos
        ) {
    // Construct the wrapped SparkMax in the superclass and reuse it here.
    super(new SparkMax(deviceID, type));
    config = new SparkMaxConfig();
        config.inverted(isInverted).closedLoop
            .maxMotion
                .allowedProfileError(kAllowedProfileError)
                .maxAcceleration(kMaxAcceleration)
                .cruiseVelocity(kCruiseVelocity)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        config.
        closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .feedForward
                .kS(kS)
                .kV(kV)
                .kA(kA)
                .kG(kG)
                .kCos(kCos);
                
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid = sparkMax.getClosedLoopController();
        encoder = sparkMax.getEncoder();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void runPosition(double setpoint) {
        pid.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
        
    }

    @Override
    public double getSetpoint() {
        return pid.getSetpoint();
    }

    @Override
    public boolean isAtTarget() {
        return Math.abs(pid.getSetpoint() - encoder.getPosition()) < DEFAULT_TOLERANCE;
    }

    @Override
    public boolean isAtTarget(double tolerance) {
        return Math.abs(pid.getSetpoint() - encoder.getPosition()) < tolerance;
    }

    @Override
    public void zeroPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double kCos) {
        SparkMaxConfig update = new SparkMaxConfig();
        update.closedLoop.p(kP).i(kI).d(kD).feedForward.kS(kS).kV(kV).kA(kA).kG(kG).kCos(kCos);
        sparkMax.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
