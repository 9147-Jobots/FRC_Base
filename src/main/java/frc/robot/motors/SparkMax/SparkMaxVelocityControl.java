package frc.robot.motors.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.motors.IMotorVelocityControl;

import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxVelocityControl extends SparkMaxBase implements IMotorVelocityControl {

    private final SparkMaxConfig config;;

    private final RelativeEncoder encoder;

    private final SparkClosedLoopController pid;

    private static final double DEFAULT_TOLERANCE = 0.05;

    /**
     * Creates a new Spark Max velocity controller with constants P, S, V and A.
     * @param deviceID the CAN ID of the Spark Max
     * @param type the type of motor being used
     * @param isInverted whether the motor output should be inverted
     * @param positionConversionFactor the factor to convert the motor's native units to the desired position units
     * @param velocityConversionFactor the factor to convert the motor's native units to the desired velocity units
     * @param kCruiseVelocity the cruise velocity for motion profiling
     * @param kMaxAcceleration the maximum acceleration for motion profiling
     * @param kAllowedProfileError the allowed error for the motion profile
     * @param kP the proportional gain for the PID controller
     * @param kS the static gain for the feedforward controller
     * @param kV the velocity gain for the feedforward controller
     * @param kA the acceleration gain for the feedforward controller
     * @return
     */
    public static IMotorVelocityControl CreateSparkMaxVelocityController(
        int deviceID,
        MotorType type,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor,
        double kCruiseVelocity,
        double kMaxAcceleration,
        double kAllowedProfileError,
        double kP,
        double kS,
        double kV,
        double kA
    ) {
        return new SparkMaxVelocityControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, kP,0, 0, kS, kV, kA);
    }


    /**
     * DO NOT USE THIS UNLESS YOU KNOW WHAT ALL OF THESE PARAMETERS DO AND WHY YOU NEED TO SPECIFY ALL OF THEM.
     * Note: This should only be used on rare occasions when ou need to specify all parameters. It is recommended to use one of the other CreateSparkMaxPositionController methods instead, as they have more reasonable defaults for the parameters that are not specified.
     * @param deviceID the CAN ID of the Spark Max
     * @param isInverted whether the motor output should be inverted
     * @param positionConversionFactor the factor to convert the motor's native units to the desired position units
     * @param velocityConversionFactor the factor to convert the motor's native units to the desired velocity units
     * @param kCruiseVelocity the cruise velocity for motion profiling
     * @param kMaxAcceleration the maximum acceleration for motion profiling
     * @param kAllowedProfileError the allowed error for the motion profile
     * @param KP the proportional gain for the PID controller
     * @param kI the integral gain for the PID controller
     * @param kD the derivative gain for the PID controller
     * @param kS the static gain for the feedforward controller
     * @param kV the velocity gain for the feedforward controller
     * @param kA the acceleration gain for the feedforward controller
     * @return
     */
    public static IMotorVelocityControl OverloadCreateSparkMaxVelocityController(
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
        double kA
    ) {
        return new SparkMaxVelocityControl(deviceID, type, isInverted, positionConversionFactor, velocityConversionFactor, kCruiseVelocity, kMaxAcceleration, kAllowedProfileError, kP, kI, kD, kS, kV, kA);
    }

    /**
     * 
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
     */
    private SparkMaxVelocityControl(int deviceID,
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
        double kA
        ) {
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
                .kV(kV)
                .kA(kA);
                
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
    public void runVelocity(double setpoint) {
        pid.setSetpoint(setpoint, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void runVelocity(double setpoint, double ffVolts) {
        pid.setSetpoint(setpoint, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, ffVolts);
    }

    @Override
    public double getSetpoint() {
        return pid.getSetpoint();
    }

    @Override
    public boolean isAtTarget() {
        return isAtTarget(DEFAULT_TOLERANCE);
    }

    @Override
    public boolean isAtTarget(double tolerance) {
        return Math.abs(getVelocity() - getSetpoint()) <= tolerance;
    }

    @Override
    public void stop() {
        sparkMax.stopMotor();
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        SparkMaxConfig update = new SparkMaxConfig();
        update.closedLoop.p(kP).i(kI).d(kD).feedForward.kS(kS).kV(kV).kA(kA);
        sparkMax.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
