package frc.robot.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.IMotorPositionControl;

public class PositionGainsTuner {

    private final IMotorPositionControl motor;

    private double kP, kI, kD, kS, kV, kA, kG, kCos;
    private double maxVelocity, maxAcceleration, allowedProfileError;
    private double testSetpoint;

    // Pre-computed keys to avoid string concatenation in the 20ms loop
    private final String keyKP, keyKI, keyKD, keyKS, keyKV, keyKA, keyKG, keyKCos;
    private final String keyMaxVelocity, keyMaxAcceleration, keyAllowedProfileError;
    private final String keyIsTuning, keyTestSetpoint;
    private final String keyPosition, keyCurrentSetpoint;

    public PositionGainsTuner(String prefix, IMotorPositionControl motor,
            double kP, double kI, double kD, double kS, double kV, double kA, double kG, double kCos,
            double maxVelocity, double maxAcceleration, double allowedProfileError) {
        this.motor = motor;
        this.kP = kP; this.kI = kI; this.kD = kD;
        this.kS = kS; this.kV = kV; this.kA = kA;
        this.kG = kG; this.kCos = kCos;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.allowedProfileError = allowedProfileError;

        String gains       = prefix + "/gains/";
        String constraints = prefix + "/constraints/";
        String control     = prefix + "/control/";
        String output      = prefix + "/output/";

        keyKP   = gains + "kP";
        keyKI   = gains + "kI";
        keyKD   = gains + "kD";
        keyKS   = gains + "kS";
        keyKV   = gains + "kV";
        keyKA   = gains + "kA";
        keyKG   = gains + "kG";
        keyKCos = gains + "kCos";
        keyMaxVelocity        = constraints + "maxVelocity";
        keyMaxAcceleration    = constraints + "maxAcceleration";
        keyAllowedProfileError = constraints + "allowedProfileError";
        keyIsTuning     = control + "isTuning";
        keyTestSetpoint = control + "testSetpoint";
        keyPosition        = output + "position";
        keyCurrentSetpoint = output + "currentSetpoint";

        SmartDashboard.putBoolean(keyIsTuning, false);
        SmartDashboard.putNumber(keyTestSetpoint, testSetpoint);
        SmartDashboard.putNumber(keyKP,   kP);
        SmartDashboard.putNumber(keyKI,   kI);
        SmartDashboard.putNumber(keyKD,   kD);
        SmartDashboard.putNumber(keyKS,   kS);
        SmartDashboard.putNumber(keyKV,   kV);
        SmartDashboard.putNumber(keyKA,   kA);
        SmartDashboard.putNumber(keyKG,   kG);
        SmartDashboard.putNumber(keyKCos, kCos);
        SmartDashboard.putNumber(keyMaxVelocity,        maxVelocity);
        SmartDashboard.putNumber(keyMaxAcceleration,    maxAcceleration);
        SmartDashboard.putNumber(keyAllowedProfileError, allowedProfileError);
    }

    public void update() {
        double newKP   = SmartDashboard.getNumber(keyKP,   kP);
        double newKI   = SmartDashboard.getNumber(keyKI,   kI);
        double newKD   = SmartDashboard.getNumber(keyKD,   kD);
        double newKS   = SmartDashboard.getNumber(keyKS,   kS);
        double newKV   = SmartDashboard.getNumber(keyKV,   kV);
        double newKA   = SmartDashboard.getNumber(keyKA,   kA);
        double newKG   = SmartDashboard.getNumber(keyKG,   kG);
        double newKCos = SmartDashboard.getNumber(keyKCos, kCos);

        if (newKP != kP || newKI != kI || newKD != kD || newKS != kS || newKV != kV || newKA != kA
                || newKG != kG || newKCos != kCos) {
            kP = newKP; kI = newKI; kD = newKD;
            kS = newKS; kV = newKV; kA = newKA;
            kG = newKG; kCos = newKCos;
            motor.updateGains(kP, kI, kD, kS, kV, kA, kG, kCos);
        }

        double newMaxVelocity        = SmartDashboard.getNumber(keyMaxVelocity,        maxVelocity);
        double newMaxAcceleration    = SmartDashboard.getNumber(keyMaxAcceleration,    maxAcceleration);
        double newAllowedProfileError = SmartDashboard.getNumber(keyAllowedProfileError, allowedProfileError);

        if (newMaxVelocity != maxVelocity || newMaxAcceleration != maxAcceleration
                || newAllowedProfileError != allowedProfileError) {
            maxVelocity         = newMaxVelocity;
            maxAcceleration     = newMaxAcceleration;
            allowedProfileError = newAllowedProfileError;
            motor.updateMotionConstraints(maxVelocity, maxAcceleration, allowedProfileError);
        }

        boolean isTuning = SmartDashboard.getBoolean(keyIsTuning, false);
        if (isTuning) {
            testSetpoint = SmartDashboard.getNumber(keyTestSetpoint, testSetpoint);
            motor.runPosition(testSetpoint);
            SmartDashboard.putNumber(keyPosition,        motor.getPosition());
            SmartDashboard.putNumber(keyCurrentSetpoint, motor.getSetpoint());
        }
    }
}
