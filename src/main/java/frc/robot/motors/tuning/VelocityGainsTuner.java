package frc.robot.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.IMotorVelocityControl;

public class VelocityGainsTuner {

    public static class Builder {
        private final String prefix;
        private final IMotorVelocityControl motor;

        private double kP = 0, kI = 0, kD = 0;
        private double kS = 0, kV = 0, kA = 0;
        private double maxVelocity = 0, maxAcceleration = 0, allowedProfileError = 0;
        private double minOutput = -1.0, maxOutput = 1.0;

        public Builder(String prefix, IMotorVelocityControl motor) {
            this.prefix = prefix;
            this.motor = motor;
        }

        public Builder kP(double kP)           { this.kP = kP;                           return this; }
        public Builder kI(double kI)           { this.kI = kI;                           return this; }
        public Builder kD(double kD)           { this.kD = kD;                           return this; }
        public Builder kS(double kS)           { this.kS = kS;                           return this; }
        public Builder kV(double kV)           { this.kV = kV;                           return this; }
        public Builder kA(double kA)           { this.kA = kA;                           return this; }
        public Builder maxVelocity(double v)   { this.maxVelocity = v;                   return this; }
        public Builder maxAcceleration(double a) { this.maxAcceleration = a;             return this; }
        public Builder allowedProfileError(double e) { this.allowedProfileError = e;     return this; }
        public Builder minOutput(double min)   { this.minOutput = min;                   return this; }
        public Builder maxOutput(double max)   { this.maxOutput = max;                   return this; }

        public VelocityGainsTuner build() {
            return new VelocityGainsTuner(this);
        }
    }

    private final IMotorVelocityControl motor;

    private double kP, kI, kD, kS, kV, kA;
    private double maxVelocity, maxAcceleration, allowedProfileError;
    private double minOutput, maxOutput;
    private double testSetpoint;
    private boolean brakeMode;

    // Pre-computed keys to avoid string concatenation in the 20ms loop
    private final String keyKP, keyKI, keyKD, keyKS, keyKV, keyKA;
    private final String keyMaxVelocity, keyMaxAcceleration, keyAllowedProfileError;
    private final String keyMinOutput, keyMaxOutput;
    private final String keyIsTuning, keyTestSetpoint, keyBrakeMode;
    private final String keyVelocity, keyCurrentSetpoint;

    private VelocityGainsTuner(Builder b) {
        this.motor = b.motor;
        this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
        this.kS = b.kS; this.kV = b.kV; this.kA = b.kA;
        this.maxVelocity = b.maxVelocity;
        this.maxAcceleration = b.maxAcceleration;
        this.allowedProfileError = b.allowedProfileError;
        this.minOutput = b.minOutput;
        this.maxOutput = b.maxOutput;
        this.brakeMode = true;

        String gains       = b.prefix + "/gains/";
        String constraints = b.prefix + "/constraints/";
        String control     = b.prefix + "/control/";
        String output      = b.prefix + "/output/";

        keyKP = gains + "kP";
        keyKI = gains + "kI";
        keyKD = gains + "kD";
        keyKS = gains + "kS";
        keyKV = gains + "kV";
        keyKA = gains + "kA";
        keyMaxVelocity         = constraints + "maxVelocity";
        keyMaxAcceleration     = constraints + "maxAcceleration";
        keyAllowedProfileError = constraints + "allowedProfileError";
        keyMinOutput           = constraints + "minOutput";
        keyMaxOutput           = constraints + "maxOutput";
        keyIsTuning     = control + "isTuning";
        keyTestSetpoint = control + "testSetpoint";
        keyBrakeMode    = control + "brakeMode";
        keyVelocity        = output + "velocity";
        keyCurrentSetpoint = output + "currentSetpoint";

        SmartDashboard.putBoolean(keyIsTuning, false);
        SmartDashboard.putBoolean(keyBrakeMode, brakeMode);
        SmartDashboard.putNumber(keyTestSetpoint, testSetpoint);
        SmartDashboard.putNumber(keyKP, kP);
        SmartDashboard.putNumber(keyKI, kI);
        SmartDashboard.putNumber(keyKD, kD);
        SmartDashboard.putNumber(keyKS, kS);
        SmartDashboard.putNumber(keyKV, kV);
        SmartDashboard.putNumber(keyKA, kA);
        SmartDashboard.putNumber(keyMaxVelocity,        maxVelocity);
        SmartDashboard.putNumber(keyMaxAcceleration,    maxAcceleration);
        SmartDashboard.putNumber(keyAllowedProfileError, allowedProfileError);
        SmartDashboard.putNumber(keyMinOutput, minOutput);
        SmartDashboard.putNumber(keyMaxOutput, maxOutput);
    }

    public void update() {
        double newKP = SmartDashboard.getNumber(keyKP, kP);
        double newKI = SmartDashboard.getNumber(keyKI, kI);
        double newKD = SmartDashboard.getNumber(keyKD, kD);
        double newKS = SmartDashboard.getNumber(keyKS, kS);
        double newKV = SmartDashboard.getNumber(keyKV, kV);
        double newKA = SmartDashboard.getNumber(keyKA, kA);

        if (newKP != kP || newKI != kI || newKD != kD || newKS != kS || newKV != kV || newKA != kA) {
            kP = newKP; kI = newKI; kD = newKD;
            kS = newKS; kV = newKV; kA = newKA;
            motor.updateGains(kP, kI, kD, kS, kV, kA);
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

        double newMinOutput = SmartDashboard.getNumber(keyMinOutput, minOutput);
        double newMaxOutput = SmartDashboard.getNumber(keyMaxOutput, maxOutput);
        if (newMinOutput != minOutput || newMaxOutput != maxOutput) {
            minOutput = newMinOutput;
            maxOutput = newMaxOutput;
            motor.setOutputRange(minOutput, maxOutput);
        }

        boolean newBrakeMode = SmartDashboard.getBoolean(keyBrakeMode, brakeMode);
        if (newBrakeMode != brakeMode) {
            brakeMode = newBrakeMode;
            motor.setIdleMode(brakeMode);
        }

        boolean isTuning = SmartDashboard.getBoolean(keyIsTuning, false);
        if (isTuning) {
            testSetpoint = SmartDashboard.getNumber(keyTestSetpoint, testSetpoint);
            motor.runVelocity(testSetpoint);
            SmartDashboard.putNumber(keyVelocity,        motor.getVelocity());
            SmartDashboard.putNumber(keyCurrentSetpoint, motor.getSetpoint());
        }
    }
}
