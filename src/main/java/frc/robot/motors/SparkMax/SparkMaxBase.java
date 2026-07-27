package frc.robot.motors.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.motors.IMotorBase;

public abstract class SparkMaxBase implements IMotorBase {

    // Expose the underlying SparkMax to subclasses for configuration and use.
    // Kept protected so subclasses (e.g., position/velocity controllers) can
    // reuse the same hardware instance instead of creating duplicates.
    protected final SparkMax sparkMax;

    public SparkMaxBase(SparkMax sparkMax) {
        this.sparkMax = sparkMax;
    }

    @Override
    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return sparkMax.getMotorTemperature();
    }

    @Override
    public double getVoltage() {
        return sparkMax.getBusVoltage();
    }

    @Override
    public void runVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    @Override
    public void updateMotionConstraints(double maxVelocity, double maxAcceleration, double allowedProfileError) {
        SparkMaxConfig update = new SparkMaxConfig();
        update.closedLoop.maxMotion.cruiseVelocity(maxVelocity).maxAcceleration(maxAcceleration).allowedProfileError(allowedProfileError);
        sparkMax.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setIdleMode(boolean brake) {
        SparkMaxConfig update = new SparkMaxConfig();
        update.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        sparkMax.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        SparkMaxConfig update = new SparkMaxConfig();
        update.closedLoop.outputRange(minOutput, maxOutput);
        sparkMax.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}

