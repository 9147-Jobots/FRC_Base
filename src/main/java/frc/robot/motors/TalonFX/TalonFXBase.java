package frc.robot.motors.TalonFX;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.motors.IMotorBase;

public abstract class TalonFXBase implements IMotorBase {
    
    protected final TalonFX talonFX;

    protected TalonFXBase(TalonFX talonFX) {
        this.talonFX = talonFX;
    }

    @Override
    public double getCurrent() {
        return talonFX.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return talonFX.getDeviceTemp().getValueAsDouble();
    }
 
    @Override
    public double getVoltage() {
        return talonFX.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void runVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    }

    @Override
    public void updateMotionConstraints(double maxVelocity, double maxAcceleration, double allowedProfileError) {
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = maxVelocity;
        mm.MotionMagicAcceleration = maxAcceleration;
        talonFX.getConfigurator().apply(mm);
    }

    @Override
    public void setIdleMode(boolean brake) {
        talonFX.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        talonFX.getConfigurator().refresh(motorOutput);
        motorOutput.PeakForwardDutyCycle = maxOutput;
        motorOutput.PeakReverseDutyCycle = minOutput;
        talonFX.getConfigurator().apply(motorOutput);
    }
}
