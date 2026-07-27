package frc.robot.motors;

public interface IMotorVelocityControl extends IMotorBase {
    
    /**
     * Runs the motor at the specified velocity setpoint.
     * @param setpoint the velocity setpoint to run the motor at
     */
    public void runVelocity(double setpoint);

    /**
     * NOTE DO NOT USE THIS UNLESS IT IS ABSOLUTELY NECESSARY, instead set ffvolts in the pid.
     * Runs the motor at the specified velocity setpoint, using the specified feedforward calculation to determine the necessary voltage to achieve that velocity.
     * @param speedSetpoint the velocity setpoint to run the motor at
     * @param ffVolts the feedforward calculation to determine the necessary voltage
     */
    public void runVelocity(double speedSetpoint, double ffVolts);

    /**
     * Returns the current setpoint of the motor. This is not necessarily the same as the current velocity of the motor, as the motor may not be at the setpoint yet.
     * @return the current setpoint of the motor
     */
    public double getSetpoint();

    /**
     * Returns whether the motor is at its target velocity.
     * @return true if the motor is at its target velocity, false otherwise
     */
    public boolean isAtTarget();
    
    /**
     * Returns whether the motor is within the specified tolerance of its target velocity.
     * @param tolerance the tolerance to check against
     * @return true if the motor is within the specified tolerance of its target velocity, false otherwise
     */
    public boolean isAtTarget(double tolerance);

    /**
     * Stops the motor.
     */
    public void stop();

    /**
     * Updates the PID and feedforward gains at runtime without redeploying code.
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kS static feedforward gain (overcomes friction)
     * @param kV velocity feedforward gain
     * @param kA acceleration feedforward gain
     */
    public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA);

}
