package frc.robot.motors;

public interface IMotorBase {

    /**
     * Returns the current velocity of the motor.
     * @return the current velocity of the motor
     */
    public double getVelocity();
        
    /**
     * Returns the current position of the motor.
     * @return the current position of the motor
     */
    public double getPosition();

    /**
     * Returns the current drawn by the motor in amps.
     * @return the current drawn by the motor in amps
     */
    public double getCurrent();

    /**
     * Returns the temperature of the motor in degrees Celsius.
     * @return the temperature of the motor in degrees Celsius
     */
    public double getTemperature();


    /**
     * runs the motor at a voltage.
     */
    public void runVoltage(double voltage);

    /**
     * Returns the voltage supplied to the motor in volts.
     * @return the voltage supplied to the motor in volts
     */
    public double getVoltage();

    /**
     * Updates the motion profile constraints at runtime without redeploying code.
     * @param maxVelocity maximum cruise velocity for the motion profile
     * @param maxAcceleration maximum acceleration for the motion profile
     * @param allowedProfileError allowed error for the motion profile (SparkMax/MAXMotion only; ignored by TalonFX)
     */
    public void updateMotionConstraints(double maxVelocity, double maxAcceleration, double allowedProfileError);
}
