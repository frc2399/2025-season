package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    static class ElevatorIOInputs {
        public double position = 0.0; 
        public double velocity = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double positionSetPoint = 0.0;
        public double current = 0.0;
    }

    public void disableElevator();
    public void setSpeed(double speed);
    public void setGoalPositionPID(double position);
    public void setPositionMotionProfiling(double position);
    public void calculateNextSetpoint();
    public void setSetpointState(double position, double velocity);
    public void setEncoderPosition(double position);
    public double getEncoderVelocity();
    public double getEncoderPosition();
    public void setPercentOutput(double percentOutput);
    public void updateStates(ElevatorIOInputs inputs);
}