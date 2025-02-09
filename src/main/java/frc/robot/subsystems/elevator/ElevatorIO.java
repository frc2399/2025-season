package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    static class ElevatorIOInputs {
        public double position = 0.0; 
        public double velocity = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double positionSetPoint = 0.0;
        public double goalStatePosition = 0.0;
        public double setpointStatePosition = 0.0;
        public double current = 0.0;
        
    }

    public void disableElevator();
    public void enableElevator(); 
    public void incrementGoalPosition(double newGoalPosition);
    public void setGoalPosition(double position);
    public void calculateNextSetpoint();
    public void setEncoderPosition(double position);
    public void setSetpointState(double position, double velocity);
    public double getEncoderVelocity();
    public double getEncoderPosition();
    public void updateStates(ElevatorIOInputs inputs);
}