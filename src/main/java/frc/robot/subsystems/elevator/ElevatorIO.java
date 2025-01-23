package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    static class ElevatorIOInputs {
        public double position = 0.0; 
        public double velocity = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double positionSetPoint = 0.0;
    }

    public void setSpeed(double speed);
    public void setPosition(double position);
    public void setEncoderPosition(double position);
    public double getVelocity();
    public double getPosition();
    public void setPercentOutput(double percentOutput);
    public void updateStates(ElevatorIOInputs inputs);
}