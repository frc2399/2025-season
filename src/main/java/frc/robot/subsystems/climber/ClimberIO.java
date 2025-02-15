package frc.robot.subsystems.climber;

public interface ClimberIO {
    
    public double getAngle();
    public void setEncoderAngle(double Angle); 
    public void setGoalAngle(double desiredPosition);
    public void setSpeed(double speed);
    public void getEncoderVelocity();
    public void getEncoderPosition(); 
    public void updateStates(); 
}
