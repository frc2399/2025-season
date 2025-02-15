package frc.robot.subsystems.climber;

public interface ClimberIO {

    static class ClimberIOInputs {
        double position = 0.0;
        double goalPosition = 0.0;
        double velocity = 0.0; 
    }
    
    public void setAngle(double angle); 
    public void setGoalAngle(double desiredPosition);
    public double getAngle();
    public void setSpeed(double speed);
    public double getVelocity();
    public void updateStates(ClimberIOInputs inputs); 
}
