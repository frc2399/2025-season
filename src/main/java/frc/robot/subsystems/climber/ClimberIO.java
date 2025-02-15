package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {

    static class ClimberIOInputs {
        double angle = 0.0;
        double goalAngle = 0.0;
        double velocity = 0.0; 
    }
    
    public void setGoalAngle(Angle desiredAngle);
    public double getAngle();
    public void setSpeed(double speed);
    public double getVelocity();
    public void updateStates(ClimberIOInputs inputs); 
}
