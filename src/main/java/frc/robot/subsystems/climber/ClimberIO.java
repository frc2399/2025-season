package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {

    static class ClimberIOInputs {
        double climberAngle = 0.0;
        double climberGoalAngle = 0.0;
        double climberVelocity = 0.0; 
        double servoAngle = 0.0;
        double servoGoalAngle = 0.0;
        double servoVelocity = 0.0;
    }
    
    public void setGoalAngle(Angle desiredAngle);
    public void setServoAngle(Angle desiredAngle); 
    public void setSpeed(double speed);
    public void updateStates(ClimberIOInputs inputs); 
}
