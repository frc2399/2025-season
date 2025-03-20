package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ClimberIO {

    static class ClimberIOInputs {
        double climberAngle = 0.0;
        double climberGoalAngle = 0.0;
        double climberVelocity = 0.0; 
        double servoAngle = 0.0;
        double servoGoalAngle = 0.0;
        double servoVelocity = 0.0;
    }
    
    public void setSpeed(LinearVelocity speed);
    public void updateStates(ClimberIOInputs inputs); 
}
