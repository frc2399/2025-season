package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimberPlacebo implements ClimberIO{
    
    public void setGoalAngle(Angle desiredAngle){}

    public double getAngle(){
        return 0.0; 
    }

    public void setSpeed(LinearVelocity speed){}

    public double getVelocity(){
        return 0.0; 
    }

    public void updateStates(ClimberIOInputs inputs){}


}
