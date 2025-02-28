package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;

public class ClimberPlacebo implements ClimberIO{
    
    public void setGoalAngle(Angle desiredAngle){}

    public double getAngle(){
        return 0.0; 
    }

    public void setSpeed(double speed){}

    public double getVelocity(){
        return 0.0; 
    }

    public void setServoAngle(Angle desiredAngle){

    }

    public void updateStates(ClimberIOInputs inputs){}


}
