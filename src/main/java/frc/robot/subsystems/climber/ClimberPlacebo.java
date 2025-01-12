package frc.robot.subsystems.climber;

public class ClimberPlacebo implements ClimberIO{
    
    public double getHeight(){
        return 0.0;
    }

    public void setHeight(double height){}

    public void extend(){}

    public void retract(){}

    public boolean isExtended(){
        return false;
    }

    public boolean isRetracted(){
        return false; 
    }

}
