package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberHardware implements ClimberIO {
    final TalonFX climber;
    final TalonFXConfiguration climberConfiguration;

    //TODO: tune min and max positions 
    public static final double MAX_POSITION = 0; 
    public static final double MIN_POSITION = 0;

    public ClimberHardware(){
        climber = new TalonFX(0);

        //creates a configuration for the climber that is set to the factory defaults 
        climberConfiguration = new TalonFXConfiguration();

        //TODO: tune PID 
        climberConfiguration.Slot0.kP = 0;
        climberConfiguration.Slot0.kI = 0;
        climberConfiguration.Slot0.kD = 0;
        climberConfiguration.Slot0.kV = 0;
        //TODO: add current limits: climberConfiguration.withCurrentLimits(new limit here);

        //applies the configuration to the climber 
        climber.getConfigurator().apply(climberConfiguration);

    }

    //returns height in unit of rotations
    public double getHeight(){
        return climber.getRotorPosition().getValueAsDouble(); 
        //return climber.getPosition().getValueAsDouble();
        //TODO: what's the difference between position and rotor position??

    }

    public void setHeight(double height){
        climber.setPosition(height); 
    } 

    public void extend(){
        if(!isExtended())
        {
            climber.setPosition(MAX_POSITION); 
        }
    }

    public void retract(){
        if(!isRetracted())
        {
            climber.setPosition(MIN_POSITION);
        }
    }

    public boolean isExtended()
    {
        return (climber.getPosition().getValueAsDouble() >= MAX_POSITION); 
    }

    public boolean isRetracted()
    {
        return (climber.getPosition().getValueAsDouble() <= MIN_POSITION); 
    }
}

    
