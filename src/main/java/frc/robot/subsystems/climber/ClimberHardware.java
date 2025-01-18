package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {
    final TalonFX climber;
    final TalonFXConfiguration climberConfiguration;

    //TODO: tune min and max positions 
    private static final double MAX_POSITION = 0; 
    private static final double MIN_POSITION = 0;

    public ClimberHardware(){
        climber = new TalonFX(MotorIdConstants.CLIMBER_CAN_ID);

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
        return climber.getPosition().getValueAsDouble(); 
        //TODO: add a position conversion fcator 
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
        //TODO: add tolerance 
    }

    public boolean isRetracted()
    {
        return (climber.getPosition().getValueAsDouble() <= MIN_POSITION); 
    }

    public void set(double speed)
    {
        climber.set(speed);
    }
}

    
