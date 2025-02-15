package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {
    final TalonFX leftClimber = new TalonFX(MotorIdConstants.CLIMBER_LEFT_CAN_ID);
    final TalonFX rightClimber  = new TalonFX(MotorIdConstants.CLIMBER_RIGHT_CAN_ID);
    final TalonFXConfigurator leftConfigurator = leftClimber.getConfigurator();  
    final TalonFXConfigurator rightConfigurator = rightClimber.getConfigurator(); 
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    //TODO: tune min and max positions 
    private static final double MAX_POSITION = 0; 
    private static final double MIN_POSITION = 0;

    //TODO: tune tolerance
    private static final double HEIGHT_TOLERANCE = 0.05;

    //TODO: tune! 
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;

    public ClimberHardware(){

        //TODO: tune PID 
        configuration.Slot0.kP = 0;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;
        configuration.Slot0.kV = 0;

        //we may have to add a second configuration if the inversion is switched 
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        configuration.CurrentLimits.StatorCurrentLimit = 120;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.Feedback.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

        leftConfigurator.apply(configuration); 
        rightConfigurator.apply(configuration); 

        rightClimber.setControl(new StrictFollower(leftClimber.getDeviceID()));

    }

    //returns angle in unit of rotations
    //TODO: convert to degrees/radians 
    public double getAngle(){
        return leftClimber.getPosition().getValueAsDouble(); 
        //TODO: add a position conversion fcator 
    }

    public void setGoalAngle(double desiredAngle){
        leftClimber.setPosition(desiredAngle); 
    } 
  
    public void setSpeed(double speed)
    {
        leftClimber.set(speed);
    }

    
}

    
