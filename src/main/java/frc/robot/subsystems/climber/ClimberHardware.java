package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {
    final TalonFX leftClimber = new TalonFX(MotorIdConstants.CLIMBER_LEFT_CAN_ID);
    final TalonFX rightClimber  = new TalonFX(MotorIdConstants.CLIMBER_RIGHT_CAN_ID);
    final TalonFXConfigurator leftConfigurator = leftClimber.getConfigurator();  
    final TalonFXConfigurator rightConfigurator = rightClimber.getConfigurator(); 
    final TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    final TalonFXConfiguration rightConfiguration = new TalonFXConfiguration(); 

    //TODO: tune! 
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;

    public ClimberHardware(){

        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        leftConfiguration.CurrentLimits.withStatorCurrentLimit(80);
        leftConfiguration.Feedback.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

        //rightMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        rightConfiguration.CurrentLimits.withStatorCurrentLimit(80);
        rightConfiguration.Feedback.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

        rightConfigurator.apply(rightConfiguration); 
        leftConfigurator.apply(leftConfiguration); 

        rightClimber.setNeutralMode(NeutralModeValue.Brake);
        leftClimber.setNeutralMode(NeutralModeValue.Brake);

        rightClimber.setControl(new Follower(leftClimber.getDeviceID(), true));

    }

    public void setAngle(double angle)
    {
        leftClimber.setPosition(angle); 
    }

    public double getAngle(){
        return leftClimber.getPosition().getValueAsDouble(); 
        //TODO: add a position conversion fcator 
    }

    public void setGoalAngle(double desiredAngle){
        // leftClimber.setControl(elevatorPIDPositionControl.withPosition(desiredPosition)
        //         .withFeedForward(KrakenElevatorConstants.ARBITRARY_FF_GRAVITY_COMPENSATION));
        // goalPosition = desiredPosition; 
    } 
  
    public void setSpeed(double speed)
    {
        leftClimber.setControl(new DutyCycleOut(speed));
    }

    public double getVelocity()
    {
        return leftClimber.getVelocity().getValueAsDouble(); 
    }

    public void updateStates(ClimberIOInputs inputs){
        //add 
    }

}

    
