package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {

  public static final class ClimberConstants{
    private static final double CURRENT_LIMIT = 80; 
      //TODO: tune!
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;
    private static final Voltage FEEDFORWARD_VALUE = Volts.of(1);
    private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(1);
    private static final Voltage P_VALUE = Volts.of(1);
    private static final Voltage I_VALUE = Volts.of(0);
    private static final Voltage D_VALUE = Volts.of(0);
    private static final Angle MAX_ANGLE = Degrees.of(90);
  } 

    final TalonFX leftClimber = new TalonFX(MotorIdConstants.LEFT_CLIMBER_CAN_ID);
    final TalonFX rightClimber  = new TalonFX(MotorIdConstants.RIGHT_CLIMBER_CAN_ID);
    final TalonFXConfigurator leftConfigurator = leftClimber.getConfigurator();  
    final TalonFXConfigurator rightConfigurator = rightClimber.getConfigurator(); 
    final TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    final TalonFXConfiguration rightConfiguration = new TalonFXConfiguration(); 
    final TalonFXConfiguration globalMotorConfiguration = new TalonFXConfiguration();
    private PositionVoltage climberPIDPositionControl;
    private double goalAngle; 

    public ClimberHardware(){

        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        leftConfiguration.CurrentLimits.withStatorCurrentLimit(ClimberConstants.CURRENT_LIMIT);
        leftConfiguration.Feedback.withSensorToMechanismRatio(ClimberConstants.SENSOR_TO_MECHANISM_RATIO);

        //rightMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        rightConfiguration.CurrentLimits.withStatorCurrentLimit(ClimberConstants.CURRENT_LIMIT);
        rightConfiguration.Feedback.withSensorToMechanismRatio(ClimberConstants.SENSOR_TO_MECHANISM_RATIO);

        rightConfigurator.apply(rightConfiguration); 
        leftConfigurator.apply(leftConfiguration); 

        rightClimber.setNeutralMode(NeutralModeValue.Brake);
        leftClimber.setNeutralMode(NeutralModeValue.Brake);

        rightClimber.setControl(new Follower(leftClimber.getDeviceID(), true));


        globalMotorConfiguration.Slot0.kS = (ClimberConstants.FEEDFORWARD_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kG = (ClimberConstants.ARBITRARY_FF_GRAVITY_COMPENSATION).in(Volts);
        globalMotorConfiguration.Slot0.kP = (ClimberConstants.P_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kI = (ClimberConstants.I_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kD = (ClimberConstants.D_VALUE).in(Volts);

        //TODO: add this back once the correct constants for max/min angle are added
        // globalMotorConfiguration.SoftwareLimitSwitch
        //         .withForwardSoftLimitThreshold(ClimberConstants.MAX_ANGLE.in(Degrees));
        // globalMotorConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        // globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
        // globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

        rightConfigurator.apply(globalMotorConfiguration);
        leftConfigurator.apply(globalMotorConfiguration);
        
        leftClimber.setPosition(0); 
    }


    public double getAngle(){
        return leftClimber.getPosition().getValueAsDouble(); 
    }

    public void setGoalAngle(Angle desiredAngle){
        leftClimber.setControl(climberPIDPositionControl.withPosition(desiredAngle)
                .withFeedForward(ClimberConstants.ARBITRARY_FF_GRAVITY_COMPENSATION));
        goalAngle = desiredAngle.in(Degrees); 
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
        inputs.angle = getAngle(); 
        inputs.velocity = getVelocity();
        inputs.goalAngle = goalAngle; 
    }

}

    
