package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {

  public static final class ClimberConstants{
    //use motor current limit in constants 
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;
    private static final Voltage FEEDFORWARD_VALUE = Volts.of(1);
    private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(1);
    private static final Voltage P_VALUE = Volts.of(1);
    private static final Voltage I_VALUE = Volts.of(0);
    private static final Voltage D_VALUE = Volts.of(0);
    private static final Angle MAX_ANGLE = Degrees.of(90);

    private static final boolean LEFT_CLIMBER_INVERTED = false;
    private static final boolean RIGHT_CLIMBER_INVERTED = false;

    private static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

  } 

    final SparkFlex leftClimber = new SparkFlex(MotorIdConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
    final SparkFlex rightClimber  = new SparkFlex(MotorIdConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
    final SparkFlexConfig leftClimberConfig = new SparkFlexConfig();
    final SparkFlexConfig rightClimberConfig = new SparkFlexConfig();
    private PositionVoltage climberPIDPositionControl;
    private double goalAngle; 

    public ClimberHardware(){

        leftClimberConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED).idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        leftClimberConfig.encoder.positionConversionFactor(); //TODO: add conversion factor 
        leftClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf()
                

        rightClimberConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED).idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        rightClimberConfig.encoder.positionConversionFactor();
        rightClimberConfig.follow(leftClimber.getDeviceId(), true);

        leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);



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

    
