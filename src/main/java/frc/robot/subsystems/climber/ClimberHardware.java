package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.subsystems.coralWrist.CoralWristHardware;

public class ClimberHardware implements ClimberIO {

  public static final class ClimberConstants{
    //use motor current limit in constants 
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;
    private static final double FEEDFORWARD_VALUE = 1.0;
    private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(1);
    private static final double P_VALUE = 1.0;
    private static final double I_VALUE = 0.0;
    private static final double D_VALUE = 0.0;
    private static final double CLIMBER_MOTOR_MIN_OUTPUT = -1.0;
    private static final double CLIMBER_MOTOR_MAX_OUTPUT = 1.0;
    private static final Angle MAX_ANGLE = Degrees.of(90);

    private static final boolean LEFT_CLIMBER_INVERTED = false;
    private static final boolean RIGHT_CLIMBER_INVERTED = false;

    private static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

  } 

    final SparkFlex leftClimber = new SparkFlex(MotorIdConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
    final SparkFlex rightClimber  = new SparkFlex(MotorIdConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
    final SparkFlexConfig leftClimberConfig = new SparkFlexConfig();
    final SparkFlexConfig rightClimberConfig = new SparkFlexConfig();
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final SparkClosedLoopController climberClosedLoopController = leftClimber.getClosedLoopController();
    private PositionVoltage climberPIDPositionControl;
    private double goalAngle; 

    public ClimberHardware(){

        leftClimberConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED).idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        leftClimberConfig.encoder.positionConversionFactor(); //TODO: add conversion factor AND add velocity converison factor

        leftClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE, ClimberConstants.FEEDFORWARD_VALUE)
                .outputRange(ClimberConstants.CLIMBER_MOTOR_MIN_OUTPUT, ClimberConstants.CLIMBER_MOTOR_MAX_OUTPUT);
        //TODO: uncomment once max angle is correct!
        // leftClimberConfig.softLimit
        //         .forwardSoftLimit(ClimberConstants.MAX_ANGLE)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(0)
        //         .reverseSoftLimitEnabled(true);
                

        rightClimberConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED).idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        rightClimberConfig.encoder.positionConversionFactor();

        rightClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE, ClimberConstants.FEEDFORWARD_VALUE)
                .outputRange(ClimberConstants.CLIMBER_MOTOR_MIN_OUTPUT, ClimberConstants.CLIMBER_MOTOR_MAX_OUTPUT);

        //TODO: uncomment once max angle is correct!
        // rightClimberConfig.softLimit
        //         .forwardSoftLimit(ClimberConstants.MAX_ANGLE)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(0)
        //         .reverseSoftLimitEnabled(true);

        rightClimberConfig.follow(leftClimber.getDeviceId(), true);

        leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        
        leftClimberEncoder.setPosition(0); 
    }


    public double getAngle(){
        return leftClimberEncoder.getPosition(); 
    }

    public void setGoalAngle(Angle desiredAngle){
        climberClosedLoopController.setReference(desiredAngle.in(Radians), ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            climberFeedForward.calculate(desiredAngle.in(Radians), leftClimberEncoder.getVelocity());
    
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

    
