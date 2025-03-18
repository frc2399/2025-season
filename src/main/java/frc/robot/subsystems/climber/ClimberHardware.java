package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {

  public static final class ClimberConstants{
    //use motor current limit in constants 
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;
    private static final double FEEDFORWARD_VALUE = 1.0;
    private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(1);
    private static final double P_VALUE = 1.0;
    private static final double I_VALUE = 0.0;
    private static final double D_VALUE = 0.0;
    //TODO: this is definitely wrong... figure out how we wanna init and handle soft limit + also offset angle
    private static final Angle MAX_ANGLE = Degrees.of(90);
    private static final Angle CLIMBER_OFFSET_ANGLE = Degrees.of(0);

    //TODO: check!
    private static final Angle CLIMBER_POSITION_CONVERSION_FACTOR = Radians.of(0.0235);
    private static final double CLIMBER_VELOCITY_CONVERSION_FACTOR = (CLIMBER_POSITION_CONVERSION_FACTOR).in(Radians) / 60.0; 
    private static final double CLIMBER_STATIC_FF = 0.0;
    private static final double CLIMBER_GRAVITY_FF = 0.0;
    private static final double CLIMBER_VELOCITY_FF = 0.0; 
    private static final int SERVO_CHANNEL = 1; 

    private static final boolean LEFT_CLIMBER_INVERTED = false;
    private static final boolean RIGHT_CLIMBER_INVERTED = false;

    private static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

  } 

    private final SparkMax leftClimber = new SparkMax(MotorIdConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightClimber  = new SparkMax(MotorIdConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
    private final Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL); 

    private final SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightClimberConfig = new SparkMaxConfig();

    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final SparkClosedLoopController climberClosedLoopController = leftClimber.getClosedLoopController();
    private final ArmFeedforward climberFeedforward = new ArmFeedforward(ClimberConstants.CLIMBER_STATIC_FF, ClimberConstants.CLIMBER_GRAVITY_FF, ClimberConstants.CLIMBER_VELOCITY_FF); 
    
    private Angle climberGoalAngle = Radians.of(0.0);
    private Angle servoGoalAngle = Radians.of(0.0);

    public ClimberHardware(){

        leftClimberConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED)
                        .idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                        .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));

        leftClimberConfig.encoder.positionConversionFactor((ClimberConstants.CLIMBER_POSITION_CONVERSION_FACTOR).in(Radians))
                .velocityConversionFactor(ClimberConstants.CLIMBER_VELOCITY_CONVERSION_FACTOR);

        leftClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE, ClimberConstants.FEEDFORWARD_VALUE);
    
        //TODO: uncomment once max angle is correct!
        // leftClimberConfig.softLimit
        //         .forwardSoftLimit(ClimberConstants.MAX_ANGLE)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(0)
        //         .reverseSoftLimitEnabled(true);
                

        rightClimberConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED).idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        rightClimberConfig.encoder.positionConversionFactor(ClimberConstants.CLIMBER_POSITION_CONVERSION_FACTOR.in(Radians))
                .velocityConversionFactor(ClimberConstants.CLIMBER_VELOCITY_CONVERSION_FACTOR);

        rightClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE, ClimberConstants.FEEDFORWARD_VALUE);

        //TODO: uncomment once max angle is correct! we should start at 'furthest back' position, which is our soft limit, and this is either forward or reverse
        // rightClimberConfig.softLimit
        //         .forwardSoftLimit(ClimberConstants.MAX_ANGLE)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(0)
        //         .reverseSoftLimitEnabled(true);

        rightClimberConfig.follow(leftClimber.getDeviceId(), true);

        leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftClimberEncoder.setPosition(0); 
    }


    public void setGoalAngle(Angle desiredAngle){
        climberClosedLoopController.setReference(desiredAngle.in(Radians), ControlType.kPosition,
                                ClosedLoopSlot.kSlot0,
                                climberFeedforward.calculate( // arm FF assumes 0 = horizontal ... this will likely not be the case
                                                desiredAngle.in(Radians) + ClimberConstants.CLIMBER_OFFSET_ANGLE.in(Radians),
                                                leftClimberEncoder.getVelocity()));
        climberGoalAngle = desiredAngle; 
    } 
  
    public void setSpeed(double speed)
    {
        leftClimber.set(speed + climberFeedforward.calculate(leftClimberEncoder.getPosition() + ClimberConstants.CLIMBER_OFFSET_ANGLE.in(Radians), speed));
    }

    public void setServoAngle(Angle desiredAngle)
    {
        climberServo.setAngle(desiredAngle.in(Degrees));
        servoGoalAngle = desiredAngle;
    }

    public void updateStates(ClimberIOInputs inputs){
        inputs.climberAngle = leftClimberEncoder.getPosition(); 
        inputs.climberVelocity = leftClimberEncoder.getVelocity();
        inputs.climberGoalAngle = climberGoalAngle.in(Degrees); 

        inputs.servoAngle = climberServo.getAngle();
        inputs.servoVelocity = climberServo.getSpeed(); 
        inputs.servoGoalAngle = servoGoalAngle.in(Degrees);
    }

}

    
