package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class CoralIntakeHardware implements CoralIntakeIO {
   private final SparkMax coralIntakeLeftSparkMax;
   private final SparkMax coralIntakeRightSparkMax;
   private final SparkFlex coralIntakeWristSparkFlex;


   private final SparkClosedLoopController coralIntakeLeftClosedLoopController;
   private final SparkClosedLoopController coralIntakeRightClosedLoopController;
   private final SparkClosedLoopController coralIntakeWristClosedLoopController;


   private final RelativeEncoder coralIntakeLeftEncoder;
   private final RelativeEncoder coralIntakeRightEncoder;
   private final AbsoluteEncoder coralIntakeWristAbsoluteEncoder;


   private static final SparkMaxConfig LEFT_SPARK_MAX_CONFIG = new SparkMaxConfig();
   private static final SparkMaxConfig RIGHT_SPARK_MAX_CONFIG = new SparkMaxConfig();
   private static final SparkFlexConfig WRIST_SPARK_FLEX_CONFIG = new SparkFlexConfig();


   private static final boolean ENCODER_INVERTED = true;
   private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
   private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
   private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second


   private static final double P = 1.0;
   private static final double I = 0.0;
   private static final double D = 0.001;
   private static final double FF = 0.0;
   private static final double MIN_OUTPUT = -1.0;
   private static final double MAX_OUTPUT = 1.0;


   private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);
   public static final LinearVelocity CORAL_INTAKE_MAX_VELOCITY = MetersPerSecond.of(0); //needs to be tested



   public CoralIntakeHardware() {
       LEFT_SPARK_MAX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
               .smartCurrentLimit(MotorConstants.NEO550_FREE_SPEED_RPM);
       LEFT_SPARK_MAX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
               .velocityConversionFactor(ENCODER_POSITION_FACTOR);
       LEFT_SPARK_MAX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
               .pidf(P, I, D, FF).outputRange(MIN_OUTPUT, MAX_OUTPUT);


       RIGHT_SPARK_MAX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
               .smartCurrentLimit(MotorConstants.NEO550_FREE_SPEED_RPM);
       RIGHT_SPARK_MAX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
               .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
       RIGHT_SPARK_MAX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
               .pidf(P, I, D, FF).outputRange(MIN_OUTPUT, MAX_OUTPUT);


       WRIST_SPARK_FLEX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
               .smartCurrentLimit(MotorConstants.VORTEX_CURRENT_LIMIT);
       WRIST_SPARK_FLEX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
               .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
       WRIST_SPARK_FLEX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
               .pidf(P, I, D, FF).outputRange(MIN_OUTPUT, MAX_OUTPUT);


       coralIntakeLeftSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);
       coralIntakeRightSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_RIGHT_CAN_ID, MotorType.kBrushless);
       coralIntakeWristSparkFlex = new SparkFlex(MotorIdConstants.CORAL_INTAKE_WRIST_CAN_ID, MotorType.kBrushless);


       coralIntakeLeftEncoder = coralIntakeLeftSparkMax.getEncoder();
       coralIntakeRightEncoder = coralIntakeRightSparkMax.getEncoder();
       coralIntakeWristAbsoluteEncoder = coralIntakeWristSparkFlex.getAbsoluteEncoder();


       coralIntakeLeftSparkMax.configure(LEFT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);
       coralIntakeRightSparkMax.configure(RIGHT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);
       coralIntakeWristSparkFlex.configure(WRIST_SPARK_FLEX_CONFIG, ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);

       coralIntakeLeftClosedLoopController = coralIntakeLeftSparkMax.getClosedLoopController();
       coralIntakeRightClosedLoopController = coralIntakeRightSparkMax.getClosedLoopController();
       coralIntakeWristClosedLoopController = coralIntakeWristSparkFlex.getClosedLoopController();
   }

   public void setSpeed(double speed) {
       coralIntakeLeftClosedLoopController.setReference(
               speed * CORAL_INTAKE_MAX_VELOCITY.in(MetersPerSecond), ControlType.kVelocity);
       coralIntakeRightClosedLoopController.setReference(
               speed * CORAL_INTAKE_MAX_VELOCITY.in(MetersPerSecond), ControlType.kVelocity);
       coralIntakeWristClosedLoopController.setReference(
               speed * CORAL_INTAKE_MAX_VELOCITY.in(MetersPerSecond), ControlType.kVelocity);
   }

   public double getVelocity() {
       return coralIntakeLeftEncoder.getVelocity();
   }

   public double getCurrent() {
       return coralIntakeLeftSparkMax.getOutputCurrent();
   }

   @Override
   public void updateStates(CoralIntakeIOStates states) {
       states.velocity = getVelocity();
       states.leftAppliedVoltage = coralIntakeLeftSparkMax.getAppliedOutput()
               * coralIntakeLeftSparkMax.getBusVoltage();
       states.rightAppliedVoltage = coralIntakeRightSparkMax.getAppliedOutput()
               * coralIntakeRightSparkMax.getBusVoltage();
       states.wristAppliedVoltage = coralIntakeWristSparkFlex.getAppliedOutput()
               * coralIntakeWristSparkFlex.getBusVoltage();

       states.leftCurrent = coralIntakeLeftSparkMax.getOutputCurrent();
       states.rightCurrent = coralIntakeRightSparkMax.getOutputCurrent();
       states.wristCurrent = coralIntakeWristSparkFlex.getOutputCurrent();
   }
}