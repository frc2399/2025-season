package frc.robot.subsystems.coralWrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SetpointConstants;

public class CoralWristHardware implements CoralWristIO {

  private final double STATIC_FF_CORAL = 0;
  private final double GRAVITY_FF_CORAL = 0.036;
  private final double VELOCITY_FF_CORAL = 0.25;

  private final ArmFeedforward coralWristFeedFoward = new ArmFeedforward(STATIC_FF_CORAL, GRAVITY_FF_CORAL,
      VELOCITY_FF_CORAL);

  private final Angle WRIST_ANGULAR_OFFSET = Degrees.of(90);

  private final SparkFlex coralIntakeWristSparkFlex;

  private final SparkClosedLoopController coralIntakeWristClosedLoopController;
  private final AbsoluteEncoder coralIntakeWristAbsoluteEncoder;
  private final RelativeEncoder coralIntakeWristRelativeEncoder;
  private static final SparkFlexConfig wristSparkFlexConfig = new SparkFlexConfig();
  private static final boolean WRIST_MOTOR_INVERTED = false;

  private static final boolean ABSOLUTE_ENCODER_INVERTED = false;

  private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

  // 64:16 (4:1) gear ratio (through bore encoder on shaft)
  private final double ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR; // radians
  // divide position factor by 60 for radians per second
  private final double ABSOLUTE_ENCODER_VELOCITY_FACTOR; // radians per second
  // 3:1 and 5:1 gearbox on motor. 64:16 (4:1) gear ratio. 3 * 5 * 4 = 60
  private static final double RELATIVE_ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 60; // radians
  // divide position factor by 60 for radians per second
  private static final double RELATIVE_ENCODER_WRIST_VELOCITY_FACTOR = (2 * Math.PI) / 3600; // radians per second

  private static final boolean POSITION_WRAPPING_ENABLED = false;
  private static final Angle POSITION_WRAPPING_MIN_INPUT = Degrees.of(-90);
  private static final Angle POSITION_WRAPPING_MAX_INPUT = Degrees.of(90);

  private static final double WRIST_MOTOR_P = 1;
  private static final double WRIST_MOTOR_I = 0.0;
  private static final double WRIST_MOTOR_D = 0.3;
  private static final double WRIST_MOTOR_FF = 0.0;
  private static final double WRIST_MOTOR_MIN_OUTPUT = -1.0;
  private static final double WRIST_MOTOR_MAX_OUTPUT = 1.0;

  private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(25);
  private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-90);

  private Angle goalAngle = Radians.of(0);
  private static final Angle WRIST_ANGLE_TOLERANCE = Degrees.of(1);

  public CoralWristHardware(double ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR,
      double ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR,
      boolean SOFT_LIMIT_ENABLED, int CAN_ID) {

    ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR = ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR;
    ABSOLUTE_ENCODER_VELOCITY_FACTOR = ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR;

    wristSparkFlexConfig.inverted(WRIST_MOTOR_INVERTED).idleMode(IDLE_MODE)
        .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

    wristSparkFlexConfig.absoluteEncoder.positionConversionFactor(ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR)
        .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
        .inverted(ABSOLUTE_ENCODER_INVERTED).zeroCentered(true);

    wristSparkFlexConfig.encoder.positionConversionFactor(RELATIVE_ENCODER_WRIST_POSITION_FACTOR)
        .velocityConversionFactor(RELATIVE_ENCODER_WRIST_VELOCITY_FACTOR);

    wristSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
        .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT)
        .positionWrappingEnabled(POSITION_WRAPPING_ENABLED)
        .positionWrappingInputRange(POSITION_WRAPPING_MIN_INPUT.in(Radians),
            POSITION_WRAPPING_MAX_INPUT.in(Radians));

    wristSparkFlexConfig.softLimit
        .forwardSoftLimit(FORWARD_SOFT_LIMIT.in(Radians))
        .forwardSoftLimitEnabled(SOFT_LIMIT_ENABLED)
        .reverseSoftLimit(REVERSE_SOFT_LIMIT.in(Radians))
        .reverseSoftLimitEnabled(SOFT_LIMIT_ENABLED);

    coralIntakeWristSparkFlex = new SparkFlex(CAN_ID, MotorType.kBrushless);
    coralIntakeWristAbsoluteEncoder = coralIntakeWristSparkFlex.getAbsoluteEncoder();
    coralIntakeWristRelativeEncoder = coralIntakeWristSparkFlex.getEncoder();
    coralIntakeWristRelativeEncoder.setPosition(
        coralIntakeWristAbsoluteEncoder.getPosition());
    coralIntakeWristClosedLoopController = coralIntakeWristSparkFlex.getClosedLoopController();
  }

  @Override
  public void setGoalAngle(Supplier<ScoringLevel> scoringLevel) {
    Angle desiredAngle = Radians.of(0);
    if (scoringLevel.get() == ScoringLevel.L_ONE) {
      desiredAngle = SetpointConstants.CORAL_L1_ANGLE;
    } else if (scoringLevel.get() == ScoringLevel.L_TWO || scoringLevel.get() == ScoringLevel.L_THREE) {
      desiredAngle = SetpointConstants.CORAL_L2_L3_OUTTAKE_ANGLE;
    } else if (scoringLevel.get() == ScoringLevel.L_FOUR) {
      desiredAngle = SetpointConstants.CORAL_L4_ANGLE;
    } else if (scoringLevel.get() == ScoringLevel.INTAKE) {
      desiredAngle = SetpointConstants.CORAL_INTAKE_ANGLE;
    }
    coralIntakeWristClosedLoopController.setReference(desiredAngle.in(Radians), ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        coralWristFeedFoward.calculate(desiredAngle.in(Radians),
            coralIntakeWristRelativeEncoder.getVelocity()));
    goalAngle = desiredAngle;
  }

  @Override
  public void setWristSpeed(double speed) {
    coralIntakeWristSparkFlex.set(speed
        + coralWristFeedFoward.calculate(coralIntakeWristRelativeEncoder.getPosition()
            + WRIST_ANGULAR_OFFSET.in(Radians), speed));
  }

  @Override
  public void updateStates(CoralWristIOStates states) {
    states.wristVelocity = coralIntakeWristRelativeEncoder.getVelocity();
    states.wristAppliedVoltage = coralIntakeWristSparkFlex.getAppliedOutput()
        * coralIntakeWristSparkFlex.getBusVoltage();
    states.wristCurrent = coralIntakeWristSparkFlex.getOutputCurrent();
    states.wristRelativeEncoderAngle = coralIntakeWristRelativeEncoder.getPosition();
    states.goalAngle = goalAngle.in(Radians);
    states.wristAbsoluteAngle = coralIntakeWristAbsoluteEncoder.getPosition();
    // states.trapezoidProfileGoalAngle = goalState.position;
  }

  @Override
  public boolean atGoal() {
    return (Math.abs(coralIntakeWristRelativeEncoder.getPosition() - goalAngle.in(Radians)) < WRIST_ANGLE_TOLERANCE
        .in(Radians));
  }

  @Override
  public void periodic() {
    // setpointState = wristTrapezoidProfile.calculate(0.02,
    // setpointState, goalState);
  }

}