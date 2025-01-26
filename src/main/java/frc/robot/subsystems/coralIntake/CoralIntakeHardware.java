package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class CoralIntakeHardware implements CoralIntakeIO {
        private final double STATIC_FF_CORAL = 0;
        private final double GRAVITY_FF_CORAL = 0.43;
        private final double VELOCITY_FF_CORAL = 1.01;

        private final ArmFeedforward coralWristFeedFoward = new ArmFeedforward(STATIC_FF_CORAL, GRAVITY_FF_CORAL, VELOCITY_FF_CORAL);
        private final SparkMax coralIntakeTopSparkMax;
        private final SparkMax coralIntakeBottomSparkMax;
        private final SparkFlex coralIntakeWristSparkFlex;

        private final SparkClosedLoopController coralIntakeTopClosedLoopController;
        private final SparkClosedLoopController coralIntakeBottomClosedLoopController;
        private final SparkClosedLoopController coralIntakeWristClosedLoopController;

        private final RelativeEncoder coralIntakeTopEncoder;
        private final RelativeEncoder coralIntakeBottomEncoder;
        private final RelativeEncoder coralIntakeWristRelativeEncoder;

        private static final SparkMaxConfig topSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig bottomSparkMaxConfig = new SparkMaxConfig();
        private static final SparkFlexConfig wristSparkFlexConfig = new SparkFlexConfig();

        private static final boolean TOP_MOTOR_INVERTED = false;
        private static final boolean BOTTOM_MOTOR_INVERTED = true;
        private static final boolean WRIST_MOTOR_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_ROLLER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_WRIST_POSITION_FACTOR = 60 * (2 * Math.PI); //radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double SIDE_MOTOR_P = 0.5;
        private static final double SIDE_MOTOR_I = 0.0;
        private static final double SIDE_MOTOR_D = 0.0;
        private static final double SIDE_MOTOR_FF = 0.1;
        private static final double SIDE_MOTOR_MIN_OUTPUT = -1.0;
        private static final double SIDE_MOTOR_MAX_OUTPUT = 1.0;

        private static final double WRIST_MOTOR_P = 0.0;
        private static final double WRIST_MOTOR_I = 0.0;
        private static final double WRIST_MOTOR_D = 0.0;
        private static final double WRIST_MOTOR_FF = 0.0;
        private static final double WRIST_MOTOR_MIN_OUTPUT = -1.0;
        private static final double WRIST_MOTOR_MAX_OUTPUT = 1.0;

        private static final boolean POSITION_WRAPPING_ENABLED_SIDE_MOTORS = true;

        private static final double GRAIVTY_COMPENSATION = 0.01;

        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);
        public static final LinearVelocity CORAL_INTAKE_MAX_VELOCITY = MetersPerSecond.of(0.1); // needs to be tested

        public CoralIntakeHardware() {
                topSparkMaxConfig.inverted(TOP_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                topSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_ROLLER_POSITION_FACTOR);
                topSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(SIDE_MOTOR_P, SIDE_MOTOR_I, SIDE_MOTOR_D, SIDE_MOTOR_FF)
                                .outputRange(SIDE_MOTOR_MIN_OUTPUT, SIDE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED_SIDE_MOTORS);

                bottomSparkMaxConfig.inverted(BOTTOM_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                bottomSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                bottomSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(SIDE_MOTOR_P, SIDE_MOTOR_I, SIDE_MOTOR_D, SIDE_MOTOR_FF)
                                .outputRange(SIDE_MOTOR_MIN_OUTPUT, SIDE_MOTOR_MAX_OUTPUT);

                wristSparkFlexConfig.inverted(WRIST_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.VORTEX_CURRENT_LIMIT);
                wristSparkFlexConfig.encoder.positionConversionFactor(ENCODER_WRIST_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                wristSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
                                .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT);

                coralIntakeTopSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_TOP_CAN_ID, MotorType.kBrushless);
                coralIntakeBottomSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_BOTTOM_CAN_ID,
                                MotorType.kBrushless);
                coralIntakeWristSparkFlex = new SparkFlex(MotorIdConstants.CORAL_INTAKE_WRIST_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeTopEncoder = coralIntakeTopSparkMax.getEncoder();
                coralIntakeBottomEncoder = coralIntakeBottomSparkMax.getEncoder();
                coralIntakeWristRelativeEncoder = coralIntakeWristSparkFlex.getEncoder();

                coralIntakeTopSparkMax.configure(topSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeBottomSparkMax.configure(bottomSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeWristSparkFlex.configure(wristSparkFlexConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                coralIntakeTopClosedLoopController = coralIntakeTopSparkMax.getClosedLoopController();
                coralIntakeBottomClosedLoopController = coralIntakeBottomSparkMax.getClosedLoopController();
                coralIntakeWristClosedLoopController = coralIntakeWristSparkFlex.getClosedLoopController();
        }

        public void setRollerSpeed(double speed) {
                coralIntakeBottomSparkMax.set(speed);
                coralIntakeTopSparkMax.set(speed);
        }

        public void goToSetpoint(Angle angle) {
                coralIntakeWristClosedLoopController.setReference(angle.in(Radians), ControlType.kPosition, 
                ClosedLoopSlot.kSlot0, coralWristFeedFoward.calculate(angle.in(Radians), ENCODER_VELOCITY_FACTOR));
        }

        public void setWristSpeed(double speed) {
                coralIntakeWristSparkFlex.set(speed);
        }

        public void setGravityCompensation() {
                coralIntakeWristClosedLoopController.setReference(
                                Math.cos(coralIntakeWristRelativeEncoder.getPosition()) * GRAIVTY_COMPENSATION,
                                ControlType.kVelocity);
        }

        public double getVelocity() {
                return coralIntakeTopEncoder.getVelocity();
        }

        public double getCurrent() {
                return coralIntakeTopSparkMax.getOutputCurrent();
        }

        @Override
        public void updateStates(CoralIntakeIOStates states) {
                states.velocity = getVelocity();
                states.topAppliedVoltage = coralIntakeTopSparkMax.getAppliedOutput()
                                * coralIntakeTopSparkMax.getBusVoltage();
                states.bottomAppliedVoltage = coralIntakeBottomSparkMax.getAppliedOutput()
                                * coralIntakeBottomSparkMax.getBusVoltage();
                states.wristAppliedVoltage = coralIntakeWristSparkFlex.getAppliedOutput()
                                * coralIntakeWristSparkFlex.getBusVoltage();

                states.topCurrent = coralIntakeTopSparkMax.getOutputCurrent();
                states.bottomCurrent = coralIntakeBottomSparkMax.getOutputCurrent();
                states.wristCurrent = coralIntakeWristSparkFlex.getOutputCurrent();
        }
}