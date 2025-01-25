package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
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

        private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();
        private static final SparkFlexConfig wristSparkFlexConfig = new SparkFlexConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final boolean RIGHT_MOTOR_INVERTED = false;
        private static final boolean WRIST_MOTOR_INVERTED = false;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double SIDE_MOTOR_P = 1.0;
        private static final double SIDE_MOTOR_I = 0.0;
        private static final double SIDE_MOTOR_D = 0.001;
        private static final double SIDE_MOTOR_FF = 0.0;
        private static final double SIDE_MOTOR_MIN_OUTPUT = -1.0;
        private static final double SIDE_MOTOR_MAX_OUTPUT = 1.0;

        private static final double WRIST_MOTOR_P = 1.0;
        private static final double WRIST_MOTOR_I = 0.0;
        private static final double WRIST_MOTOR_D = 0.001;
        private static final double WRIST_MOTOR_FF = 0.0;
        private static final double WRIST_MOTOR_MIN_OUTPUT = -1.0;
        private static final double WRIST_MOTOR_MAX_OUTPUT = 1.0;

        private static final double GRAIVTY_COMPENSATION = 0.01;

        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);
        public static final LinearVelocity CORAL_INTAKE_MAX_VELOCITY = MetersPerSecond.of(0.1); // needs to be tested

        public CoralIntakeHardware() {
                leftSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                leftSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_POSITION_FACTOR);
                leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(SIDE_MOTOR_P, SIDE_MOTOR_I, SIDE_MOTOR_D, SIDE_MOTOR_FF)
                                .outputRange(SIDE_MOTOR_MIN_OUTPUT, SIDE_MOTOR_MAX_OUTPUT);

                rightSparkMaxConfig.inverted(RIGHT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT)
                                .follow(MotorIdConstants.CORAL_INTAKE_LEFT_CAN_ID);
                rightSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                rightSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(SIDE_MOTOR_P, SIDE_MOTOR_I, SIDE_MOTOR_D, SIDE_MOTOR_FF)
                                .outputRange(SIDE_MOTOR_MIN_OUTPUT, SIDE_MOTOR_MAX_OUTPUT);

                wristSparkFlexConfig.inverted(WRIST_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.VORTEX_CURRENT_LIMIT);
                wristSparkFlexConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                wristSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
                                .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT);

                coralIntakeLeftSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);
                coralIntakeRightSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_RIGHT_CAN_ID,
                                MotorType.kBrushless);
                coralIntakeWristSparkFlex = new SparkFlex(MotorIdConstants.CORAL_INTAKE_WRIST_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeLeftEncoder = coralIntakeLeftSparkMax.getEncoder();
                coralIntakeRightEncoder = coralIntakeRightSparkMax.getEncoder();
                coralIntakeWristAbsoluteEncoder = coralIntakeWristSparkFlex.getAbsoluteEncoder();

                coralIntakeLeftSparkMax.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeRightSparkMax.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeWristSparkFlex.configure(wristSparkFlexConfig, ResetMode.kResetSafeParameters,
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
        }

        public void goToSetpoint(Angle angle) {
                coralIntakeWristClosedLoopController.setReference(angle.in(Radians), ControlType.kPosition);
        }

        public void setGravityCompensation() {
                coralIntakeWristClosedLoopController.setReference(
                                Math.cos(coralIntakeWristAbsoluteEncoder.getPosition()) * GRAIVTY_COMPENSATION,
                                ControlType.kVelocity);
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