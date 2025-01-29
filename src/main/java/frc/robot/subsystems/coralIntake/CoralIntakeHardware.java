package frc.robot.subsystems.coralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class CoralIntakeHardware implements CoralIntakeIO {

        private final SparkMax coralIntakeTopSparkMax;
        private final SparkMax coralIntakeBottomSparkMax;

        private final SparkClosedLoopController coralIntakeTopClosedLoopController;
        private final SparkClosedLoopController coralIntakeBottomClosedLoopController;

        private final RelativeEncoder coralIntakeTopEncoder;
        private final RelativeEncoder coralIntakeBottomEncoder;

        private static final SparkMaxConfig topSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig bottomSparkMaxConfig = new SparkMaxConfig();

        private static final boolean TOP_MOTOR_INVERTED = false;
        private static final boolean BOTTOM_MOTOR_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_ROLLER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double SIDE_MOTOR_P = 0.5;
        private static final double SIDE_MOTOR_I = 0.0;
        private static final double SIDE_MOTOR_D = 0.0;
        private static final double SIDE_MOTOR_FF = 0.1;
        private static final double SIDE_MOTOR_MIN_OUTPUT = -1.0;
        private static final double SIDE_MOTOR_MAX_OUTPUT = 1.0;

        private static final boolean POSITION_WRAPPING_ENABLED_SIDE_MOTORS = true;

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

                coralIntakeTopSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_TOP_CAN_ID, MotorType.kBrushless);
                coralIntakeBottomSparkMax = new SparkMax(MotorIdConstants.CORAL_INTAKE_BOTTOM_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeTopEncoder = coralIntakeTopSparkMax.getEncoder();
                coralIntakeBottomEncoder = coralIntakeBottomSparkMax.getEncoder();

                coralIntakeTopSparkMax.configure(topSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeBottomSparkMax.configure(bottomSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                coralIntakeTopClosedLoopController = coralIntakeTopSparkMax.getClosedLoopController();
                coralIntakeBottomClosedLoopController = coralIntakeBottomSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(double speed) {
                coralIntakeBottomSparkMax.set(speed);
                coralIntakeTopSparkMax.set(speed);
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
                states.topCurrent = coralIntakeTopSparkMax.getOutputCurrent();
                states.bottomCurrent = coralIntakeBottomSparkMax.getOutputCurrent();
        }
}