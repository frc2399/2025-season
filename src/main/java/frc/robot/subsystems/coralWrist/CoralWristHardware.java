package frc.robot.subsystems.coralWrist;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class CoralWristHardware implements CoralWristIO {
        private final double STATIC_FF_CORAL = 0;
        private final double GRAVITY_FF_CORAL = 0.013;
        private final double VELOCITY_FF_CORAL = 0.0;

        private final ArmFeedforward coralWristFeedFoward = new ArmFeedforward(STATIC_FF_CORAL, GRAVITY_FF_CORAL,
                        VELOCITY_FF_CORAL);

        private final SparkFlex coralIntakeWristSparkFlex;

        private final SparkClosedLoopController coralIntakeWristClosedLoopController;
        private final AbsoluteEncoder coralIntakeWristAbsoluteEncoder;
        private static final SparkFlexConfig wristSparkFlexConfig = new SparkFlexConfig();
        private static final boolean WRIST_MOTOR_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        private static final double ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 60.0; // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double WRIST_MOTOR_P = 0.0;
        private static final double WRIST_MOTOR_I = 0.0;
        private static final double WRIST_MOTOR_D = 0.0;
        private static final double WRIST_MOTOR_FF = 0.0;
        private static final double WRIST_MOTOR_MIN_OUTPUT = -1.0;
        private static final double WRIST_MOTOR_MAX_OUTPUT = 1.0;

        public CoralWristHardware() {
                wristSparkFlexConfig.inverted(WRIST_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.VORTEX_CURRENT_LIMIT);
                wristSparkFlexConfig.encoder.positionConversionFactor(ENCODER_WRIST_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                wristSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
                                .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT);

                coralIntakeWristSparkFlex = new SparkFlex(MotorIdConstants.CORAL_INTAKE_WRIST_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeWristAbsoluteEncoder = coralIntakeWristSparkFlex.getAbsoluteEncoder();

                coralIntakeWristSparkFlex.configure(wristSparkFlexConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeWristClosedLoopController = coralIntakeWristSparkFlex.getClosedLoopController();
        }

        @Override
        public void goToSetpoint(Angle angle) {
                coralIntakeWristClosedLoopController.setReference(angle.in(Radians), ControlType.kPosition,
                                ClosedLoopSlot.kSlot0,
                                coralWristFeedFoward.calculate(angle.in(Radians), ENCODER_VELOCITY_FACTOR));
        }

        @Override
        public void setWristSpeed(double speed) {
                coralIntakeWristSparkFlex.set(speed
                                + coralWristFeedFoward.calculate(coralIntakeWristAbsoluteEncoder.getPosition(), speed));
        }

        @Override
        public void updateStates(CoralWristIOStates states) {
                states.wristAppliedVoltage = coralIntakeWristSparkFlex.getAppliedOutput()
                                * coralIntakeWristSparkFlex.getBusVoltage();
                states.wristCurrent = coralIntakeWristSparkFlex.getOutputCurrent();
                states.wristAbsoluteEncoderAngle = coralIntakeWristAbsoluteEncoder.getPosition();
        }
}