package frc.robot.subsystems.coralWrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
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

        private static final boolean ABSOLUTE_ENCODER_INVERTED = true;

        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        private static final double ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 4.0; // radians
        private static final double ABSOLUTE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 240.0; // radians per second

        private static final boolean ABSOLUTE_ENCODER_POSITION_WRAPPING = true;
        private static final Angle ABSOLUTE_ENCODER_POSITION_MIN_INPUT = Degrees.of(-90);
        private static final Angle ABSOLUTE_ENCODER_POSITION_MAX_INPUT = Degrees.of(90);

        private static final double WRIST_MOTOR_P = 0.5;
        private static final double WRIST_MOTOR_I = 0.0;
        private static final double WRIST_MOTOR_D = 0.0;
        private static final double WRIST_MOTOR_FF = 0.0;
        private static final double WRIST_MOTOR_MIN_OUTPUT = -1.0;
        private static final double WRIST_MOTOR_MAX_OUTPUT = 1.0;

        private double goalAngle;

        public CoralWristHardware() {
                wristSparkFlexConfig.inverted(WRIST_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.ALPHA_CORAL_WRIST_CURRENT_LIMIT.in(Amps));
                wristSparkFlexConfig.absoluteEncoder.positionConversionFactor(ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR)
                                .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
                                .inverted(ABSOLUTE_ENCODER_INVERTED).zeroCentered(true);

                wristSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
                                .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(ABSOLUTE_ENCODER_POSITION_WRAPPING)
                                .positionWrappingInputRange(ABSOLUTE_ENCODER_POSITION_MIN_INPUT.in(Radians),
                                                ABSOLUTE_ENCODER_POSITION_MAX_INPUT.in(Radians));

                coralIntakeWristSparkFlex = new SparkFlex(MotorIdConstants.CORAL_INTAKE_WRIST_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeWristSparkFlex.configure(wristSparkFlexConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeWristAbsoluteEncoder = coralIntakeWristSparkFlex.getAbsoluteEncoder();
                coralIntakeWristClosedLoopController = coralIntakeWristSparkFlex.getClosedLoopController();
        }

        @Override
        public void setGoalAngle(double desiredAngle) {
                coralIntakeWristClosedLoopController.setReference(desiredAngle, ControlType.kPosition,
                                ClosedLoopSlot.kSlot0,
                                coralWristFeedFoward.calculate(desiredAngle,
                                                coralIntakeWristAbsoluteEncoder.getVelocity()));
                goalAngle = desiredAngle;
        }

        // taking out motion profiling for now
        // public void setGoalStateTrapezoid(Angle angle) {
        // goalState.position = angle.in(Radians);
        // }

        @Override
        public void setWristSpeed(double speed) {
                coralIntakeWristSparkFlex.set(speed
                                + coralWristFeedFoward.calculate(coralIntakeWristAbsoluteEncoder.getPosition(), speed));
        }

        @Override
        public void updateStates(CoralWristIOStates states) {
                states.wristVelocity = coralIntakeWristAbsoluteEncoder.getVelocity();
                states.wristAppliedVoltage = coralIntakeWristSparkFlex.getAppliedOutput()
                                * coralIntakeWristSparkFlex.getBusVoltage();
                states.wristCurrent = coralIntakeWristSparkFlex.getOutputCurrent();
                states.wristAbsoluteEncoderAngle = coralIntakeWristAbsoluteEncoder.getPosition();
                states.goalAngle = goalAngle;
                // states.trapezoidProfileGoalAngle = goalState.position;
        }

        @Override
        public void periodic() {
                // setpointState = wristTrapezoidProfile.calculate(0.02,
                // setpointState, goalState);

        }

}