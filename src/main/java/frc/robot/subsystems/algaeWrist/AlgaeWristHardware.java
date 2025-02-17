package frc.robot.subsystems.algaeWrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class AlgaeWristHardware implements AlgaeWristIO {

        private final double STATIC_FF_ALGAE = 0.0;
        private final double GRAVITY_FF_ALGAE = 0.0;
        private final double VELOCITY_FF_ALGAE = 0.0;
        private final Angle WRIST_ANGULAR_OFFSET = Degrees.of(90);

        private final ArmFeedforward algaeWristFeedFoward = new ArmFeedforward(STATIC_FF_ALGAE, GRAVITY_FF_ALGAE,
                        VELOCITY_FF_ALGAE);

        private final SparkMax algaeWristSparkMax;

        private final SparkClosedLoopController algaeWristClosedLoopController;
        private final AbsoluteEncoder algaeWristAbsoluteEncoder;
        private final RelativeEncoder algaeWristRelativeEncoder;
        private static final SparkMaxConfig wristSparkMaxConfig = new SparkMaxConfig();
        private static final boolean MOTOR_INVERTED = false;

        private static final boolean ABSOLUTE_ENCODER_INVERTED = false;

        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        private static final double ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 2.0;
        private static final double ABSOLUTE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 2 / 60.0;
        private static final double RELATIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / 40.0; // radians
        private static final double RELATIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 2400.0; // radians per second

        private static final double WRIST_MOTOR_P = 0.1;
        private static final double WRIST_MOTOR_I = 0;
        private static final double WRIST_MOTOR_D = 0;
        private static final double WRIST_MOTOR_FF = 0;
        private static final double WRIST_MOTOR_MIN_OUTPUT = -1;
        private static final double WRIST_MOTOR_MAX_OUTPUT = 1;

        private static final boolean POSITION_WRAPPING_ENABLED = true;
        private static final Angle POSITION_WRAPPING_MIN_INPUT = Degrees.of(-90);
        private static final Angle POSITION_WRAPPING_MAX_INPUT = Degrees.of(90);

        private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(0);
        private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-100);
        private static final boolean SOFT_LIMIT_ENABLED = true;

        private double goalAngle;

        public AlgaeWristHardware() {
                wristSparkMaxConfig.inverted(MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                wristSparkMaxConfig.absoluteEncoder.positionConversionFactor(ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR)
                                .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
                                .inverted(ABSOLUTE_ENCODER_INVERTED).zeroCentered(true);
                wristSparkMaxConfig.encoder.positionConversionFactor(RELATIVE_ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(RELATIVE_ENCODER_VELOCITY_FACTOR);
                wristSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF)
                                .outputRange(WRIST_MOTOR_MIN_OUTPUT, WRIST_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED)
                                .positionWrappingInputRange(POSITION_WRAPPING_MIN_INPUT.in(Radians),
                                                POSITION_WRAPPING_MAX_INPUT.in(Radians));

                wristSparkMaxConfig.softLimit
                                .forwardSoftLimit(FORWARD_SOFT_LIMIT.in(Radians))
                                .forwardSoftLimitEnabled(SOFT_LIMIT_ENABLED)
                                .reverseSoftLimit(REVERSE_SOFT_LIMIT.in(Radians))
                                .reverseSoftLimitEnabled(SOFT_LIMIT_ENABLED);

                algaeWristSparkMax = new SparkMax(MotorIdConstants.ALGAE_BETA_WRIST_CAN_ID, MotorType.kBrushless);

                algaeWristSparkMax.configure(wristSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                algaeWristAbsoluteEncoder = algaeWristSparkMax.getAbsoluteEncoder();
                algaeWristRelativeEncoder = algaeWristSparkMax.getEncoder();

                algaeWristRelativeEncoder.setPosition(algaeWristAbsoluteEncoder.getPosition());

                algaeWristClosedLoopController = algaeWristSparkMax.getClosedLoopController();

        }

        @Override
        public void setGoalAngle(double desiredAngle) {
                algaeWristClosedLoopController.setReference(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                                algaeWristFeedFoward.calculate(desiredAngle + WRIST_ANGULAR_OFFSET.in(Radians),
                                                algaeWristAbsoluteEncoder.getVelocity()));
                // the arm feedforward assumes horizontal = 0. ours is vertical (up) = 0, so add
                // 90 degrees to get us from encoder position to position for arm feedforward
                goalAngle = desiredAngle;
        }

        @Override
        public void setWristSpeed(double speed) {
                algaeWristSparkMax.set(speed
                                + algaeWristFeedFoward.calculate(algaeWristAbsoluteEncoder.getPosition()
                                                + WRIST_ANGULAR_OFFSET.in(Radians), speed));
        }

        @Override
        public void updateStates(AlgaeWristIOStates states) {
                states.wristVelocity = algaeWristAbsoluteEncoder.getVelocity();
                states.wristAppliedVoltage = algaeWristSparkMax.getAppliedOutput() * algaeWristSparkMax.getBusVoltage();
                states.wristCurrent = algaeWristSparkMax.getOutputCurrent();
                states.wristRelativeEncoderAngle = algaeWristRelativeEncoder.getPosition();
                states.wristAbsoluteEncoderAngle = algaeWristAbsoluteEncoder.getPosition();
                states.goalAngle = goalAngle;
        }

        @Override
        public void periodic() {
        }

}
