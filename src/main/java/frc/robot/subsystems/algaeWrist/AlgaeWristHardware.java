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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SetpointConstants;

public class AlgaeWristHardware implements AlgaeWristIO {

        private final double STATIC_FF_ALGAE = 0.0;
        private final double GRAVITY_FF_ALGAE = 0.015;
        private final double VELOCITY_FF_ALGAE = 0.50;
        private final Angle WRIST_ANGULAR_OFFSET = Degrees.of(90);

        private final ArmFeedforward algaeWristFeedFoward = new ArmFeedforward(STATIC_FF_ALGAE, GRAVITY_FF_ALGAE,
                        VELOCITY_FF_ALGAE);

        private final SparkFlex algaeWristSparkMax;

        private final SparkClosedLoopController algaeWristClosedLoopController;
        private final AbsoluteEncoder algaeWristAbsoluteEncoder;
        private final RelativeEncoder algaeWristRelativeEncoder;
        private static final SparkFlexConfig wristSparkMaxConfig = new SparkFlexConfig();

        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        private static final double ABSOLUTE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / 2.0;
        private static final double ABSOLUTE_ENCODER_VELOCITY_FACTOR = ABSOLUTE_ENCODER_POSITION_FACTOR / 60.0;
        private static final double RELATIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / 40.0; // radians
        private static final double RELATIVE_ENCODER_VELOCITY_FACTOR = RELATIVE_ENCODER_POSITION_FACTOR / 60; // radians
                                                                                                              // per
                                                                                                              // second

        private static final double WRIST_MOTOR_P = 1.0;
        private static final double WRIST_MOTOR_I = 0;
        private static final double WRIST_MOTOR_D = 0;
        private static final double WRIST_MOTOR_FF = 0;

        private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(0);
        private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-110);
        private static final boolean SOFT_LIMIT_ENABLED = true;

        private Angle goalAngle = Radians.of(0);

        public AlgaeWristHardware(boolean motorInversion, boolean absoluteEncoderInverison) {
                wristSparkMaxConfig.inverted(motorInversion).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                wristSparkMaxConfig.absoluteEncoder.positionConversionFactor(ABSOLUTE_ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
                                .inverted(absoluteEncoderInverison).zeroCentered(true);
                wristSparkMaxConfig.encoder.positionConversionFactor(RELATIVE_ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(RELATIVE_ENCODER_VELOCITY_FACTOR);
                wristSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(WRIST_MOTOR_P, WRIST_MOTOR_I, WRIST_MOTOR_D, WRIST_MOTOR_FF);

                wristSparkMaxConfig.softLimit
                                .forwardSoftLimit(FORWARD_SOFT_LIMIT.in(Radians))
                                .forwardSoftLimitEnabled(SOFT_LIMIT_ENABLED)
                                .reverseSoftLimit(REVERSE_SOFT_LIMIT.in(Radians))
                                .reverseSoftLimitEnabled(SOFT_LIMIT_ENABLED);

                wristSparkMaxConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                algaeWristSparkMax = new SparkFlex(MotorIdConstants.ALGAE_BETA_WRIST_CAN_ID, MotorType.kBrushless);

                algaeWristSparkMax.configure(wristSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                algaeWristAbsoluteEncoder = algaeWristSparkMax.getAbsoluteEncoder();
                algaeWristRelativeEncoder = algaeWristSparkMax.getEncoder();

                algaeWristRelativeEncoder.setPosition(algaeWristAbsoluteEncoder.getPosition());

                algaeWristClosedLoopController = algaeWristSparkMax.getClosedLoopController();
        }

        @Override
        public void resetRelativeToAbsolute() {
                algaeWristRelativeEncoder.setPosition(algaeWristAbsoluteEncoder.getPosition());
        }

        @Override
        public void setGoalAngle(Setpoint setpoint) {
                Angle desiredAngle = Radians.of(0);
                if (setpoint == Setpoint.L_ONE) {
                        desiredAngle = SetpointConstants.ALGAE_WRIST_INTAKE_ANGLE;
                } else if (setpoint == Setpoint.L_TWO || setpoint == Setpoint.L_THREE) {
                        desiredAngle = SetpointConstants.ALGAE_REEF_REMOVER_ANGLE;
                } else if (setpoint == Setpoint.TURTLE) {
                        desiredAngle = SetpointConstants.ALGAE_WRIST_TURTLE_ANGLE;
                } else if (setpoint == Setpoint.ZERO) {
                        desiredAngle = SetpointConstants.ALGAE_WRIST_ZERO_ANGLE;
                }
                algaeWristClosedLoopController.setReference(desiredAngle.in(Radians), ControlType.kPosition,
                                ClosedLoopSlot.kSlot0,
                                algaeWristFeedFoward.calculate(
                                                desiredAngle.in(Radians) + WRIST_ANGULAR_OFFSET.in(Radians),
                                                algaeWristAbsoluteEncoder.getVelocity()));
                // the arm feedforward assumes horizontal = 0. ours is vertical (up) = 0, so add
                // 90 degrees to get us from encoder position to position for arm feedforward
                goalAngle = desiredAngle;
        }

        @Override
        public void setWristSpeed(double speed) {
                algaeWristSparkMax.set(speed
                                + algaeWristFeedFoward.calculate(algaeWristRelativeEncoder.getPosition()
                                                + WRIST_ANGULAR_OFFSET.in(Radians), speed));
        }

        @Override
        public void updateStates(AlgaeWristIOStates states) {
                states.wristVelocity = algaeWristAbsoluteEncoder.getVelocity();
                states.wristAppliedVoltage = algaeWristSparkMax.getAppliedOutput() * algaeWristSparkMax.getBusVoltage();
                states.wristCurrent = algaeWristSparkMax.getOutputCurrent();
                states.wristRelativeEncoderAngle = algaeWristRelativeEncoder.getPosition();
                states.wristAbsoluteEncoderAngle = algaeWristAbsoluteEncoder.getPosition();
                states.goalAngle = goalAngle.in(Radians);
        }

        @Override
        public void periodic() {
        }
}