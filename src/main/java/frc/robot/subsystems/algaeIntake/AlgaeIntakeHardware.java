package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class AlgaeIntakeHardware implements AlgaeIntakeIO {

        private final SparkMax algaeIntakeLeftSparkMax;
        private final SparkMax algaeIntakeRightSparkMax;

        private final SparkClosedLoopController algaeIntakeLeftClosedLoopController;
        private final SparkClosedLoopController algaeIntakeRightClosedLoopController;

        private final RelativeEncoder algaeIntakeLeftEncoder;
        private final RelativeEncoder algaeIntakeRightEncoder;

        private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final boolean RIGHT_MOTOR_INVERTED = false;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double ALGAE_MOTOR_P = 0;
        private static final double ALGAE_MOTOR_I = 0;
        private static final double ALGAE_MOTOR_D = 0;
        private static final double ALGAE_MOTOR_FF = 0;
        private static final double ALGAE_MOTOR_MIN_OUTPUT = -1;
        private static final double ALGAE_MOTOR_MAX_OUTPUT = 1;

        private static final boolean POSITION_WRAPPING_ENABLED_SIDE_MOTORS = false;

        public AlgaeIntakeHardware() {

                leftSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                leftSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D, ALGAE_MOTOR_FF)
                                .outputRange(ALGAE_MOTOR_MIN_OUTPUT, ALGAE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED_SIDE_MOTORS);

                rightSparkMaxConfig.inverted(RIGHT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                rightSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                rightSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D, ALGAE_MOTOR_FF)
                                .outputRange(ALGAE_MOTOR_MIN_OUTPUT, ALGAE_MOTOR_MAX_OUTPUT);

                algaeIntakeLeftSparkMax = new SparkMax(MotorIdConstants.ALGAE_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);
                algaeIntakeRightSparkMax = new SparkMax(MotorIdConstants.ALGAE_INTAKE_RIGHT_CAN_ID,
                                MotorType.kBrushless);

                algaeIntakeLeftEncoder = algaeIntakeLeftSparkMax.getEncoder();
                algaeIntakeRightEncoder = algaeIntakeRightSparkMax.getEncoder();

                algaeIntakeLeftSparkMax.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeIntakeRightSparkMax.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeIntakeLeftClosedLoopController = algaeIntakeLeftSparkMax.getClosedLoopController();
                algaeIntakeRightClosedLoopController = algaeIntakeRightSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(double speed) {
                algaeIntakeLeftSparkMax.set(speed);
                algaeIntakeRightSparkMax.set(speed);
        }

        public double getVelocity() {
                return algaeIntakeLeftEncoder.getVelocity();
        }

        public double getCurrent() {
                return algaeIntakeLeftSparkMax.getOutputCurrent();
        }

        @Override
        public void updateStates(AlgaeIntakeIOStates states) {
                states.intakeVelocity = getVelocity();
                states.leftAppliedVoltage = algaeIntakeLeftSparkMax.getAppliedOutput()
                                * algaeIntakeLeftSparkMax.getBusVoltage();
                states.rightAppliedVoltage = algaeIntakeRightSparkMax.getAppliedOutput()
                                * algaeIntakeRightSparkMax.getBusVoltage();
                states.leftCurrent = algaeIntakeLeftSparkMax.getOutputCurrent();
                states.rightCurrent = algaeIntakeRightSparkMax.getOutputCurrent();
        }

}