package frc.robot.subsystems.algaeEjector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeEjectorHardware implements AlgaeEjectorIO {

        private final SparkMax algaeEjectorLeftSparkMax;
        private final SparkMax algaeEjectorRightSparkMax;

        private final SparkClosedLoopController algaeEjectorLeftClosedLoopController;
        private final SparkClosedLoopController algaeEjectorRightClosedLoopController;

        private final RelativeEncoder algaeEjectorLeftEncoder;
        private final RelativeEncoder algaeEjectorRightEncoder;

        private static final SparkMaxConfig LEFT_SPARK_MAX_CONFIG = new SparkMaxConfig();
        private static final SparkMaxConfig RIGHT_SPARK_MAX_CONFIG = new SparkMaxConfig();
        private static final boolean ENCODER_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double P = 1.0;
        private static final double I = 0;
        private static final double D = 0.001;
        private static final double FF = 0;
        private static final double MIN_OUTPUT = -1;
        private static final double MAX_OUTPUT = 1;

        public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);

        public AlgaeEjectorHardware(int ALGAE_EJECTOR_LEFT_CAN_ID, int ALGAE_EJECTOR_RIGHT_CAN_ID) {

                LEFT_SPARK_MAX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                LEFT_SPARK_MAX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                LEFT_SPARK_MAX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(P, I, D, FF)
                                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

                RIGHT_SPARK_MAX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                LEFT_SPARK_MAX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                LEFT_SPARK_MAX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(P, I, D, FF)
                                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

                algaeEjectorLeftSparkMax = new SparkMax(ALGAE_EJECTOR_LEFT_CAN_ID, MotorType.kBrushless);
                algaeEjectorRightSparkMax = new SparkMax(ALGAE_EJECTOR_RIGHT_CAN_ID, MotorType.kBrushless);

                algaeEjectorLeftEncoder = algaeEjectorLeftSparkMax.getEncoder();
                algaeEjectorRightEncoder = algaeEjectorRightSparkMax.getEncoder();

                algaeEjectorLeftSparkMax.configure(LEFT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeEjectorRightSparkMax.configure(RIGHT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeEjectorLeftClosedLoopController = algaeEjectorLeftSparkMax.getClosedLoopController();
                algaeEjectorRightClosedLoopController = algaeEjectorRightSparkMax.getClosedLoopController();

        }

        @Override
        public void setSpeed(double speed) {
                algaeEjectorLeftClosedLoopController.setReference(
                                speed * SpeedConstants.ALGAE_EJECTOR_MAX_SPEED_MPS.in(MetersPerSecond),
                                ControlType.kVelocity);
                algaeEjectorRightClosedLoopController.setReference(
                                speed * SpeedConstants.ALGAE_EJECTOR_MAX_SPEED_MPS.in(MetersPerSecond),
                                ControlType.kVelocity);
        }

        @Override
        public double getVelocity() {
                return algaeEjectorLeftEncoder.getVelocity();
        }

        @Override
        public double getCurrent() {
                return ((SparkBase) algaeEjectorRightEncoder).getOutputCurrent();
        }

        @Override
        public void updateStates(AlgaeEjectorIOStates states) {
                states.velocity = getVelocity();
                states.leftAppliedVoltage = algaeEjectorLeftSparkMax.getAppliedOutput()
                                * algaeEjectorLeftSparkMax.getBusVoltage();
                states.rightAppliedVoltage = algaeEjectorRightSparkMax.getAppliedOutput()
                                * algaeEjectorRightSparkMax.getBusVoltage();

                states.leftCurrent = algaeEjectorLeftSparkMax.getOutputCurrent();
                states.rightCurrent = algaeEjectorRightSparkMax.getOutputCurrent();
        }

}