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
        private final SparkMax algaeIntakeSparkMax;
        private final SparkClosedLoopController algaeIntakeClosedLoopController;
        private final RelativeEncoder algaeIntakeEncoder;

        private static final SparkMaxConfig algaeIntakeSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
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

                algaeIntakeSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                algaeIntakeSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                algaeIntakeSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D, ALGAE_MOTOR_FF)
                                .outputRange(ALGAE_MOTOR_MIN_OUTPUT, ALGAE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED_SIDE_MOTORS);

                algaeIntakeSparkMax = new SparkMax(MotorIdConstants.ALGAE_BETA_INTAKE_CAN_ID, MotorType.kBrushless);

                algaeIntakeEncoder = algaeIntakeSparkMax.getEncoder();

                algaeIntakeSparkMax.configure(algaeIntakeSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeIntakeClosedLoopController = algaeIntakeSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(double speed) {
                algaeIntakeSparkMax.set(speed);
        }

        public double getVelocity() {
                return algaeIntakeEncoder.getVelocity();
        }

        public double getCurrent() {
                return algaeIntakeSparkMax.getOutputCurrent();
        }

        @Override
        public void updateStates(AlgaeIntakeIOStates states) {
                states.intakeVelocity = getVelocity();
                states.leftAppliedVoltage = algaeIntakeSparkMax.getAppliedOutput()
                                * algaeIntakeSparkMax.getBusVoltage();                
                states.leftCurrent = algaeIntakeSparkMax.getOutputCurrent();
        }

}