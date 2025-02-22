package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;
import static edu.wpi.first.units.Units.RPM;

public class AlgaeIntakeHardware implements AlgaeIntakeIO {
        private final SparkMax algaeIntakeSparkMax;
        private final SparkClosedLoopController algaeIntakeClosedLoopController;
        private final RelativeEncoder algaeIntakeEncoder;

        private static final SparkMaxConfig algaeIntakeSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        // gearbox ratio 9:1
        private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / 9; // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 9 / 60.0; // radians per second

        private static final double ALGAE_MOTOR_P = 0.001;
        private static final double ALGAE_MOTOR_I = 0;
        private static final double ALGAE_MOTOR_D = 0;
        private static final double ALGAE_MOTOR_FF = 0;
        private static final double ALGAE_MOTOR_MIN_OUTPUT = -1;
        private static final double ALGAE_MOTOR_MAX_OUTPUT = 1;

        private static final boolean POSITION_WRAPPING_ENABLED = true;

        private static final Current ALGAE_INTAKE_STALL_THRESHOLD = Amps.of(10);
        private static final Time ALGAE_INTAKE_STALL_TIME = Seconds.of(1);

        private static final Debouncer algaeIntakeDebouncer = new Debouncer(ALGAE_INTAKE_STALL_TIME.in(Seconds));

        public AlgaeIntakeHardware() {
                algaeIntakeSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                algaeIntakeSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                algaeIntakeSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D, ALGAE_MOTOR_FF)
                                .outputRange(ALGAE_MOTOR_MIN_OUTPUT, ALGAE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED);

                algaeIntakeSparkMax = new SparkMax(MotorIdConstants.ALGAE_BETA_INTAKE_CAN_ID, MotorType.kBrushless);

                algaeIntakeEncoder = algaeIntakeSparkMax.getEncoder();

                algaeIntakeSparkMax.configure(algaeIntakeSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeIntakeClosedLoopController = algaeIntakeSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(AngularVelocity speed) {
                algaeIntakeClosedLoopController.setReference(speed.in(RadiansPerSecond), ControlType.kVelocity);
        }

        public double getVelocity() {
                return algaeIntakeEncoder.getVelocity();
        }

        public void intake() {
                algaeIntakeClosedLoopController.setReference(SpeedConstants.ALPHA_ALGAE_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
                
        }
        public void outtake() {
                algaeIntakeClosedLoopController.setReference(SpeedConstants.ALPHA_ALGAE_OUTTAKE_SPEED.in(RPM), ControlType.kVelocity);
        }

        public double getCurrent() {
                return algaeIntakeSparkMax.getOutputCurrent();
        }

        @Override
        public boolean isStalling() {
                return algaeIntakeDebouncer.calculate(getCurrent() > ALGAE_INTAKE_STALL_THRESHOLD.in(Amps));
        }

        @Override
        public void updateStates(AlgaeIntakeIOStates states) {
                states.intakeVelocity = getVelocity();
                states.leftAppliedVoltage = algaeIntakeSparkMax.getAppliedOutput()
                                * algaeIntakeSparkMax.getBusVoltage();
                states.leftCurrent = algaeIntakeSparkMax.getOutputCurrent();
        }
}