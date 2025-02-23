package frc.robot.subsystems.coralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class CoralIntakeAlphaHardware implements CoralIntakeIO {

        private final SparkMax coralIntakeLeftSparkMax;
        private final SparkMax coralIntakeRightSparkMax;

        private final SparkClosedLoopController coralIntakeLeftClosedLoopController;
        private final SparkClosedLoopController coralIntakeRightClosedLoopController;

        private final RelativeEncoder coralIntakeLeftEncoder;
        private final RelativeEncoder coralIntakeRightEncoder;

        private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final boolean RIGHT_MOTOR_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_ROLLER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double INTAKE_MOTOR_P = 0.125;
        private static final double INTAKE_MOTOR_I = 0.0;
        private static final double INTAKE_MOTOR_D = 0.0;
        private static final double INTAKE_MOTOR_FF = 0.1;
        private static final double INTAKE_MOTOR_MIN_OUTPUT = -1.0;
        private static final double INTAKE_MOTOR_MAX_OUTPUT = 1.0;

        private static final boolean POSITION_WRAPPING_ENABLED_SIDE_MOTORS = true;

        private static final Time ALPHA_CORAL_DEBOUNCER_TIME = Seconds.of(0.5);
        private static final Current CORAL_INTAKE_STALL_THRESHOLD = Amps.of(15);
        private static final Debouncer CORAL_ALPHA_DEBOUNCER = new Debouncer(ALPHA_CORAL_DEBOUNCER_TIME.in(Seconds));

        public CoralIntakeAlphaHardware() {
                leftSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                leftSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(INTAKE_MOTOR_P, INTAKE_MOTOR_I, INTAKE_MOTOR_D, INTAKE_MOTOR_FF)
                                .outputRange(INTAKE_MOTOR_MIN_OUTPUT, INTAKE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED_SIDE_MOTORS);

                rightSparkMaxConfig.inverted(RIGHT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                rightSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                rightSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(INTAKE_MOTOR_P, INTAKE_MOTOR_I, INTAKE_MOTOR_D, INTAKE_MOTOR_FF)
                                .outputRange(INTAKE_MOTOR_MIN_OUTPUT, INTAKE_MOTOR_MAX_OUTPUT);

                coralIntakeLeftSparkMax = new SparkMax(MotorIdConstants.CORAL_ALPHA_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);
                coralIntakeRightSparkMax = new SparkMax(MotorIdConstants.CORAL_ALPHA_INTAKE_RIGHT_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeLeftEncoder = coralIntakeLeftSparkMax.getEncoder();
                coralIntakeRightEncoder = coralIntakeRightSparkMax.getEncoder();

                coralIntakeLeftSparkMax.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeRightSparkMax.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                coralIntakeLeftClosedLoopController = coralIntakeLeftSparkMax.getClosedLoopController();
                coralIntakeRightClosedLoopController = coralIntakeRightSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(AngularVelocity speed) {
                coralIntakeRightSparkMax.set(speed.in(RPM));
                coralIntakeLeftSparkMax.set(speed.in(RPM));
        }

        public double getVelocity() {
                return coralIntakeLeftEncoder.getVelocity();
        }

        public double getCurrent() {
                return coralIntakeLeftSparkMax.getOutputCurrent();
        }

        @Override
        public void intake() {
                coralIntakeLeftClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
        }

        @Override 
        public void outtake() {
                System.out.println("outtake!");
                coralIntakeLeftClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_OUTTAKE_SPEED.in(RPM), ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_OUTTAKE_SPEED.in(RPM), ControlType.kVelocity);
        }

        @Override
        public void setZero() {
                coralIntakeLeftClosedLoopController.setReference(0, ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setReference(0, ControlType.kVelocity);
        }

        @Override
        public void keepCoral() {
                coralIntakeLeftClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_HOLDING_SPEED.in(RPM), ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_HOLDING_SPEED.in(RPM), ControlType.kVelocity);
        }

        @Override
        public boolean isStalling() {
                return (CORAL_ALPHA_DEBOUNCER.calculate(getCurrent() > CORAL_INTAKE_STALL_THRESHOLD.in(Amps)));
        }

        @Override
        public void updateStates(CoralIntakeIOStates states) {
                states.velocity = getVelocity();
                states.leftAppliedVoltage = coralIntakeLeftSparkMax.getAppliedOutput()
                                * coralIntakeLeftSparkMax.getBusVoltage();
                states.rightAppliedVoltage = coralIntakeRightSparkMax.getAppliedOutput()
                                * coralIntakeRightSparkMax.getBusVoltage();
                states.leftCurrent = coralIntakeLeftSparkMax.getOutputCurrent();
                states.rightCurrent = coralIntakeRightSparkMax.getOutputCurrent();
        }
}