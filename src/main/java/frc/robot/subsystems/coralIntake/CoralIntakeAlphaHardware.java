package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class CoralIntakeAlphaHardware implements CoralIntakeIO {

        private final SparkMax coralIntakeLeftSparkMax;

        private final SparkClosedLoopController coralIntakeLeftClosedLoopController;

        private final RelativeEncoder coralIntakeLeftEncoder;

        private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_ROLLER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = ENCODER_ROLLER_POSITION_FACTOR / 60.0; // radians per second

        private static final double INTAKE_MOTOR_P = 0.00018;
        private static final double INTAKE_MOTOR_I = 0.0;
        private static final double INTAKE_MOTOR_D = 0.0;
        private static final double INTAKE_MOTOR_FF = 0.001;

        private static final Time ALPHA_CORAL_DEBOUNCER_TIME = Seconds.of(0.06);
        private static final Current CORAL_INTAKE_STALL_THRESHOLD = Amps.of(0.004);
        private static final Debouncer CORAL_ALPHA_DEBOUNCER = new Debouncer(ALPHA_CORAL_DEBOUNCER_TIME.in(Seconds));

        
        public CoralIntakeAlphaHardware() {
                leftSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                leftSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(INTAKE_MOTOR_P, INTAKE_MOTOR_I, INTAKE_MOTOR_D, INTAKE_MOTOR_FF);

                leftSparkMaxConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                coralIntakeLeftSparkMax = new SparkMax(MotorIdConstants.CORAL_ALPHA_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);

                coralIntakeLeftEncoder = coralIntakeLeftSparkMax.getEncoder();

                coralIntakeLeftSparkMax.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                coralIntakeLeftClosedLoopController = coralIntakeLeftSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(AngularVelocity speed) {
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
        }

        @Override
        public void setOuttakeSpeed(Setpoint setpoint) {
                double desiredVelocity = 0;
                if (setpoint == Setpoint.L_ONE) {
                    desiredVelocity = SpeedConstants.ALPHA_CORAL_L1_OUTTAKE_SPEED.in(RPM);
                    coralIntakeLeftClosedLoopController.setReference(desiredVelocity, ControlType.kVelocity);
                } else {
                     desiredVelocity = SpeedConstants.ALPHA_CORAL_OUTTAKE_SPEED.in(RPM);
                     coralIntakeLeftClosedLoopController.setReference(desiredVelocity, ControlType.kVelocity);
                }

                SmartDashboard.putNumber("coralIntake/desiredVelocity", desiredVelocity);
            }

        @Override
        public void setZero() {
                coralIntakeLeftClosedLoopController.setReference(0, ControlType.kVelocity);
        }

        @Override
        public boolean isStalling() {
                return (CORAL_ALPHA_DEBOUNCER.calculate(getCurrent() > CORAL_INTAKE_STALL_THRESHOLD.in(Amps)));
        }

        @Override
        public void passiveIntake() {
            if (!isStalling()) {
                coralIntakeLeftClosedLoopController.setReference(SpeedConstants.ALPHA_CORAL_PASSIVE_SPEED.in(RPM), ControlType.kVelocity);
            }
        }

        @Override
        public void updateStates(CoralIntakeIOStates states) {
                states.velocity = getVelocity();
                states.leftAppliedVoltage = coralIntakeLeftSparkMax.getAppliedOutput()
                                * coralIntakeLeftSparkMax.getBusVoltage();
                states.leftCurrent = coralIntakeLeftSparkMax.getOutputCurrent();
        }
}