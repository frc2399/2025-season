package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {

        public static final class ClimberConstants {
                // use motor current limit in constants
                private static final double P_VALUE = 0.0;
                private static final double I_VALUE = 0.0;
                private static final double D_VALUE = 0.0;

                private static final Distance UPPER_LIMIT = Inches.of(30.25);
                // INIT TO HERE
                private static final Distance LOWER_LIMIT = Inches.of(16);
                private static final Distance ZERO_POSITION = Inches.of(14.665);

                // TODO: check!
                private static final double CLIMBER_POSITION_CONVERSION_FACTOR = 1.5 * 1.0 / 100.0 * Math.PI;
                private static final double CLIMBER_VELOCITY_CONVERSION_FACTOR = CLIMBER_POSITION_CONVERSION_FACTOR
                                / 60.0;
                private static final double CLIMBER_VELOCITY_FF = 1.0
                                / (MotorConstants.NEO_FREE_SPEED.in(RPM) * CLIMBER_VELOCITY_CONVERSION_FACTOR);
                private static final int SERVO_CHANNEL = 0;

                private static final boolean LEFT_CLIMBER_INVERTED = false;
                private static final boolean RIGHT_CLIMBER_INVERTED = false;

                private static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        }

        private final SparkMax leftClimber = new SparkMax(MotorIdConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
        private final SparkMax rightClimber = new SparkMax(MotorIdConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
        private final Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL);

        private final SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        private final SparkMaxConfig rightClimberConfig = new SparkMaxConfig();

        private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
        private final SparkClosedLoopController climberClosedLoopController = leftClimber.getClosedLoopController();

        private Angle climberGoalAngle = Radians.of(0.0);
        private Angle servoGoalAngle = Radians.of(0.0);

        public ClimberHardware() {

                leftClimberConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED)
                                .idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

                rightClimberConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED)
                                .idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

                leftClimberConfig.encoder.positionConversionFactor(ClimberConstants.CLIMBER_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(ClimberConstants.CLIMBER_VELOCITY_CONVERSION_FACTOR);

                leftClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE,
                                                ClimberConstants.CLIMBER_VELOCITY_FF);

                leftClimberConfig.softLimit
                                .forwardSoftLimit(ClimberConstants.UPPER_LIMIT.in(Inches))
                                .forwardSoftLimitEnabled(false)
                                .reverseSoftLimit(ClimberConstants.LOWER_LIMIT.in(Inches))
                                .reverseSoftLimitEnabled(true);

                rightClimberConfig.follow(leftClimber.getDeviceId(), true);

                leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                // Will made me do this
                leftClimberEncoder.setPosition(ClimberConstants.ZERO_POSITION.in(Inches)); // inches
        }

        public void setSpeed(LinearVelocity speed) {
                climberClosedLoopController.setReference(speed.in(InchesPerSecond), ControlType.kVelocity);
                if (speed.equals(InchesPerSecond.zero())) {
                        climberServo.setAngle(90);
                        servoGoalAngle = Degrees.of(90);
                } else {
                        climberServo.setAngle(270);
                        servoGoalAngle = Degrees.of(270);
                }
        }

        public void updateStates(ClimberIOInputs inputs) {
                inputs.climberAngle = leftClimberEncoder.getPosition();
                inputs.climberVelocity = leftClimberEncoder.getVelocity();
                inputs.climberGoalAngle = climberGoalAngle.in(Degrees);

                inputs.servoAngle = climberServo.getAngle();
                inputs.servoVelocity = climberServo.getSpeed();
                inputs.servoGoalAngle = servoGoalAngle.in(Degrees);
        }

}
