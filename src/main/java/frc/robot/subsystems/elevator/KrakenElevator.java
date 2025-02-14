package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class KrakenElevator implements ElevatorIO {

    public static final class KrakenElevatorConstants {
        private static final Distance METERS_PER_REVOLUTION = Inches.of(0.67); // (1/9)(1.92 * pi)
        private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(.25);
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
        private static final Voltage P_VALUE = Volts.of(2.0);
        private static final Voltage I_VALUE = Volts.of(0);
        private static final Voltage D_VALUE = Volts.of(0);
        private static final Voltage FEEDFORWARD_VALUE = Volts.of(1.0 / 917);
        private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(0.28); // calculated using recalc
        private static final double OUTPUT_RANGE_MIN_VALUE = -1;
        private static final double OUTPUT_RANGE_MAX_VALUE = 1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
        private static final Distance MAX_ELEVATOR_HEIGHT = Inches.of(34.25); // inches
        private static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 1; //TODO: tune!
        private static final Current KRAKEN_CURRENT_LIMIT = Amps.of(80); 
    }

    private TalonFX elevatorRightMotorFollower, elevatorLeftMotorLeader;
    private TalonFXConfigurator rightMotorFollowerConfigurator, leftMotorLeaderConfigurator;
    private TalonFXConfiguration globalMotorConfiguration, rightMotorFollowerConfiguration, leftMotorLeaderConfiguration;
    private RelativeEncoder leftEncoder;
    private double goalPosition;
    private TrapezoidProfile elevatorMotionProfile;
    private PositionVoltage elevatorPIDPositionControl; 
    private VelocityVoltage elevatorPIDVelocityControl; 

    public KrakenElevator() {
        elevatorRightMotorFollower = new TalonFX(MotorIdConstants.RIGHT_ELEVATOR_MOTOR_ID);
        elevatorLeftMotorLeader = new TalonFX(MotorIdConstants.LEFT_ELEVATOR_MOTOR_ID);

        rightMotorFollowerConfigurator = elevatorRightMotorFollower.getConfigurator();
        leftMotorLeaderConfigurator = elevatorLeftMotorLeader.getConfigurator();


        globalMotorConfiguration = new TalonFXConfiguration();

        globalMotorConfiguration.Feedback.withSensorToMechanismRatio(KrakenElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO); 

        globalMotorConfiguration.Slot0.kS = (KrakenElevatorConstants.FEEDFORWARD_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kG = (KrakenElevatorConstants.ARBITRARY_FF_GRAVITY_COMPENSATION).in(Volts);
        globalMotorConfiguration.Slot0.kP = (KrakenElevatorConstants.P_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kI = (KrakenElevatorConstants.I_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kD = (KrakenElevatorConstants.D_VALUE).in(Volts);

        globalMotorConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(KrakenElevatorConstants.MAX_ELEVATOR_HEIGHT.in(Meters));
        globalMotorConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
        
        globalMotorConfiguration.CurrentLimits.withStatorCurrentLimit(KrakenElevatorConstants.KRAKEN_CURRENT_LIMIT.in(Amps)); 


        elevatorLeftMotorLeader.setPosition(0);

        elevatorMotionProfile = new TrapezoidProfile(
                new Constraints(KrakenElevatorConstants.MAX_VEL.in(MetersPerSecond),
                        KrakenElevatorConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)));

        elevatorRightMotorFollower.setControl(new StrictFollower(elevatorLeftMotorLeader.getDeviceID()));
        
        leftMotorLeaderConfigurator.apply(globalMotorConfiguration);
        rightMotorFollowerConfigurator.apply(globalMotorConfiguration);

        //TODO: im assuming the inversions will be opposite but check!!
        rightMotorFollowerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorLeaderConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rightMotorFollowerConfigurator.apply(rightMotorFollowerConfiguration);
        leftMotorLeaderConfigurator.apply(leftMotorLeaderConfiguration); 

        elevatorPIDPositionControl = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void disableElevator() {
        elevatorLeftMotorLeader.set(0);
    }

    @Override
    public void setSpeed(double speed) {
        //add velocity control (see docs)
        //also add Kv
    }

    @Override
    public void setGoalPosition(double desiredPosition) {

    }

    @Override
    public void setEncoderPosition(double position) {
    }

    @Override
    public double getEncoderVelocity() {
        return 0;
    }

    @Override
    public double getEncoderPosition() {
        return 0;
    }

    @Override
    public void setPercentOutput(double percentOutput) {
    }

    @Override
    public void updateStates(ElevatorIOInputs inputs) {

    }
}
