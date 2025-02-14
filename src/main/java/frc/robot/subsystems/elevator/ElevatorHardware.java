package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class ElevatorHardware implements ElevatorIO {

    public static final class ElevatorHardwareConstants {
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
    }

    private SparkFlex elevatorRightMotorFollower, elevatorLeftMotorLeader;
    private SparkFlexConfig globalMotorConfig, rightMotorConfigFollower, leftMotorConfigLeader;
    private SparkClosedLoopController leftClosedLoopController;
    private RelativeEncoder leftEncoder;
    private double goalPosition;

    public ElevatorHardware(Distance maxElevatorHeight) {

        globalMotorConfig = new SparkFlexConfig();
        rightMotorConfigFollower = new SparkFlexConfig();
        leftMotorConfigLeader = new SparkFlexConfig();

        elevatorRightMotorFollower = new SparkFlex(MotorIdConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorLeftMotorLeader = new SparkFlex(MotorIdConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        leftClosedLoopController = elevatorLeftMotorLeader.getClosedLoopController();

        leftEncoder = elevatorLeftMotorLeader.getEncoder();

        globalMotorConfig.encoder
            .positionConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION.in(Meters))
            .velocityConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION.in(Meters) / 60);
            
        globalMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorHardwareConstants.P_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
            .i(ElevatorHardwareConstants.I_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
            .d(ElevatorHardwareConstants.D_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1)
            .p(ElevatorHardwareConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .i(ElevatorHardwareConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .d(ElevatorHardwareConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
            .velocityFF(ElevatorHardwareConstants.FEEDFORWARD_VALUE.in(Volts), ClosedLoopSlot.kSlot1) 
            .outputRange(ElevatorHardwareConstants.OUTPUT_RANGE_MIN_VALUE, ElevatorHardwareConstants.OUTPUT_RANGE_MAX_VALUE, ClosedLoopSlot.kSlot1);
     
        globalMotorConfig.softLimit
            .forwardSoftLimit((maxElevatorHeight).in(Meters) - 0.02) //a little less than max height for safety
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(false);

        leftMotorConfigLeader
                .apply(globalMotorConfig)
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        rightMotorConfigFollower
                .follow(MotorIdConstants.LEFT_ELEVATOR_MOTOR_ID, true)
                .apply(globalMotorConfig)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        elevatorLeftMotorLeader.configure(leftMotorConfigLeader, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        elevatorRightMotorFollower.configure(rightMotorConfigFollower, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void disableElevator() {
        elevatorLeftMotorLeader.set(0);
    }

    @Override
    public void setSpeed(double speed) {
        leftClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1,
                ElevatorHardwareConstants.ARBITRARY_FF_GRAVITY_COMPENSATION.in(Volts));
        // leftClosedLoopController.setReference(speed,
        // SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setGoalPosition(double desiredPosition) {
        leftClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                ElevatorHardwareConstants.ARBITRARY_FF_GRAVITY_COMPENSATION.in(Volts));
        // leftClosedLoopController.setReference(position,
        // SparkBase.ControlType.kMAXMotionPositionControl);
        goalPosition = desiredPosition;
    }

    @Override
    public void setEncoderPosition(double position) {
        leftEncoder.setPosition(position);
    }

    @Override
    public double getEncoderVelocity() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return leftEncoder.getPosition();
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        // elevatorLeftMotorLeader.set(percentOutput);
        leftClosedLoopController.setReference(percentOutput, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0,
                ElevatorHardwareConstants.ARBITRARY_FF_GRAVITY_COMPENSATION.in(Volts));
    }

    @Override
    public void updateStates(ElevatorIOInputs inputs) {
        inputs.position = getEncoderPosition();
        inputs.velocity = getEncoderVelocity();
        inputs.appliedVoltageRight = elevatorRightMotorFollower.getAppliedOutput()
                * elevatorRightMotorFollower.getBusVoltage();
        inputs.appliedVoltageLeft = elevatorLeftMotorLeader.getAppliedOutput()
                * elevatorLeftMotorLeader.getBusVoltage();
        inputs.positionSetPoint = goalPosition;
        inputs.current = elevatorLeftMotorLeader.getOutputCurrent();
    }
}
