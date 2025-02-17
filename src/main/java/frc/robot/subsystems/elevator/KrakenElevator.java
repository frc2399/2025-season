package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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
        private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(1); //TODO: calculate on beta 
        private static final double OUTPUT_RANGE_MIN_VALUE = -1;
        private static final double OUTPUT_RANGE_MAX_VALUE = 1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
        private static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 2; // TODO: tune! make sure there is a factor of 2 bc for every inch of chain the elevator moves two inches
        private static final double kDt = 0.02;
        private static final Current KRAKEN_CURRENT_LIMIT = Amps.of(80);
    }

    private TalonFX elevatorRightMotorFollower, elevatorLeftMotorLeader;
    private TalonFXConfigurator rightMotorFollowerConfigurator, leftMotorLeaderConfigurator;
    private TalonFXConfiguration globalMotorConfiguration, rightMotorFollowerConfiguration,
            leftMotorLeaderConfiguration;
    private TrapezoidProfile elevatorMotionProfile;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State intermediateSetpointState = new TrapezoidProfile.State();
    private PositionVoltage closedLoopController;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0); 

    public KrakenElevator(Distance maxElevatorHeight) {
        elevatorRightMotorFollower = new TalonFX(MotorIdConstants.RIGHT_BETA_ELEVATOR_CAN_ID);
        elevatorLeftMotorLeader = new TalonFX(MotorIdConstants.LEFT_BETA_ELEVATOR_CAN_ID);

        rightMotorFollowerConfigurator = elevatorRightMotorFollower.getConfigurator();
        leftMotorLeaderConfigurator = elevatorLeftMotorLeader.getConfigurator();

        globalMotorConfiguration = new TalonFXConfiguration();
        rightMotorFollowerConfiguration = new TalonFXConfiguration();
        leftMotorLeaderConfiguration = new TalonFXConfiguration(); 

        globalMotorConfiguration.Feedback
                .withSensorToMechanismRatio(KrakenElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO);

        globalMotorConfiguration.Slot0.kS = 0;
        globalMotorConfiguration.Slot0.kG = (KrakenElevatorConstants.ARBITRARY_FF_GRAVITY_COMPENSATION).in(Volts);
        globalMotorConfiguration.Slot0.kV = (KrakenElevatorConstants.FEEDFORWARD_VALUE).in(Volts); // TODO: tune Kv
        globalMotorConfiguration.Slot0.kP = (KrakenElevatorConstants.P_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kI = (KrakenElevatorConstants.I_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kD = (KrakenElevatorConstants.D_VALUE).in(Volts);

        globalMotorConfiguration.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(maxElevatorHeight.in(Meters));
        globalMotorConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(false);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);

        globalMotorConfiguration.CurrentLimits
                .withStatorCurrentLimit(KrakenElevatorConstants.KRAKEN_CURRENT_LIMIT.in(Amps));

        elevatorLeftMotorLeader.setPosition(0);

        elevatorMotionProfile = new TrapezoidProfile(
                new Constraints(KrakenElevatorConstants.MAX_VEL.in(MetersPerSecond),
                        KrakenElevatorConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)));

        elevatorRightMotorFollower.setControl(new Follower(elevatorLeftMotorLeader.getDeviceID(), true));

        leftMotorLeaderConfigurator.apply(globalMotorConfiguration);
        rightMotorFollowerConfigurator.apply(globalMotorConfiguration);

        // TODO: check inversions 
        leftMotorLeaderConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rightMotorFollowerConfigurator.apply(rightMotorFollowerConfiguration);
        leftMotorLeaderConfigurator.apply(leftMotorLeaderConfiguration);

        closedLoopController = new PositionVoltage(0).withSlot(0);
    }

    public void resetSetpointsToCurrentPosition() {
        goalState.position = elevatorLeftMotorLeader.getPosition().getValueAsDouble();
        intermediateSetpointState.position = elevatorLeftMotorLeader.getPosition().getValueAsDouble();

        goalState.velocity = 0;
        intermediateSetpointState.velocity = 0;

    }

    @Override
    public void setGoalPosition(Distance newGoalPosition) {
        goalState.position = newGoalPosition.in(Meters);

    }

    @Override
    public void setIntermediateSetpoint(Distance position, double velocity) {
        intermediateSetpointState.position = position.in(Meters);
        intermediateSetpointState.velocity = velocity;
    }

    public void incrementGoalPosition(Distance changeInGoalPosition) {
        goalState.position += changeInGoalPosition.in(Meters);
    }

    @Override
    public void calculateNextIntermediateSetpoint() {
        intermediateSetpointState = elevatorMotionProfile.calculate(KrakenElevatorConstants.kDt,
                intermediateSetpointState, goalState);
        closedLoopController.Position = intermediateSetpointState.position; 
        closedLoopController.Velocity = intermediateSetpointState.velocity;
        elevatorLeftMotorLeader.setControl(velocityVoltage.withVelocity(0.1));
        //StatusCode status = elevatorLeftMotorLeader.setControl(closedLoopController.withPosition(intermediateSetpointState.position));
        //System.out.println(status.getName()); 
    }

    @Override
    public double getEncoderVelocity() {
        return elevatorLeftMotorLeader.getVelocity().getValueAsDouble();
    }

    @Override
    public double getEncoderPosition() {
        return elevatorLeftMotorLeader.getPosition().getValueAsDouble();
    }

    @Override
     public void setSpeedManualControl(double speed)
    {
        elevatorLeftMotorLeader.setControl(new DutyCycleOut(speed));
    }


    @Override
    public void updateStates(ElevatorIOInputs inputs) {
        inputs.position = getEncoderPosition();
        inputs.velocity = getEncoderVelocity();
        inputs.appliedVoltageRight = elevatorRightMotorFollower.getClosedLoopOutput().getValueAsDouble()
                * elevatorRightMotorFollower.getSupplyVoltage().getValueAsDouble();
        inputs.appliedVoltageLeft = elevatorLeftMotorLeader.getClosedLoopOutput().getValueAsDouble()
                * elevatorLeftMotorLeader.getSupplyVoltage().getValueAsDouble();
        // TODO: check if this is correct
        inputs.current = elevatorLeftMotorLeader.getSupplyCurrent().getValueAsDouble();
        inputs.goalPosition = goalState.position; 
        inputs.intermediateSetpointPosition = intermediateSetpointState.position; 
    }
}
