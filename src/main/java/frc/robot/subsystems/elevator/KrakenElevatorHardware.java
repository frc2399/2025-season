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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SetpointConstants;

public class KrakenElevatorHardware implements ElevatorIO {

    public static final class KrakenElevatorConstants {
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(1.6);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(12);
        private static final Voltage P_VALUE = Volts.of(24.0);
        private static final Voltage I_VALUE = Volts.of(0);
        private static final Voltage D_VALUE = Volts.of(0);
        private static final Voltage FEEDFORWARD_VALUE = Volts.of(1.0 / 917);
        private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(.25); // TODO: calculate on beta
        private static final Distance ELEVATOR_SENSOR_TO_MECHANISM_RATIO = Meters.of(53.40295);// Inches.of(1.35);
                                                                                               // //
                                                                                               // Inches.of(1.76).times(Math.PI
                                                                                               // * 2.0 * 1./15.);
                                                                                               // // gear ratio *
                                                                                               // sprocket
                                                                                               // circumference * 2
                                                                                               // bc elevator moves
                                                                                               // 2 inches per inch
                                                                                               // of chain
        private static final Distance ELEVATOR_ROTOR_TO_SENSOR_RATIO = Inches.of(1);
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

    public KrakenElevatorHardware(Distance maxElevatorHeight) {
        elevatorRightMotorFollower = new TalonFX(MotorIdConstants.RIGHT_BETA_ELEVATOR_CAN_ID);
        elevatorLeftMotorLeader = new TalonFX(MotorIdConstants.LEFT_BETA_ELEVATOR_CAN_ID);

        rightMotorFollowerConfigurator = elevatorRightMotorFollower.getConfigurator();
        leftMotorLeaderConfigurator = elevatorLeftMotorLeader.getConfigurator();

        globalMotorConfiguration = new TalonFXConfiguration();
        rightMotorFollowerConfiguration = new TalonFXConfiguration();
        leftMotorLeaderConfiguration = new TalonFXConfiguration();

        globalMotorConfiguration.Feedback
                .withSensorToMechanismRatio(KrakenElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO.in(Meters));
        globalMotorConfiguration.Feedback
                .withRotorToSensorRatio(KrakenElevatorConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO.in(Meters));

        globalMotorConfiguration.Slot0.kS = 0;
        globalMotorConfiguration.Slot0.kG = (KrakenElevatorConstants.ARBITRARY_FF_GRAVITY_COMPENSATION).in(Volts);
        globalMotorConfiguration.Slot0.kV = (KrakenElevatorConstants.FEEDFORWARD_VALUE).in(Volts); // TODO: tune Kv
        globalMotorConfiguration.Slot0.kP = (KrakenElevatorConstants.P_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kI = (KrakenElevatorConstants.I_VALUE).in(Volts);
        globalMotorConfiguration.Slot0.kD = (KrakenElevatorConstants.D_VALUE).in(Volts);

        globalMotorConfiguration.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(maxElevatorHeight.in(Meters));
        globalMotorConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
        globalMotorConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

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

        // rightMotorFollowerConfigurator.apply(rightMotorFollowerConfiguration);
        // leftMotorLeaderConfigurator.apply(leftMotorLeaderConfiguration);

        elevatorLeftMotorLeader.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightMotorFollower.setNeutralMode(NeutralModeValue.Brake);

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
        StatusCode status = elevatorLeftMotorLeader
                .setControl(closedLoopController.withPosition(intermediateSetpointState.position));
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
    public void setSpeedManualControl(double speed) {
        elevatorLeftMotorLeader.setControl(new DutyCycleOut(speed));
    }

    @Override
    public boolean willCrossCronchZone(ScoringLevel scoringLevel) {
        double currentPosition = getEncoderPosition();
        // if the enum is null somehow, nothing will move so will not cross cronch
        // (default value)
        double goalPosition = currentPosition;
        if (scoringLevel == ScoringLevel.INTAKE) {
            goalPosition = SetpointConstants.ELEVATOR_TURTLE_HEIGHT.in(Meters); // turtle mode = bottom, where intake is
        } else if (scoringLevel == ScoringLevel.L_ONE) {
            goalPosition = SetpointConstants.L_ONE_CORAL_HEIGHT.in(Meters);
        } else if (scoringLevel == ScoringLevel.L_TWO) {
            goalPosition = SetpointConstants.L_TWO_CORAL_HEIGHT.in(Meters);
        } else if (scoringLevel == ScoringLevel.L_THREE) {
            goalPosition = SetpointConstants.L_THREE_CORAL_HEIGHT.in(Meters);
        } else if (scoringLevel == ScoringLevel.L_FOUR) {
            goalPosition = SetpointConstants.L_FOUR_CORAL_HEIGHT.in(Meters);
        }

        // if currently above the cronch range and our goal is below, or if currently
        // below cronch range and our goal is above, return true
        if (currentPosition > SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters)) {
            return (goalPosition < SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters));
        } else if (currentPosition < SetpointConstants.ELEVATOR_COLLISION_RANGE_BOTTOM.in(Meters)) {
            return (goalPosition > SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters));
        } else {
            return true;
        }
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
