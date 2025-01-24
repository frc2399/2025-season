package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants;

public class ElevatorHardware implements ElevatorIO {

    public static final class ElevatorHardwareConstants {
        private static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        private static final int RIGHT_ELEVATOR_MOTOR_ID = 10;
        private static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
        private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(1); 
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
        private static final double P_VALUE = 0.1;
        private static final double I_VALUE = 0;
        private static final double D_VALUE = 0.1;
        private static final double FEEDFORWARD_VALUE = 1.0 / 917;
        private static final double OUTPUTRANGE_MIN_VALUE = -1;
        private static final double OUTPUTRANGE_MAX_VALUE = 1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
    }

    private SparkFlex elevatorRightMotorFollower, elevatorLeftMotorLeader;
    private SparkFlexConfig globalMotorConfig, rightMotorConfigFollower, leftMotorConfigLeader;
    private SparkClosedLoopController leftClosedLoopController;
    private RelativeEncoder rightEncoder, leftEncoder;
    private double position;
    private TrapezoidProfile elevatorMotionProfile;
    private State currentState = new State();
    private State goalState = new State();
    
    public ElevatorHardware() {

        globalMotorConfig = new SparkFlexConfig();
        rightMotorConfigFollower = new SparkFlexConfig();
        leftMotorConfigLeader = new SparkFlexConfig();

        elevatorRightMotorFollower = new SparkFlex(ElevatorHardwareConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorLeftMotorLeader = new SparkFlex(ElevatorHardwareConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        leftClosedLoopController = elevatorLeftMotorLeader.getClosedLoopController();

        //rightEncoder = elevatorRightMotorFollower.getEncoder();
        leftEncoder = elevatorLeftMotorLeader.getEncoder();

        elevatorMotionProfile = new TrapezoidProfile(new Constraints(ElevatorHardwareConstants.MAX_ACCEL.in(MetersPerSecondPerSecond), ElevatorHardwareConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)));

        globalMotorConfig.encoder
            .positionConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION)
            .velocityConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION / 60);
            
        globalMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorHardwareConstants.P_VALUE, ClosedLoopSlot.kSlot0)
            .i(ElevatorHardwareConstants.I_VALUE, ClosedLoopSlot.kSlot0)
            .d(ElevatorHardwareConstants.D_VALUE, ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1)
            .p(ElevatorHardwareConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .i(ElevatorHardwareConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .d(ElevatorHardwareConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
            .velocityFF(ElevatorHardwareConstants.FEEDFORWARD_VALUE, ClosedLoopSlot.kSlot1) 
            .outputRange(ElevatorHardwareConstants.OUTPUTRANGE_MIN_VALUE, ElevatorHardwareConstants.OUTPUTRANGE_MAX_VALUE, ClosedLoopSlot.kSlot1);
        globalMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorHardwareConstants.MAX_VEL.in(MetersPerSecond))
            .maxAcceleration(ElevatorHardwareConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))
            .allowedClosedLoopError(ElevatorHardwareConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

        leftMotorConfigLeader
            .apply(globalMotorConfig)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.NEO_VORTEX_CURRENT_LIMIT);

        rightMotorConfigFollower
            .follow(ElevatorHardwareConstants.LEFT_ELEVATOR_MOTOR_ID, true)
            .apply(globalMotorConfig)
            .smartCurrentLimit(MotorConstants.NEO_VORTEX_CURRENT_LIMIT);

        elevatorLeftMotorLeader.configure(leftMotorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorRightMotorFollower.configure(rightMotorConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        leftClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        //leftClosedLoopController.setReference(speed, SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setPosition(double position) {
        goalState.position = position;
        leftClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        //leftClosedLoopController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
        
        this.position = position;
    }

    @Override
    public void setEncoderPosition(double position) {
        leftEncoder.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return leftEncoder.getPosition();
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        elevatorLeftMotorLeader.set(percentOutput);
    }

    @Override
    public void updateStates(ElevatorIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.appliedVoltageRight = elevatorRightMotorFollower.getAppliedOutput() * elevatorRightMotorFollower.getBusVoltage();
        inputs.appliedVoltageLeft = elevatorLeftMotorLeader.getAppliedOutput() * elevatorLeftMotorLeader.getBusVoltage();
        inputs.positionSetPoint = position;
    }
}
