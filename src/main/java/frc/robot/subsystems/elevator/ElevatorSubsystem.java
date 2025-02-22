package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs states = new ElevatorIOInputs();
    public boolean profiledPIDEnabled = false;
    private double goalSetpoint;
    private final Distance HEIGHT_TOLERANCE = Inches.of(0.5);
    private double JOYSTICK_INPUT_TO_CHANGE_IN_POSITION_CONVERSION_FACTOR = 0.002;

    public static final class ElevatorConstants {
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setIntermediateSetpoint(Meters.of(0), 0);
    }

    public Command goToGoalSetpointCmd(Supplier<ScoringLevel> scoringLevel) {
        return this.runOnce(() -> {
            if (scoringLevel.get() == ScoringLevel.INTAKE) {
                elevatorIO.setGoalPosition(SetpointConstants.ELEVATOR_TURTLE_HEIGHT); // turtle mode = bottom, where intake is
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.ELEVATOR_TURTLE_HEIGHT.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.L_ONE) {
                elevatorIO.setGoalPosition(SetpointConstants.L_ONE_HEIGHT); // turtle mode = bottom, where intake is
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.L_ONE_HEIGHT.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.L_TWO) {
                elevatorIO.setGoalPosition(SetpointConstants.L_TWO_HEIGHT);
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.L_TWO_HEIGHT.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.L_THREE) {
                elevatorIO.setGoalPosition(SetpointConstants.L_THREE_HEIGHT);
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.L_THREE_HEIGHT.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.L_FOUR) {
                elevatorIO.setGoalPosition(SetpointConstants.L_FOUR_HEIGHT);
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.L_FOUR_HEIGHT.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.ELEVATOR_TOP_INTERMEDIATE_SETPOINT) {
                elevatorIO.setGoalPosition(SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP);
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters); 
            } else if (scoringLevel.get() == ScoringLevel.ELEVATOR_BOTTOM_INTERMEDIATE_SETPOINT) {
                elevatorIO.setGoalPosition(SetpointConstants.ELEVATOR_COLLISION_RANGE_BOTTOM);
                profiledPIDEnabled = true;
                goalSetpoint = SetpointConstants.ELEVATOR_COLLISION_RANGE_BOTTOM.in(Meters); 
            }  
            // if the enum is null, do nothing          
        });
    }

    public boolean atGoal() {
        return (Math.abs(goalSetpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

    public Command incrementGoalPosition(Distance changeInGoalPosition)
    {
        return this.run(()-> {
            profiledPIDEnabled = true;
            elevatorIO.incrementGoalPosition(changeInGoalPosition);
        });
    }

    public double getCurrentPosition() {
        return elevatorIO.getEncoderPosition();
    }

    public Command setSpeedManualControl(double speed)
    {
        return this.run(() -> elevatorIO.setSpeedManualControl(speed)); 
    }

    public boolean willCrossCronchZone(Supplier<ScoringLevel> scoringLevel) {
        return elevatorIO.willCrossCronchZone(scoringLevel);
    }


    @Override
    public void periodic() {
        if (!profiledPIDEnabled) {
            elevatorIO.resetSetpointsToCurrentPosition();
        }
       elevatorIO.calculateNextIntermediateSetpoint();

        elevatorIO.updateStates(states);
        SmartDashboard.putNumber("Elevator/position", states.position);
        SmartDashboard.putNumber("Elevator/velocity", states.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltageRight", states.appliedVoltageRight);
        SmartDashboard.putNumber("Elevator/appliedVoltageLeft", states.appliedVoltageLeft);
        SmartDashboard.putNumber("Elevator/goalStatePosition", states.goalPosition);
        SmartDashboard.putNumber("Elevator/output current", states.current);
        SmartDashboard.putNumber("Elevator/intermediate setpoint position", states.intermediateSetpointPosition);
        SmartDashboard.putBoolean("Elevator/profiled PID enabled", profiledPIDEnabled);
    }

}
