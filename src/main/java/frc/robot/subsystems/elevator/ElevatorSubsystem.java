package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOStates;;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO elevatorIO;
    private final ElevatorIOStates states = new ElevatorIOStates();
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

    public Command goToGoalSetpointCmd(Distance position) {
        return this.runOnce(() -> {
            elevatorIO.setGoalPosition(position);
            profiledPIDEnabled = true;
            goalSetpoint = position.in(Meters);
        });
    }

    public boolean atGoal() {
        return (Math.abs(goalSetpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

    public Command incrementGoalPosition(Distance changeInGoalPosition) {
        profiledPIDEnabled = true;
        return this.run(() -> elevatorIO.incrementGoalPosition(changeInGoalPosition));
    }

    public double getCurrentPosition() {
        return elevatorIO.getEncoderPosition();
    }

    public boolean willCrossCronchZone(ScoringLevel scoringLevel) {
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
