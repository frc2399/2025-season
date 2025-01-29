package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();

    public static final class ElevatorConstants {}

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setEncoderPosition(0);
    }

    public Command disableElevator() {
        return this.run(() ->elevatorIO.disableElevator());
    }

    public Command setEncoderPositionCommand(double position) {
        return this.run(() -> elevatorIO.setEncoderPosition(position));
    }

    public Command goToSetPointCommand(double position) {
        return this.startEnd(() -> elevatorIO.setGoalPosition(position), () -> elevatorIO.setGoalPosition(0));
    }

    public Command setSpeedCommand(double speed) {
        return this.run(() -> elevatorIO.setSpeed(speed));
    }

    public Command setPercentOutputCommand(double percentOutput) {
        return this.startEnd(() -> elevatorIO.setPercentOutput(percentOutput), () -> elevatorIO.setPercentOutput(0));
    }

    @Override
    public void periodic() {
        elevatorIO.updateStates(inputs);
        SmartDashboard.putNumber("Elevator/position", inputs.position);
        SmartDashboard.putNumber("Elevator/velocity", inputs.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltageRight", inputs.appliedVoltageRight);
        SmartDashboard.putNumber("Elevator/appliedVoltageLeft", inputs.appliedVoltageLeft);
        SmartDashboard.putNumber("Elevator/positionSetPoint", inputs.positionSetPoint);
        SmartDashboard.putNumber("Elevator/output current", inputs.current);
    }

}
 