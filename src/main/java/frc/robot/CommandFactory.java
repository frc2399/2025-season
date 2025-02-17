package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CommandFactory {

    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final CoralWristSubsystem coralWrist;

    public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist) {
        this.drive = drive;
        this.elevator = elevator;
        this.coralWrist = coralWrist;
    }

    public Command turtleMode() {
        return Commands
                .parallel(elevator.goToGoalSetpointCmd(Constants.SetpointConstants.ELEVATOR_TURTLE_HEIGHT),
                        coralWrist.goToSetpointCommand((Constants.SetpointConstants.CORAL_TURTLE_ANGLE).in(Degrees)));
    }

}
