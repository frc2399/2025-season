package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
                .parallel(elevator.goToSetPointCommand(Constants.SetpointConstants.ELEVATOR_TURTLE_HEIGHT.in(Meters)),
                        coralWrist.goToSetpointCommand(Constants.SetpointConstants.CORAL_TURTLE_ANGLE.in(Degrees)));
    }

    public Command L1Mode() {
        return Commands.parallel(elevator.goToSetPointCommand(Constants.SetpointConstants.L_ONE_HEIGHT.in(Meters)),
                coralWrist.goToSetpointCommand(Constants.SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Degrees)));
    }

    public Command L2Mode() {
        return Commands.parallel(elevator.goToSetPointCommand(Constants.SetpointConstants.L_TWO_HEIGHT.in(Meters)),
                coralWrist.goToSetpointCommand(Constants.SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Degrees)));
    }

    public Command L3Mode() {
        return Commands.parallel(elevator.goToSetPointCommand(Constants.SetpointConstants.L_THREE_HEIGHT.in(Meters)),
                coralWrist.goToSetpointCommand(Constants.SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Degrees)));
    }

    public Command L4Mode() {
        return Commands.parallel(elevator.goToSetPointCommand(Constants.SetpointConstants.L_FOUR_HEIGHT.in(Meters)),
                coralWrist.goToSetpointCommand(Constants.SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Degrees)));
    }

}
