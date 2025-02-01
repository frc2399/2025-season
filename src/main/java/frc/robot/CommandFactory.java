package frc.robot;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CommandFactory {

    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;

    public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator) {
        this.drive = drive;
        this.elevator = elevator;
    }
}
