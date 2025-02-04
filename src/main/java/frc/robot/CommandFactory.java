package frc.robot;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;

public class CommandFactory {

    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final AlgaeIntakeSubsystem algaeIntake;
    private final AlgaeWristSubsystem algaeWrist;

    public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake,
            AlgaeWristSubsystem algaeWrist) {
        this.drive = drive;
        this.elevator = elevator;
        this.algaeIntake = algaeIntake;
        this.algaeWrist = algaeWrist;
    }
}
