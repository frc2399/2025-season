package frc.robot;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CommandFactory {
    private final ElevatorSubsystem elevator;

    public CommandFactory(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    public void disableSubstems()  {
        elevator.disableElevator();
    }
}
