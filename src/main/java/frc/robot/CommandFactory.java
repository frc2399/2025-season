package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CommandFactory {

    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final CoralWristSubsystem coralWrist;
    private final NetworkTableEntry ntEntry; //one for each entry we want to read (state changes)
    private final NetworkTable scoringStateTables;
    private boolean indicator;
    private final NetworkTableEntry newEntry;
    
        public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist) {
            this.drive = drive;
            this.elevator = elevator;
            this.coralWrist = coralWrist;
            scoringStateTables = NetworkTableInstance.getDefault().getTable("sidecarTable");
            ntEntry = scoringStateTables.getEntry("GameMode"); //one for each key
            newEntry = scoringStateTables.getEntry("Indicator");
        }
    
        public Command turtleMode() {
            return Commands
                    .parallel(elevator.goToGoalSetpointCmd(Constants.SetpointConstants.ELEVATOR_TURTLE_HEIGHT),
                            coralWrist.goToSetpointCommand((Constants.SetpointConstants.CORAL_TURTLE_ANGLE).in(Degrees)));
        }
    
        public Command testNumber() {
            return Commands
                .runOnce(() -> System.out.println(ntEntry.getDouble(0)));
        }
    
        public Command indicatorChange() {
          return Commands
            .runOnce(() -> {
              if (indicator = true) {
                indicator = false;
              } else {
                indicator = true;
              }
              System.out.println("Indicator is " + indicator);
              newEntry.setBoolean(indicator);
            });
    }
}
