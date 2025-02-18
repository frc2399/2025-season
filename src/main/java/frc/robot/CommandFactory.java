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
    //private final NetworkTableEntry ntEntry; //one for each entry we want to read (state changes)
    private final NetworkTable scoringStateTables;
    private boolean indicator;
    //private final NetworkTableEntry newEntry;
    private final NetworkTableEntry levelEntry;
    private final NetworkTableEntry gameModeEntry;
    private final NetworkTableEntry leftRightEntry;
    
    private enum RobotPosition {
      LEFT,
      RIGHT,
    }

    private enum ScoringLevel {
      L_ONE,
      L_TWO,
      L_THREE,
      L_FOUR
    }

    private enum GameMode {
      CORAL,
      ALGAE
    }

    private RobotPosition robotPosition;
    private ScoringLevel scoringLevel;
    private GameMode gameMode;

        public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist) {
            this.drive = drive;
            this.elevator = elevator;
            this.coralWrist = coralWrist;
            scoringStateTables = NetworkTableInstance.getDefault().getTable("sidecarTable");
            //ntEntry = scoringStateTables.getEntry("GameMode"); //one for each key
            //newEntry = scoringStateTables.getEntry("Indicator");
            levelEntry = scoringStateTables.getEntry("scoringLevel");
            gameModeEntry = scoringStateTables.getEntry("gamePieceMode");
            leftRightEntry = scoringStateTables.getEntry("Position"); 
            
            if (levelEntry.getString("None").equals("Level 1")) {
              scoringLevel = ScoringLevel.L_ONE;
            } else if (levelEntry.getString("None").equals("Level 2")) {
              scoringLevel = ScoringLevel.L_TWO;
            } else if (levelEntry.getString("None").equals("Level 3")) {
              scoringLevel = ScoringLevel.L_THREE;
            } else if (levelEntry.getString("None").equals("Level 4")) {
              scoringLevel = ScoringLevel.L_FOUR;
            }

            if (leftRightEntry.getString("None").equals("left")) {
              robotPosition = RobotPosition.LEFT;
            } else if (leftRightEntry.getString("None").equals("right")) {
              robotPosition = RobotPosition.RIGHT;
            }

            if (gameModeEntry.getString("None").equals("coral")) {
              gameMode = GameMode.CORAL;
            } else if (gameModeEntry.getString("None").equals("algae")) {
              gameMode = GameMode.ALGAE;
            }
        }
    
        public Command turtleMode() {
            return Commands
                    .parallel(elevator.goToGoalSetpointCmd(Constants.SetpointConstants.ELEVATOR_TURTLE_HEIGHT),
                            coralWrist.goToSetpointCommand((Constants.SetpointConstants.CORAL_TURTLE_ANGLE).in(Degrees)));
        }
    
        //These were test functions. I'd prefer to keep them now so I can reference how I did certain commands later. 
        //I'll eventually delete them
        // public Command testNumber() {
        //     return Commands
        //         .runOnce(() -> System.out.println(ntEntry.getDouble(0)));
        // }
    
    //     public Command indicatorChange() {
    //       return Commands
    //         .runOnce(() -> {
    //           if (indicator == true) {
    //             indicator = false;
    //           } else {
    //             indicator = true;
    //           }
    //           System.out.println("Indicator is " + indicator);
    //           newEntry.setBoolean(indicator);
    //         });
    // }
}

