package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CommandFactory {

  private final DriveSubsystem drive;
  private final ElevatorSubsystem elevator;
  private final CoralWristSubsystem coralWrist;
  private final AlgaeWristSubsystem algaeWrist;
  // private final NetworkTableEntry ntEntry; //one for each entry we want to read
  // (state changes)
  private final NetworkTable scoringStateTables;
  private boolean indicator;
  // private final NetworkTableEntry newEntry;
  private final NetworkTableEntry levelEntry;
  private final NetworkTableEntry gameModeEntry;
  private final NetworkTableEntry leftRightEntry;

  private enum RobotPosition {
    LEFT,
    RIGHT,
  }

  public enum ScoringLevel {
    L_ONE,
    L_TWO,
    L_THREE,
    L_FOUR,
    INTAKE,
    ELEVATOR_TOP_INTERMEDIATE_SETPOINT,
    ELEVATOR_BOTTOM_INTERMEDIATE_SETPOINT
  }

  private enum GameMode {
    CORAL,
    ALGAE
  }

  private static RobotPosition robotPosition;
  private static ScoringLevel scoringLevel;
  private static GameMode gameMode;

  public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist, AlgaeWristSubsystem algaeWrist) {
    this.drive = drive;
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.algaeWrist = algaeWrist;
    scoringStateTables = NetworkTableInstance.getDefault().getTable("sidecarTable");
    // ntEntry = scoringStateTables.getEntry("GameMode"); //one for each key
    // newEntry = scoringStateTables.getEntry("Indicator");
    levelEntry = scoringStateTables.getEntry("scoringLevel");
    gameModeEntry = scoringStateTables.getEntry("gamePieceMode");
    leftRightEntry = scoringStateTables.getEntry("Position");
  }

  public Command turtleMode() {
    return avoidCronchCommand(() -> ScoringLevel.INTAKE);
  }

  public Command moveElevatorAndWrist() {
    return Commands.either(avoidCronchCommand(getScoringLevel()),
        Commands.parallel(elevator.goToGoalSetpointCmd(getScoringLevel()),
                          coralWrist.goToSetpointCommand(getScoringLevel())),
        () -> elevator.willCrossCronchZone(getScoringLevel()));
  }

  public Command avoidCronchCommand(Supplier<ScoringLevel> scoringLevel) {
    // if we're above cronch zone, start by setting elevator height to top of
    // collision range; if we're below, start by setting to bottom
    return Commands.either(
        Commands.sequence(
            Commands.parallel(
                elevator.goToGoalSetpointCmd(() -> ScoringLevel.ELEVATOR_TOP_INTERMEDIATE_SETPOINT),
                coralWrist.goToSetpointCommand(() -> ScoringLevel.L_ONE))
                .until(() -> coralWrist.atGoal()),
            Commands.parallel(elevator.goToGoalSetpointCmd(scoringLevel),
                              coralWrist.goToSetpointCommand(scoringLevel))),
        Commands.sequence(
            Commands.parallel(
                elevator.goToGoalSetpointCmd(() -> ScoringLevel.ELEVATOR_BOTTOM_INTERMEDIATE_SETPOINT),
                coralWrist.goToSetpointCommand(() -> ScoringLevel.L_ONE))
                .until(() -> coralWrist.atGoal()),
            Commands.parallel(elevator.goToGoalSetpointCmd(scoringLevel),
                              coralWrist.goToSetpointCommand(scoringLevel))),
        () -> (elevator.getCurrentPosition() > SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters)));
  }

  public Supplier<ScoringLevel> getScoringLevel() {
    if (levelEntry.getString("None").equals("Level 1")) {
      scoringLevel = ScoringLevel.L_ONE;
    } else if (levelEntry.getString("None").equals("Level 2")) {
      scoringLevel = ScoringLevel.L_TWO;
    } else if (levelEntry.getString("None").equals("Level 3")) {
      scoringLevel = ScoringLevel.L_THREE;
    } else if (levelEntry.getString("None").equals("Level 4")) {
      scoringLevel = ScoringLevel.L_FOUR;
    }
    return () -> scoringLevel;
  }

  public Supplier<RobotPosition> getRobotPosition() {
    if (leftRightEntry.getString("None").equals("left")) {
      robotPosition = RobotPosition.LEFT;
    } else if (leftRightEntry.getString("None").equals("right")) {
      robotPosition = RobotPosition.RIGHT;
    }
    return () -> robotPosition;
  }

    public Supplier<GameMode> getGameMode() {
        if (gameModeEntry.getString("None").equals("coral")) {
            gameMode = GameMode.CORAL;
          } else if (gameModeEntry.getString("None").equals("algae")) {
            gameMode = GameMode.ALGAE;
          }
        return () -> gameMode;
    }

    public void setScoringLevel(String level){
        levelEntry.setString(level);
    }

    public void setRobotAlignmentPosition(String alignmentValue){
        leftRightEntry.setString(alignmentValue);
    }

    public void setGameMode(String gameMode){
        gameModeEntry.setString(gameMode);
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