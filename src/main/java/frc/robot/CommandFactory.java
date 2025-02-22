package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CommandFactory {



  private final DriveSubsystem drive;
  private final ElevatorSubsystem elevator;
  private final CoralWristSubsystem coralWrist;
  private final AlgaeWristSubsystem algaeWrist;
  private final AlgaeIntakeSubsystem algaeIntake;
  private final CoralIntakeSubsystem coralIntake;

  // private final NetworkTableEntry ntEntry; //one for each entry we want to read
  // (state changes)
  private final NetworkTable scoringStateTables = NetworkTableInstance.getDefault().getTable("sidecarTable");;
  // private final NetworkTableEntry newEntry;
  private final NetworkTableEntry levelEntry = scoringStateTables.getEntry("scoringLevel");
  private final NetworkTableEntry gameModeEntry = scoringStateTables.getEntry("gamePieceMode");
  private final NetworkTableEntry leftRightEntry = scoringStateTables.getEntry("Position");

  public CommandFactory(DriveSubsystem drive, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist,
      AlgaeWristSubsystem algaeWrist, AlgaeIntakeSubsystem algaeIntake, CoralIntakeSubsystem coralIntake) {
    this.drive = drive;
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.algaeWrist = algaeWrist;
    this.algaeIntake = algaeIntake;
    this.coralIntake = coralIntake;
    // ntEntry = scoringStateTables.getEntry("GameMode"); //one for each key
    // newEntry = scoringStateTables.getEntry("Indicator");
  }

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

  public enum GameMode {
    CORAL,
    ALGAE
  }

  public Supplier<RobotPosition> robotPosition;
  public Supplier<GameMode> gameMode;
  public Supplier<ScoringLevel> scoringLevel;

  // public Supplier<ScoringLevel> getScoringLevel = () -> {
  //   ScoringLevel scoringLevel;
  //   if (levelEntry.getString("None").equals("Level 1")) {
  //     scoringLevel = ScoringLevel.L_ONE;
  //   } else if (levelEntry.getString("None").equals("Level 2")) {
  //     scoringLevel = ScoringLevel.L_TWO;
  //   } else if (levelEntry.getString("None").equals("Level 3")) {
  //     scoringLevel = ScoringLevel.L_THREE;
  //   } else if (levelEntry.getString("None").equals("Level 4")) {
  //     scoringLevel = ScoringLevel.L_FOUR;
  //   } else {
  //     scoringLevel = ScoringLevel.L_ONE;
  //   }
  //   return scoringLevel;
  // };

  public Command turtleMode() {
    return Commands.sequence(coralWrist.goToSetpointCommand(() -> ScoringLevel.L_ONE),
        elevator.goToGoalSetpointCmd(() -> ScoringLevel.INTAKE));
  }

  // public Command moveElevatorAndWrist() {
  // return Commands.either(avoidCronchCommand(getScoringLevel()),
  // Commands.parallel(elevator.goToGoalSetpointCmd(getScoringLevel()),
  // coralWrist.goToSetpointCommand(getScoringLevel())),
  // () -> elevator.willCrossCronchZone(getScoringLevel()));
  // }

  public Command moveElevatorAndWrist(Supplier<ScoringLevel> sl) {
    return Commands.sequence(coralWrist.goToSetpointCommand(() -> ScoringLevel.L_ONE),
        elevator.goToGoalSetpointCmd(sl),
        coralWrist.goToSetpointCommand(sl));
  }

  public Command intakeBasedOnMode(Supplier<GameMode> gameMode){
    if (gameMode.get() == GameMode.CORAL){
      return coralIntake.intake();
    }
    else if (gameMode.get() == GameMode.ALGAE){
      return algaeIntake.intake();
    }
    return Commands.none();
  }

  public Command outtakeBasedOnMode(Supplier<GameMode> gameMode){
    if (gameMode.get() == GameMode.CORAL){
      return coralIntake.outtake();
    }
    else if (gameMode.get() == GameMode.ALGAE){
      return algaeIntake.outtake();
    }
    return Commands.none();
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

  // public RobotPosition getRobotPosition() {
  // if (leftRightEntry.getString("None").equals("left")) {
  // robotPosition = RobotPosition.LEFT;
  // } else if (leftRightEntry.getString("None").equals("right")) {
  // robotPosition = RobotPosition.RIGHT;
  // }
  // return () -> robotPosition;
  // }

  // public Supplier<GameMode> getGameMode() {
  // if (gameModeEntry.getString("None").equals("coral")) {
  // gameMode = GameMode.CORAL;
  // } else if (gameModeEntry.getString("None").equals("algae")) {
  // gameMode = GameMode.ALGAE;
  // }
  // return () -> gameMode;
  // }



  public void setRobotPosition(RobotPosition newRobotPosition) {
    robotPosition = () -> newRobotPosition;
  }

  public void setGameMode(GameMode newGameMode) {
    gameMode = () -> newGameMode;
  }

  public void setScoringLevel(ScoringLevel newScoringLevel) {
     scoringLevel= () -> newScoringLevel;
  }

  // public void setScoringLevel(String level) {
  //   levelEntry.setString(level);
  // }

  public void setRobotAlignmentPosition(String alignmentValue) {
    leftRightEntry.setString(alignmentValue);
  }

}