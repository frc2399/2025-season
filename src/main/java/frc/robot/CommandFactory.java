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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public enum RobotPosition {
    LEFT,
    RIGHT,
  }

  public enum Setpoint {
    L_ONE,
    L_TWO,
    L_THREE,
    L_FOUR,
    INTAKE,
    TURTLE,
    ELEVATOR_TOP_CRONCH_ZONE_INTERMEDIATE_SETPOINT,
    ELEVATOR_BOTTOM_CRONCH_ZONE_INTERMEDIATE_SETPOINT
  }

  public enum GameMode {
    CORAL,
    ALGAE
  }

  public RobotPosition robotPosition;
  public GameMode gameMode;
  public Setpoint setpoint;

  public Command turtleMode() {
    return Commands.sequence(
        Commands.parallel(coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE),
            algaeWrist.goToSetpointCommand(() -> Setpoint.TURTLE)),
        elevator.goToGoalSetpointCmd(() -> Setpoint.INTAKE, () -> GameMode.CORAL));
  }

  public Command elevatorBasedOnMode() {
    return Commands.either(
      moveElevatorAndAlgaeWrist(() -> getSetpoint()), 
      moveElevatorAndCoralWrist(() -> getSetpoint()), 
      () -> (getGameMode() == GameMode.ALGAE));
  }

public Command moveElevatorAndCoralWrist(Supplier<Setpoint> setpoint) {
    return Commands.either(avoidCronchCommand(setpoint),
        Commands.parallel(
          elevator.goToGoalSetpointCmd(setpoint, () -> GameMode.CORAL),
          coralWrist.goToSetpointCommand(setpoint)),
          () -> elevator.willCrossCronchZone(setpoint));
  }

  public Command moveElevatorAndAlgaeWrist(Supplier<Setpoint> setpoint) {
    return Commands.sequence(
      coralWrist.goToSetpointCommand(() -> Setpoint.L_ONE), 
      algaeWrist.goToSetpointCommand(setpoint),
      elevator.goToGoalSetpointCmd(setpoint, () -> GameMode.ALGAE));
  }

  public Command intakeBasedOnMode(Supplier<GameMode> gameMode) {
    return Commands.either(
      algaeIntake.intake(), 
      coralIntake.intake(), 
      () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command outtakeBasedOnMode(Supplier<GameMode> gameMode) {
    return Commands.either(
      algaeIntake.outtake(),
      coralIntake.outtake(),
      () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command avoidCronchCommand(Supplier<Setpoint> setpoint) {
    // if we're above cronch zone, start by setting elevator height to top of
    // collision range; if we're below, start by setting to bottom
    return Commands.either(
        Commands.sequence(
            Commands.parallel(
                elevator.goToGoalSetpointCmd(() -> Setpoint.ELEVATOR_TOP_CRONCH_ZONE_INTERMEDIATE_SETPOINT, () -> GameMode.CORAL),
                coralWrist.goToSetpointCommand(() -> Setpoint.L_ONE)),
            Commands.waitUntil(() -> coralWrist.atGoal()),
            elevator.goToGoalSetpointCmd(setpoint, () -> GameMode.CORAL),
            Commands.waitUntil(() -> elevator.atGoal()),
            coralWrist.goToSetpointCommand(setpoint)),
        Commands.sequence(
            Commands.parallel(
                elevator.goToGoalSetpointCmd(() -> Setpoint.ELEVATOR_BOTTOM_CRONCH_ZONE_INTERMEDIATE_SETPOINT, () -> GameMode.CORAL),
                coralWrist.goToSetpointCommand(() -> Setpoint.L_ONE)),
            Commands.waitUntil(() -> coralWrist.atGoal()),
            elevator.goToGoalSetpointCmd(setpoint, () -> GameMode.CORAL),
            Commands.waitUntil(() -> elevator.atGoal()),
            coralWrist.goToSetpointCommand(setpoint)),
        () -> (elevator.getCurrentPosition() > SetpointConstants.ELEVATOR_COLLISION_RANGE_TOP.in(Meters)));
  }  

  public Setpoint getSetpoint() {
    Setpoint setpoint;
    if (levelEntry.getString("None").equals("Level 1")) {
      setpoint = Setpoint.L_ONE;
    } else if (levelEntry.getString("None").equals("Level 2")) {
      setpoint = Setpoint.L_TWO;
    } else if (levelEntry.getString("None").equals("Level 3")) {
      setpoint = Setpoint.L_THREE;
    } else if (levelEntry.getString("None").equals("Level 4")) {
      setpoint = Setpoint.L_FOUR;
    } else {
      setpoint = Setpoint.L_ONE;
    }
    SmartDashboard.putString("networktablesData/setpoint", setpoint.toString());
    return setpoint;
  };

  public RobotPosition getRobotPosition() {
    if (leftRightEntry.getString("None").equals("left")) {
      robotPosition = RobotPosition.LEFT;
    } else if (leftRightEntry.getString("None").equals("right")) {
      robotPosition = RobotPosition.RIGHT;
    }
    SmartDashboard.putString("networktablesData/robotPosition", robotPosition.toString());
    return robotPosition;
  }

  public GameMode getGameMode() {
    if (gameModeEntry.getString("None").equals("coral")) {
      gameMode = GameMode.CORAL;
    } else if (gameModeEntry.getString("None").equals("algae")) {
      gameMode = GameMode.ALGAE;
    }
    SmartDashboard.putString("networktablesData/gameMode", gameMode.toString());
    return gameMode;
  }

  public void setGameMode(String level) {
    gameModeEntry.setString(level);
  }

  public void setScoringLevel(String level) {
    levelEntry.setString(level);
  }

  public void setRobotAlignmentPosition(String alignmentValue) {
    leftRightEntry.setString(alignmentValue);
  }
}