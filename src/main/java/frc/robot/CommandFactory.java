package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeAlphaHardware;
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
    setGameMode("coral");
    setScoringLevel("Level 1");
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
    TURTLE,
    ZERO
  }

  public enum GameMode {
    CORAL,
    ALGAE
  }

  public RobotPosition robotPosition;
  public GameMode gameMode;
  public Setpoint setpoint;

  public Command turtleBasedOnMode() {
    return Commands.either(
        algaeTurtleMode(),
        coralTurtleMode(),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command coralTurtleMode() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            elevator.goToGoalSetpointCmd(() -> Setpoint.TURTLE, () -> GameMode.CORAL)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command algaeTurtleMode() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            elevator.goToGoalSetpointCmd(() -> Setpoint.TURTLE, () -> GameMode.ALGAE),
            algaeWrist.goToSetpointCommand(() -> Setpoint.TURTLE)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command elevatorBasedOnMode() {
    return Commands.either(
        moveElevatorAndAlgaeWrist(),
        moveElevatorAndCoralWrist(),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command moveElevatorAndCoralWrist() {
    return Commands.sequence(
      Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            coralWrist.goToSetpointCommand(() -> Setpoint.ZERO)),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            elevator.goToGoalSetpointCmd(() -> getSetpoint(), () -> GameMode.CORAL),
            coralWrist.goToSetpointCommand(() -> getSetpoint())));
  }

  public Command moveElevatorAndAlgaeWrist() {
    return Commands.sequence(
      coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
      Commands.waitUntil(() -> coralWrist.atGoal()),
        algaeWrist.goToSetpointCommand(() -> getSetpoint()),
        elevator.goToGoalSetpointCmd(() -> getSetpoint(), () -> GameMode.ALGAE));
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
        coralIntake.setOuttakeSpeed(() -> getSetpoint()),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command driveBasedOnElevatorHeight(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX)
  {
      return Commands.either(
        drive.driveCommand(
        () -> -(MathUtil.applyDeadband(
            leftY.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            leftX.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            rightX.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        true,
        DriveControlConstants.SLOW_DRIVE_FACTOR), 
        drive.driveCommand(
        () -> -(MathUtil.applyDeadband(
            leftY.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            leftX.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            rightX.getAsDouble(),
            DriveControlConstants.DRIVE_DEADBAND)),
        true,
        DriveControlConstants.DRIVE_FACTOR),
        () -> elevator.isElevatorHeightAboveSpeedLimitingThreshold());
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