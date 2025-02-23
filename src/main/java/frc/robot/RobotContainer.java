// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandFactory.GameMode;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private final ElevatorSubsystem elevator = subsystemFactory.buildElevator();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  private static SendableChooser<Command> autoChooser;
  private ComplexWidget autonChooserWidget;
  private final Field2d field;
  private final CoralIntakeSubsystem coralIntake = subsystemFactory.buildCoralIntake();
  private final CoralWristSubsystem coralWrist = subsystemFactory.buildCoralWrist();
  private final AlgaeIntakeSubsystem algaeIntake = subsystemFactory.buildAlgaeIntake();
  private final AlgaeWristSubsystem algaeWrist = subsystemFactory.buildAlgaeWrist();

  // this is public because we need to run the visionPoseEstimator periodic from
  // Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive);
  public CommandFactory commandFactory = new CommandFactory(drive, elevator, coralWrist, algaeWrist, algaeIntake,
      coralIntake);

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(
      DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });
    configureDefaultCommands();
    configureButtonBindingsDriver();
    configureButtonBindingsOperator();
    setUpAuton();
  }

  public void disableSubsystems() {
    elevator.profiledPIDEnabled = false;
  }

  public void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveCommand(
        () -> -(MathUtil.applyDeadband(
            driverController.getLeftY(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            driverController.getLeftX(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            driverController.getRightX(),
            DriveControlConstants.DRIVE_DEADBAND)),
        DriveControlConstants.FIELD_ORIENTED_DRIVE));
    coralIntake.setDefaultCommand(coralIntake.setZero());
    algaeIntake.setDefaultCommand(algaeIntake.setRollerSpeed(RPM.of(0)));
    coralWrist.setDefaultCommand(coralWrist.setWristSpeed(0).withName("coral Wrist default"));
    // elevator.setDefaultCommand(elevator.setSpeedManualControl(0));
  }

  private void configureButtonBindingsDriver() {
    driverController.rightTrigger().whileTrue(commandFactory.intakeBasedOnMode(()-> commandFactory.gameMode));
    driverController.leftTrigger().whileTrue(commandFactory.outtakeBasedOnMode(()->commandFactory.gameMode));

    driverController.rightBumper().onTrue(commandFactory.elevatorBasedOnMode());

    driverController.y().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());
    driverController.a().onTrue(commandFactory.turtleMode());
    driverController.b().onTrue(coralWrist.goToSetpointCommand(() -> Setpoint.INTAKE));
  }

  private void setUpAuton() {
    NamedCommands.registerCommand("intake coral", Commands.print("intake coral"));
    NamedCommands.registerCommand("Score coral on L1", Commands.print("coral scored on L1"));
    NamedCommands.registerCommand("Score coral on L2", Commands.print("coral scored on L2"));
    NamedCommands.registerCommand("Score coral on L4", Commands.print("coral scored on L4"));
    NamedCommands.registerCommand("Elevator to L1 setpoint", elevator.goToGoalSetpointCmd(() -> Setpoint.L_ONE, () -> GameMode.CORAL).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L2 setpoint", elevator.goToGoalSetpointCmd(() -> Setpoint.L_TWO, () -> GameMode.CORAL).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L3 setpoint", elevator.goToGoalSetpointCmd(() -> Setpoint.L_THREE, () -> GameMode.CORAL).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L4 setpoint", elevator.goToGoalSetpointCmd(() -> Setpoint.L_FOUR, () -> GameMode.CORAL).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Coral wrist to L1 setpoint", coralWrist.goToSetpointCommand(() -> Setpoint.L_ONE).withTimeout(1));
    NamedCommands.registerCommand("Coral wrist to L2 setpoint", coralWrist.goToSetpointCommand(() -> Setpoint.L_TWO).withTimeout(1));
    NamedCommands.registerCommand("Coral wrist to L3 setpoint", coralWrist.goToSetpointCommand(() -> Setpoint.L_THREE).withTimeout(1));
    NamedCommands.registerCommand("Coral wrist to L4 setpoint", coralWrist.goToSetpointCommand(() -> Setpoint.L_FOUR).withTimeout(1));
    NamedCommands.registerCommand("Outtake coral", coralIntake.outtake().andThen(Commands.waitSeconds(1)));
    NamedCommands.registerCommand("Elevator and coral wrist to L1 setpoint", commandFactory.moveElevatorAndCoralWrist(() -> Setpoint.L_ONE));
    NamedCommands.registerCommand("Elevator and coral wrist to L2 setpoint", commandFactory.moveElevatorAndCoralWrist(() -> Setpoint.L_TWO));
    NamedCommands.registerCommand("Elevator and coral wrist to L3 setpoint", commandFactory.moveElevatorAndCoralWrist(() -> Setpoint.L_THREE));
    NamedCommands.registerCommand("Elevator and coral wrist to L4 setpoint", commandFactory.moveElevatorAndCoralWrist(() -> Setpoint.L_FOUR));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos/Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureButtonBindingsOperator() {
    // these buttons should not be changed for local testing and should function as
    // a replacement gamepad
    operatorController.a().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level One")));
    operatorController.b().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level Two")));
    operatorController.x().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level Three")));
    operatorController.y().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level Four")));

    operatorController.rightBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("right")));
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("left")));

    operatorController.leftTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("algae")));
    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("coral")));

    // place local buttons below here, delete before PRing

  }
}
