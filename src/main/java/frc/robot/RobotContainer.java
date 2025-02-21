// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
=======
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
=======
>>>>>>> main
import edu.wpi.first.wpilibj2.command.Commands;
>>>>>>> main
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.vision.*;
import static edu.wpi.first.units.Units.*;

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
  private CommandFactory commandFactory = new CommandFactory(drive, elevator, coralWrist, algaeWrist);

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
    coralWrist.setDefaultCommand(coralWrist.setWristSpeed(0).withName("coral Wrist default"));
    algaeIntake.setDefaultCommand(algaeIntake.setRollerSpeed(RPM.of(0)));
    algaeWrist.setDefaultCommand(algaeWrist.setWristSpeed(0).withName("algae wrist default"));
    // elevator.setDefaultCommand(elevator.setSpeedManualControl(0));
  }

  private void configureButtonBindingsDriver() {
    driverController.rightTrigger().whileTrue(coralIntake.intake());
    driverController.leftTrigger().whileTrue(coralIntake.outtake());

    driverController.y().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());
    driverController.a().onTrue(commandFactory.turtleMode());
  }

  private void setUpAuton() {
    NamedCommands.registerCommand("intake coral", Commands.print("intake coral"));
    NamedCommands.registerCommand("Score coral on L1", Commands.print("coral scored on L1"));
    NamedCommands.registerCommand("Score coral on L2", Commands.print("coral scored on L2"));
    NamedCommands.registerCommand("Score coral on L4", Commands.print("coral scored on L4"));
    NamedCommands.registerCommand("Elevator to L1 setpoint", elevator.goToGoalSetpointCmd(SetpointConstants.L_ONE_HEIGHT).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L2 setpoint", elevator.goToGoalSetpointCmd(SetpointConstants.L_TWO_HEIGHT).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L3 setpoint", elevator.goToGoalSetpointCmd(SetpointConstants.L_THREE_HEIGHT).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Elevator to L4 setpoint", elevator.goToGoalSetpointCmd(SetpointConstants.L_FOUR_HEIGHT).andThen(elevator.atGoalCommand()));
    NamedCommands.registerCommand("Coral wrist to L1 setpoint", coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L1_ANGLE.in(Radians)).withTimeout(1));
    NamedCommands.registerCommand("Coral wrist to L2 setpoint", coralWrist.goToSetpointCommand(SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Radians)));
    NamedCommands.registerCommand("Outtake coral", coralIntake.outtake().andThen(Commands.waitSeconds(1)));
    NamedCommands.registerCommand("Outtake coral print", Commands.print("outtake coral"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos/Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureButtonBindingsOperator() {
<<<<<<< HEAD
<<<<<<< HEAD
    operatorController.rightTrigger()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_INTAKE_ANGLE.in(Radians))
            .withName("move coral wrist to intake setpoint"));
    operatorController.rightBumper()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Radians))
            .withName("move coral wrist to outtake setpoint"));
    operatorController.leftBumper().onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L1_ANGLE.in(Radians))
        .withName("move coral wrist to L1 outtake setpoint"));
    operatorController.y().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_TWO_HEIGHT));
    operatorController.x().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_ONE_HEIGHT));
    operatorController.b().whileTrue(elevator.incrementGoalPosition(Meters.of(0.001)));
    operatorController.a().whileTrue(elevator.incrementGoalPosition(Meters.of(-0.001)));
    operatorController.leftBumper().onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L1_ANGLE.in(Radians))
        .withName("move coral wrist to L1 outtake setpoint"));
=======
    // these buttons should not be changed for local testing and should function as
    // a replacement gamepad
    operatorController.a().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 1")));
    operatorController.b().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 2")));
    operatorController.x().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 3")));
    operatorController.y().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 4")));

    operatorController.rightBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("right")));
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("left")));

    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("coral")));
    operatorController.leftTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("algae")));

    // place local buttons below here, delete before PRing
=======
    operatorController.rightTrigger()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_INTAKE_ANGLE)
            .withName("move coral wrist to intake setpoint"));
    operatorController.rightBumper()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_OUTTAKE_ANGLE)
            .withName("move coral wrist to outtake setpoint"));
    operatorController.y().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_TWO_HEIGHT));
    operatorController.x().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_THREE_HEIGHT));
    // operatorController.b().whileTrue(elevator.incrementGoalPosition(Meters.of(0.005)));
    operatorController.b().whileTrue(elevator.setSpeedManualControl(0.1));
    operatorController.a().whileTrue(elevator.setSpeedManualControl(-0.1));
    operatorController.leftBumper().onTrue(elevator.goToGoalSetpointCmd(Meters.of(0.0)));
    // operatorController.a().whileTrue(elevator.incrementGoalPosition(Meters.of(-0.005)));
    // operatorController.leftBumper().onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L1_ANGLE.in(Radians))
    // .withName("move coral wrist to L1 outtake setpoint"));
>>>>>>> main

>>>>>>> main
  }
  }
<<<<<<< HEAD
=======

}
>>>>>>> main
