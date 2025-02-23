// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.*;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private final ElevatorSubsystem elevator = subsystemFactory.buildElevator();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
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
    configureDefaultCommands();
    configureButtonBindingsDriver();
    configureButtonBindingsOperator();
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