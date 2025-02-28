// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.Constants.SetpointConstants;
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
  private ClimberSubsystem climber = subsystemFactory.buildClimber();
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
        climber.setDefaultCommand(climber.setSpeed(0));
      }

  private void configureButtonBindingsDriver() {
    // driverController.rightBumper()
    //     .whileTrue(coralIntake.setRollerSpeed(SpeedConstants.CORAL_INTAKE_SPEED).withName("run coral intake"));
    // driverController.leftBumper()
    //     .whileTrue(coralIntake.setRollerSpeed(SpeedConstants.CORAL_OUTTAKE_SPEED).withName("run coral outtake"));
    // driverController.b().onTrue(gyro.setYaw(0.0));
    // driverController.x().whileTrue(drive.setX());
    // driverController.a().onTrue(commandFactory.turtleMode());

    driverController.rightBumper().whileTrue(climber.setSpeed(0.1)); 
    driverController.leftBumper().whileTrue(climber.setSpeed(-0.5)); 
  }

  private void configureButtonBindingsOperator() {
    // these buttons should not be changed for local testing and should function as
    // a replacement gamepad
    operatorController.a().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 1")));
    operatorController.b().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 2")));
    operatorController.x().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 3")));
    operatorController.y().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 4")));

    operatorController.rightBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("right")));
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("left")));

    operatorController.leftTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("algae")));
    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("coral")));

    // place local buttons below here, delete before PRing

  }
}