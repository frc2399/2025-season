// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SpeedConstants;
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
  // this is public because we need to run the visionPoseEstimator periodic from
  // Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive);
  private CommandFactory commandFactory = new CommandFactory(drive, elevator);

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
    elevator.disableElevator();
  }

  public void configureDefaultCommands() {
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                -(MathUtil.applyDeadband(
                    driverController.getLeftY(),
                    DriveControlConstants.DRIVE_DEADBAND)),
                -(MathUtil.applyDeadband(
                    driverController.getLeftX(),
                    DriveControlConstants.DRIVE_DEADBAND)),
                -(MathUtil.applyDeadband(
                    driverController.getRightX(),
                    DriveControlConstants.DRIVE_DEADBAND)),
                DriveControlConstants.FIELD_ORIENTED_DRIVE),
            drive).withName("drive default"));
    coralIntake.setDefaultCommand(coralIntake.setRollerSpeed(0).withName("coral Intake default"));
    coralWrist.setDefaultCommand(coralWrist.setWristSpeed(0).withName("coral Wrist default"));
    elevator.setDefaultCommand(elevator.setPercentOutputCommand(0));
  }

  private void configureButtonBindingsDriver() {
    driverController.rightBumper()
        .whileTrue(coralIntake.setRollerSpeed(SpeedConstants.CORAL_INTAKE_SPEED).withName("run coral intake"));
    driverController.leftBumper()
        .whileTrue(coralIntake.setRollerSpeed(SpeedConstants.CORAL_OUTTAKE_SPEED).withName("run coral outtake"));
    driverController.b().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());
  }

  private void configureButtonBindingsOperator() {
    operatorController.rightTrigger()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_INTAKE_ANGLE.in(Radians))
            .withName("move coral wrist to intake setpoint"));
    operatorController.rightBumper()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_OUTTAKE_ANGLE.in(Radians))
            .withName("move coral wrist to outtake setpoint"));
    operatorController.y().onTrue(elevator.goToSetPointCommand(SetpointConstants.L_TWO_HEIGHT.in(Meters)));
    operatorController.x().onTrue(elevator.goToSetPointCommand(SetpointConstants.L_ONE_HEIGHT.in(Meters)));
    operatorController.b().whileTrue(elevator.setPercentOutputCommand(.1));
    operatorController.a().whileTrue(elevator.setPercentOutputCommand(-0.1));
  }
}
