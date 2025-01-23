// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.*;

public class RobotContainer {
  private CommandFactory commandFactory = new CommandFactory();
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  // this is public because we need to run the visionPoseEstimator periodic from
  // Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive);

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindingsDriver();
  }

  public enum AlignType {
    REEF_LEFT,
    REEF_RIGHT,
    CORAL_STATION;
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
  }

  private void configureButtonBindingsDriver() {
    driverController.b().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());

    driverController.rightTrigger()
        .onTrue(drive.driveToPoseCommand(AlignType.REEF_RIGHT, DriverStation.getAlliance())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    driverController.leftTrigger()
        .onTrue(drive.driveToPoseCommand(AlignType.REEF_LEFT, DriverStation.getAlliance())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }
}
