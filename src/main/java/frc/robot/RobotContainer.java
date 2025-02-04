// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;

public class RobotContainer {
  private CommandFactory commandFactory = new CommandFactory();
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  private Climber climber = subsystemFactory.buildClimber();

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindingsDriver();
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

      climber.setDefaultCommand(climber.set(0));
  }
 
  private void configureButtonBindingsDriver() {
    driverController.x().whileTrue(drive.setX());

  }
}
