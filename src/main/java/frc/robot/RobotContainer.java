// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem drive;
  private Gyro gyro;
  private SubsystemFactory subsystemFactory;
  public VisionPoseEstimator visionPoseEstimator;

  private boolean useDriveHardware = true;
  private boolean useGyroHardware = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    subsystemFactory = new SubsystemFactory();
    gyro = subsystemFactory.buildGyro(useGyroHardware);
    drive = subsystemFactory.buildDriveSubsystem(useDriveHardware, gyro);
    visionPoseEstimator = new VisionPoseEstimator(drive);

    configureDefaultCommands();
    // Configure the trigger bindings
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
  }

 
  private void configureButtonBindingsDriver() {
    driverController.x().whileTrue((new RunCommand(
      () -> drive.setX(),
      drive).withName("setx")));
    
    driverController.b().onTrue(gyro.setYaw(0.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // }
}
