// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.*;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  public ElevatorSubsystem elevator = subsystemFactory.buildElevator();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  //this is public because we need to run the visionPoseEstimator periodic from Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive);
  private CommandFactory commandFactory = new CommandFactory(elevator);

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(
      DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindingsDriver();
    configureButtonBindingsOperator();
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

      elevator.setDefaultCommand(elevator.setSpeedCommand(0));
  }
 
  private void configureButtonBindingsDriver() {
    driverController.b().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());
  }

  private void configureButtonBindingsOperator() {
    //a lot of these have the same button binding so be careful which one you uncomment
    //operatorController.y().whileTrue(elevator.goToSetPointCommandPID(SetpointConstants.MIDDLE.in(Meters)));
    operatorController.y().onTrue(elevator.goToSetpointCmdMotionProfling(SetpointConstants.MIDDLE.in(Meters)));
    //operatorController.b().onTrue(elevator.goToSetpointCmdMotionProfling(0));
    //operatorController.a().onTrue(elevator.goToSetpointCmdMotionProfling(SetpointConstants.L_ONE_HEIGHT.in(Meters)));

    operatorController.b().whileTrue(elevator.setPercentOutputCommand(.1));
    operatorController.x().whileTrue(elevator.setPercentOutputCommand(-0.1));
    //operatorController.x().onTrue(elevator.setEncoderPositionCommand(0.01));
  }
}
