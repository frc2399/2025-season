// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
  private CommandFactory commandFactory = new CommandFactory(drive, elevator, coralWrist, algaeWrist);

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
    coralWrist.setDefaultCommand(coralWrist.setWristSpeed(0).withName("coral Wrist default"));
    algaeIntake.setDefaultCommand(algaeIntake.setRollerSpeed(RPM.of(0)));
    algaeWrist.setDefaultCommand(algaeWrist.setWristSpeed(0).withName("algae wrist default"));
    // elevator.setDefaultCommand(elevator.setSpeedManualControl(0));

  }

  private void configureButtonBindingsDriver() {
    driverController.rightBumper().whileTrue(coralIntake.intake());
    driverController.leftBumper().whileTrue(coralIntake.outtake());
    driverController.rightTrigger().whileTrue(algaeIntake.setRollerSpeed(SpeedConstants.ALGAE_INTAKE_SPEED));
    driverController.leftTrigger().whileTrue(algaeIntake.setRollerSpeed(SpeedConstants.ALGAE_OUTAKE_SPEED));
    driverController.b().onTrue(gyro.setYaw(0.0));
    driverController.x().whileTrue(drive.setX());
    driverController.a().onTrue(commandFactory.turtleMode());
  }

  private void configureButtonBindingsOperator() {
    operatorController.rightTrigger()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_INTAKE_ANGLE)
            .withName("move coral wrist to intake setpoint"));
    operatorController.leftTrigger()
        .onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L4_OUTTAKE_ANGLE)
            .withName("move coral wrist to outtake setpoint"));
    operatorController.y().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_TWO_HEIGHT));
    operatorController.x().onTrue(elevator.goToGoalSetpointCmd(SetpointConstants.L_THREE_HEIGHT));
    operatorController.rightBumper()
        .onTrue(algaeWrist.goToSetpointCommand(SetpointConstants.ALGAE_WRIST_INTAKE_ANGLE.in(Radians))
            .withName("move algae wrist to inttake setpoint"));
    operatorController.a()
        .onTrue(algaeWrist.goToSetpointCommand(-SetpointConstants.ALGAE_WRIST_INTAKE_ANGLE.in(Radians))
            .withName("move algae wrist to outtake setpoint"));
    // operatorController.b().whileTrue(elevator.incrementGoalPosition(Meters.of(0.005)));
    operatorController.b().whileTrue(elevator.setSpeedManualControl(0.1));
    operatorController.a().whileTrue(elevator.setSpeedManualControl(-0.1));
    operatorController.leftBumper().onTrue(elevator.goToGoalSetpointCmd(Meters.of(0.0)));
    // operatorController.a().whileTrue(elevator.incrementGoalPosition(Meters.of(-0.005)));
    // operatorController.leftBumper().onTrue(coralWrist.goToSetpointCommand(SetpointConstants.CORAL_L1_ANGLE.in(Radians))
    // .withName("move coral wrist to L1 outtake setpoint"));
  }
}