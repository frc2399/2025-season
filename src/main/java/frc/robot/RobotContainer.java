// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private final ElevatorSubsystem elevator = subsystemFactory.buildElevator();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  private static SendableChooser<Command> autoChooser;
  private ComplexWidget autonChooserWidget;
  private ClimberSubsystem climber = subsystemFactory.buildClimber();
  private final CoralIntakeSubsystem coralIntake = subsystemFactory.buildCoralIntake();
  public final CoralWristSubsystem coralWrist = subsystemFactory.buildCoralWrist();
  private final AlgaeIntakeSubsystem algaeIntake = subsystemFactory.buildAlgaeIntake();
  public final AlgaeWristSubsystem algaeWrist = subsystemFactory.buildAlgaeWrist();
  // this is public because we need to run the visionPoseEstimator periodic from
  // Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
  public CommandFactory commandFactory = new CommandFactory(drive, gyro, elevator, coralWrist, algaeWrist, algaeIntake,
      coralIntake, climber);

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(
      DriveControlConstants.OPERATOR_CONTROLLER_PORT);

  private boolean allowMovement = false;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDefaultCommands();
    configureButtonBindingsDriver();
    configureButtonBindingsOperator();
    setUpAuton();
  }

  public void disableSubsystems() {
    drive.disableDriveToPose();
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
        true,
        () -> allowMovement));
    coralIntake.setDefaultCommand(coralIntake.defaultBehavior());
    algaeIntake.setDefaultCommand(algaeIntake.defaultBehavior());
    climber.setDefaultCommand(climber.setSpeed(InchesPerSecond.of(0)));
  }

  private void configureButtonBindingsDriver() {
  }

  private void setUpAuton() {
    NamedCommands.registerCommand("call scoring level 1",
        Commands.runOnce(() -> commandFactory.setScoringLevel("Level 1")));
    NamedCommands.registerCommand("call scoring level 2",
        Commands.runOnce(() -> commandFactory.setScoringLevel("Level 2")));
    NamedCommands.registerCommand("call scoring level 3",
        Commands.runOnce(() -> commandFactory.setScoringLevel("Level 3")));
    NamedCommands.registerCommand("call scoring level 4",
        Commands.runOnce(() -> commandFactory.setScoringLevel("Level 4")));
    NamedCommands.registerCommand("call game mode coral", Commands.runOnce(() -> commandFactory.setGameMode("coral")));
    NamedCommands.registerCommand("Move elevator and coral wrist", commandFactory.moveElevatorAndCoralWrist());
    NamedCommands.registerCommand("Outtake coral",
        coralIntake.setOuttakeSpeed(() -> commandFactory.getSetpoint()).withDeadline(Commands.waitSeconds(0.25)));
    NamedCommands.registerCommand("turtle", commandFactory.turtleBasedOnMode());
    NamedCommands.registerCommand("coral intake default", coralIntake.defaultBehavior());
    // typically, we put this in a race group with our max intake time. however, the
    // until isStalling allows this command to finish first if we intake earlier,
    // thus ending the race group earlier (despite the name, this is only for coral)
    NamedCommands.registerCommand("intake", coralIntake.intakeToStall().withDeadline(Commands.waitSeconds(1)));
    NamedCommands.registerCommand("set intake speed to passive", coralIntake.passiveIntakeAuton());
    // explanation for this command in command factory
    NamedCommands.registerCommand("auton default subsystem position", commandFactory.autonDefaultPosition());
    NamedCommands.registerCommand("auton turtle", commandFactory.autonTurtleMode());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos/Selector", autoChooser);

    SmartDashboard.putData("reset odometry for facing red wall", resetOdometryRed());
    SmartDashboard.putData("reset odometry for facing blue wall", resetOdometryBlue());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command resetOdometryRed() {
    return (gyro.setYaw(Degrees.of(0))).ignoringDisable(true).andThen(

        Commands.runOnce(() ->

        {

          SmartDashboard.putBoolean("reseting odometry red", true);
          var poseEstimate = visionPoseEstimator.getPoseEstimate();
          poseEstimate.ifPresent((PoseEstimate pose) -> {
            var poseCopy = pose.pose;
            drive.resetOdometry(new Pose2d(poseCopy.getTranslation(), new Rotation2d(gyro.getYaw())));
          });

        }).ignoringDisable(true));
  }

  public Command resetOdometryBlue() {

    return (gyro.setYaw(Degrees.of(180)).ignoringDisable(true)).andThen(
        Commands.runOnce(() ->

        {

          SmartDashboard.putBoolean("reseting odometry blue", true);
          var poseEstimate = visionPoseEstimator.getPoseEstimate();
          poseEstimate.ifPresent((PoseEstimate pose) -> {
            var poseCopy = pose.pose;
            drive.resetOdometry(new Pose2d(poseCopy.getTranslation(), new Rotation2d(gyro.getYaw())));
          });

        }).ignoringDisable(true));
  }

  private void configureButtonBindingsOperator() {
    operatorController.a().onTrue(new InstantCommand(() -> allowMovement = true))
        .onFalse(new InstantCommand(() -> allowMovement = false));
    // place local buttons below here, delete before PRing

  }
}
