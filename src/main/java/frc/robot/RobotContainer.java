// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ReefscapeVisionUtil;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.VisionPoseEstimator;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

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
        () -> elevator.isElevatorHeightAboveSpeedLimitingThreshold()));
    coralIntake.setDefaultCommand(coralIntake.defaultBehavior());
    algaeIntake.setDefaultCommand(algaeIntake.defaultBehavior());
    climber.setDefaultCommand(climber.setSpeed(InchesPerSecond.of(0)));
  }

  private void configureButtonBindingsDriver() {
    driverController.rightTrigger().whileTrue(commandFactory.intakeOrClimbOutBasedOnMode());
    driverController.leftTrigger().whileTrue(commandFactory.outtakeOrClimbInBasedOnMode());

    driverController.rightBumper().onTrue(commandFactory.elevatorBasedOnMode());
    driverController.leftBumper().onTrue(drive.driveToPoseCommand(() -> commandFactory.getRobotPosition()))
        .onFalse(drive.disableDriveToPose());

    driverController.y().onTrue(commandFactory.resetHeading(Degrees.of(0)));
    driverController.x().whileTrue(drive.setX());
    driverController.b().onTrue(commandFactory.turtleBasedOnMode());

    // this yucky code bc we are out of buttons and have to use the POV pad (we want
    // to make sure that anything up does up and same for down)
    driverController.povUp().whileTrue(climber.setSpeed(InchesPerSecond.of(5)));
    driverController.povUpLeft().whileTrue(climber.setSpeed(InchesPerSecond.of(5)));
    driverController.povUpRight().whileTrue(climber.setSpeed(InchesPerSecond.of(5)));
    driverController.povDown().whileTrue(climber.setSpeed(InchesPerSecond.of(-3.5)));
    driverController.povDownLeft().whileTrue(climber.setSpeed(InchesPerSecond.of(-3.5)));
    driverController.povDownRight().whileTrue(climber.setSpeed(InchesPerSecond.of(-3.5)));
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
    // these buttons should not be changed for local testing and should function as
    // a replacement gamepad
    operatorController.a().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 1")));
    operatorController.b().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 2")));
    operatorController.x().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 3")));
    operatorController.y().onTrue(Commands.runOnce(() -> commandFactory.setScoringLevel("Level 4")));

    // operatorController.rightBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("right")));
    // operatorController.leftBumper().onTrue(Commands.runOnce(() -> commandFactory.setRobotAlignmentPosition("left")));

    // operatorController.leftTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("algae")));
    // operatorController.rightTrigger().onTrue(Commands.runOnce(() -> commandFactory.setGameMode("coral")));

    // place local buttons below here, delete before PRing
    // operatorController.rightBumper().onTrue(coralIntake.coralIntakeSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // operatorController.leftBumper().onTrue(coralIntake.coralIntakeSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // operatorController.rightTrigger().onTrue(coralIntake.coralIntakeSysIdDynamic(SysIdRoutine.Direction.kForward));
    // operatorController.leftTrigger().onTrue(coralIntake.coralIntakeSysIdDynamic(SysIdRoutine.Direction.kReverse));

    operatorController.rightBumper().onTrue(coralIntake.pos1K());
    operatorController.leftBumper().onTrue(coralIntake.neg1K());
  }
}
