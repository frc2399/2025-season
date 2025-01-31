// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.*;

public class RobotContainer {
        private CommandFactory commandFactory = new CommandFactory();
        private SubsystemFactory subsystemFactory = new SubsystemFactory();
        private Gyro gyro = subsystemFactory.buildGyro();
        private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
        private AlgaeIntakeSubsystem algaeIntake = subsystemFactory.buildAlgaeIntake();
        private AlgaeWristSubsystem algaeWrist = subsystemFactory.buildAlgaeWrist();
        // this is public because we need to run the visionPoseEstimator periodic from
        // Robot
        public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive);

        private static final CommandXboxController driverController = new CommandXboxController(
                        DriveControlConstants.DRIVER_CONTROLLER_PORT);
        private final CommandXboxController operatorController = new CommandXboxController(
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
                algaeIntake.setDefaultCommand(algaeIntake.setRollerSpeed(0));
                algaeWrist.setDefaultCommand(algaeWrist.setWristSpeed(0));
        }

        private void configureButtonBindingsDriver() {
                driverController.b().onTrue(gyro.setYaw(0.0));
                driverController.x().whileTrue(drive.setX());
        }

        private void configureButtonBindingsOperator() {
                operatorController.rightTrigger()
                                .whileTrue(algaeIntake.setRollerSpeed(
                                                SpeedConstants.ALGAE_INTAKE_MAX_SPEED_MPS.in(MetersPerSecond)));
                operatorController.leftTrigger()
                                .whileTrue((algaeIntake.setRollerSpeed(
                                                -SpeedConstants.ALGAE_INTAKE_MAX_SPEED_MPS.in(MetersPerSecond))));
                operatorController.leftBumper()
                                .whileTrue((algaeWrist.setWristSpeed(
                                                SpeedConstants.ALGAE_WRIST_MAX_SPEED_MPS.in(MetersPerSecond))));
                operatorController.rightBumper()
                                .whileTrue((algaeWrist.setWristSpeed(
                                                -SpeedConstants.ALGAE_WRIST_MAX_SPEED_MPS.in(MetersPerSecond))));
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
