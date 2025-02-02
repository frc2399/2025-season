package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {
    private boolean atGoal;
    private DriveSubsystem drive;
    private Supplier<Pose2d> goalPose;
    private Pose2d robotPose;

    public DriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> goalPose) {

    }

    @Override
    public void initialize() {
        robotPose = drive.getPose();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
