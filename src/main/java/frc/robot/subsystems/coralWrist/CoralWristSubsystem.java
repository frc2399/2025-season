package frc.robot.subsystems.coralWrist;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.subsystems.coralWrist.CoralWristIO.CoralWristIOStates;

public class CoralWristSubsystem extends SubsystemBase {
    private final CoralWristIO io;
    private CoralWristIOStates states = new CoralWristIOStates();

    public CoralWristSubsystem(CoralWristIO io) {
        this.io = io;
    }

    public Command goToSetpointCommand(Supplier<ScoringLevel> scoringLevel) {
        return this.runOnce(() -> {
            io.setGoalAngle(scoringLevel.get());
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
        });
    }

    public boolean atGoal() {
        return io.atGoal();
    }

    // taking out motion profiling to see if code works
    // public Command setGoalStateTrapezoidCommand(Angle angle) {
    // return this.run(() -> {
    // io.setGoalStateTrapezoid(angle);
    // });
    // }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("coralWrist/wristAppliedVoltage",
                states.wristAppliedVoltage);
        SmartDashboard.putNumber("coralWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("coralWrist/wristEncoderAngleInDegrees",
                states.wristRelativeEncoderAngle * 180 / Math.PI);
        SmartDashboard.putNumber("coralWrist/goalAngle", states.goalAngle);
    }
}