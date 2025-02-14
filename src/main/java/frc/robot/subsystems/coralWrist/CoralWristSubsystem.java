package frc.robot.subsystems.coralWrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralWrist.CoralWristIO.CoralWristIOStates;

public class CoralWristSubsystem extends SubsystemBase {
    private final CoralWristIO io;
    private CoralWristIOStates states = new CoralWristIOStates();

    public CoralWristSubsystem(CoralWristIO io) {
        this.io = io;
    }

    public Command goToSetpointCommand(Angle angle) {
        return this.run(() -> {
            io.setGoalAngle(angle);
        });
    }

    public Command setWristSpeed(AngularVelocity speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
        });
    }

    public Command setWristSpeedType(AngularVelocity speed) {
        return this.run(() -> {
            io.setWristSpeedType(speed);
        });
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