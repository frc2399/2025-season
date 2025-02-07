package frc.robot.subsystems.coralWrist;

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

    public Command goToSetpointCommand(double angle) {
        return this.run(() -> {
            io.setGoalAngle(angle);
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
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
        SmartDashboard.putNumber("coralWrist/wristAbsoluteEncoderAngleInDegrees",
                states.wristAbsoluteEncoderAngle * 180 / Math.PI);
    }
}