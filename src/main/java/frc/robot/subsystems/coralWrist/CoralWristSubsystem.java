package frc.robot.subsystems.coralWrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralWrist.CoralWristIO.*;

public class CoralWristSubsystem extends SubsystemBase {
    public CoralWristIO io;
    public CoralWristIOStates states = new CoralWristIOStates();

    public CoralWristSubsystem(CoralWristIO io) {
        this.io = io;
    }

    public Command goToSetpoint(Angle angle) {
        return this.run(() -> {
            io.goToSetpoint(angle);
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
        });
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("coralWrist/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("coralWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("coralWrist/wristAbsoluteEncoderAngle", states.wristAbsoluteEncoderAngle);
    }
}