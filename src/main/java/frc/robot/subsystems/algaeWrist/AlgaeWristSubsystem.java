package frc.robot.subsystems.algaeWrist;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.subsystems.algaeWrist.AlgaeWristIO.AlgaeWristIOStates;

public class AlgaeWristSubsystem extends SubsystemBase {

    private final AlgaeWristIOStates states = new AlgaeWristIOStates();
    private final AlgaeWristIO io;

    public AlgaeWristSubsystem(AlgaeWristIO io) {
        this.io = io;
    }

    public Command goToSetpointCommand(Supplier<Setpoint> setpoint) {
        return this.runOnce(() -> {
            io.setGoalAngle(setpoint.get());
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> io.setWristSpeed(speed));
    }

    public void resetWrist() {
        io.resetRelativeToAbsolute();
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("algaeWrist/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("algaeWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("algaeWrist/wristRelativeEncoderAngleInDegrees",
                states.wristRelativeEncoderAngle * 180 / Math.PI);
        SmartDashboard.putNumber("algaeWrist/wristAbsoluteEncoderAngleToDegrees", 
                states.wristAbsoluteEncoderAngle * 180 / Math.PI);
        SmartDashboard.putNumber("algaeWrist/goalAngle", states.goalAngle * 180 / Math.PI);
    }
}