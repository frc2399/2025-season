package frc.robot.subsystems.algaeWrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeWrist.AlgaeWristIO.AlgaeWristIOStates;

public class AlgaeWristSubsystem extends SubsystemBase {

    private final AlgaeWristIOStates states = new AlgaeWristIOStates();
    private final AlgaeWristIO io;

    public AlgaeWristSubsystem(AlgaeWristIO io) {
        this.io = io;
    }

    public Command goToSetpoint(Angle angle) {
        return this.run(() -> {
            io.goToSetpoint(angle);
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> Commands.run(() -> io.setWristSpeed(speed)));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("algaeWrist/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("algaeWrist/wristVelocity", states.wristVelocity);
    }

}
