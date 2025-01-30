package frc.robot.subsystems.algaeWrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeWrist.AlgaeWristIO.AlgaeWristIOStates;

public class AlgaeWristSubsystem extends SubsystemBase {

    private final AlgaeWristIOStates states = new AlgaeWristIOStates();
    private AlgaeWristIO io;

    public AlgaeWristSubsystem(AlgaeWristIO io) {
        this.io = io;
    }

    public Command setSpeed(double speed) {
        return this.run(() -> Commands.run(() -> io.setSpeed(speed)));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("algaeWrist/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("algaeWrist/wristVelocity", states.wristVelocity);
    }

}
