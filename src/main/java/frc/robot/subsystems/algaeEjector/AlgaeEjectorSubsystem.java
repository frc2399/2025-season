package frc.robot.subsystems.algaeEjector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeEjector.AlgaeEjectorIO.AlgaeEjectorIOStates;

public class AlgaeEjectorSubsystem extends SubsystemBase {

    private final AlgaeEjectorIOStates states = new AlgaeEjectorIOStates();
    private AlgaeEjectorIO io;

    public AlgaeEjectorSubsystem(AlgaeEjectorIO io) {
        this.io = io;
    }

    public Command setSpeed(double speed) {
        return this.run(() -> Commands.run(() -> io.setSpeed(speed)));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("AlgaeEjector/velocity", states.velocity);
        SmartDashboard.putNumber("AlgaeEjector/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("AlgaeEjector/rightCurrent", states.rightCurrent);
        SmartDashboard.putNumber("AlgaeEjector/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("AlgaeEjector/rightAppliedVoltage", states.rightAppliedVoltage);
    }
}