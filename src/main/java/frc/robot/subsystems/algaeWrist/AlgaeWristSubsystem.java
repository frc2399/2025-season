package frc.robot.subsystems.algaeWrist;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.ScoringLevel;
import frc.robot.subsystems.algaeWrist.AlgaeWristIO.AlgaeWristIOStates;

public class AlgaeWristSubsystem extends SubsystemBase {

    private final AlgaeWristIOStates states = new AlgaeWristIOStates();
    private final AlgaeWristIO io;

    public AlgaeWristSubsystem(AlgaeWristIO io) {
        this.io = io;
    }

    public Command goToSetpointCommand(Supplier<ScoringLevel> scoringLevel) {
        return this.runOnce(() -> {
            io.setGoalAngle(scoringLevel);
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> io.setWristSpeed(speed));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("algaeWrist/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("algaeWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("algaeWrist/wristEncoderAngleInDegrees",
                states.wristRelativeEncoderAngle * 180 / Math.PI);
        SmartDashboard.putNumber("algaeWrist/goalAngle", states.goalAngle);
    }

}