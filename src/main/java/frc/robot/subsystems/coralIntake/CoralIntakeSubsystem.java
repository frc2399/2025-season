package frc.robot.subsystems.coralIntake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralIntake.CoralIntakeIO.CoralIntakeIOStates;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final CoralIntakeIOStates states = new CoralIntakeIOStates();
    private CoralIntakeIO io;

    public CoralIntakeSubsystem(CoralIntakeIO io) {
        this.io = io;
    }

    public Command setRollerSpeed(double speed) {
        return this.run(() -> io.setRollerSpeed(speed));
    }

    public Command goToSetpoint(Angle angle) {
        return this.run(() -> {
            io.goToSetpoint(angle);
            io.setRollerSpeed(0);
        });
    }

    public Command setWristSpeed(double speed) {
        return this.run(() -> {
            io.setWristSpeed(speed);
            io.setRollerSpeed(0);
        });
    }

    public Command setSpeeds() {
        return this.run(() -> {
            io.setWristSpeed(0);
            io.setRollerSpeed(0);
        });
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralIntake/velocity", states.velocity);
        SmartDashboard.putNumber("coralIntake/leftCurrent", states.topCurrent);
        SmartDashboard.putNumber("coralIntake/rightCurrent", states.bottomCurrent);
        SmartDashboard.putNumber("coralIntake/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("coralIntake/leftAppliedVoltage", states.topAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/rightAppliedVoltage", states.bottomAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/wristAppliedVoltage", states.wristAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/wristEncoderAngle", states.wristEncoderAngle);
    }
}
