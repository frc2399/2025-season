package frc.robot.subsystems.coralWrist;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralWrist.CoralWristIO.*;

public class CoralWristSubsystem extends SubsystemBase {
    private final CoralWristIO io;
    private CoralWristIOStates states = new CoralWristIOStates();
    private static final Angle WRIST_ALIGN_TOLERANCE = Radians.of(0.05);

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
        if (Math.abs(// states.trapezoidProfileGoalAngle -
                states.wristAbsoluteEncoderAngle) < WRIST_ALIGN_TOLERANCE.in(Radians))
            ;
        // {
        // io.periodic();
        // }
        SmartDashboard.putNumber("coralWrist/wristVelocity", states.wristVelocity);
        SmartDashboard.putNumber("coralWrist/wristAppliedVoltage",
                states.wristAppliedVoltage);
        SmartDashboard.putNumber("coralWrist/wristCurrent", states.wristCurrent);
        SmartDashboard.putNumber("coralWrist/wristAbsoluteEncoderAngleInDegrees",
                states.wristAbsoluteEncoderAngle * 180 / Math.PI);
    }
}