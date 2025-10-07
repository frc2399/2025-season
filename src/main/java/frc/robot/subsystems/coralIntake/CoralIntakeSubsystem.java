package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CommandFactory;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.subsystems.coralIntake.CoralIntakeIO.CoralIntakeIOStates;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final CoralIntakeIOStates states = new CoralIntakeIOStates();
    private CoralIntakeIO io;
    public boolean hasCoral = false;
    private final NetworkTableEntry coralEntry = CommandFactory.scoringStateTables.getEntry("hasCoral");

    public CoralIntakeSubsystem(CoralIntakeIO io) {
        this.io = io;
    }

    public Command intake() {
        return this.run(() -> io.intake()).withName("run coral intake");
    }

    public Command setOuttakeSpeed(Supplier<Setpoint> setpoint) {
        return this.run(() -> {
            io.setOuttakeSpeed(setpoint.get());
            setCoralEntry(false);
        });
    }

    public Command defaultBehavior() {
        return this.run(() -> {
            if (hasCoral) {
                io.passiveIntake();
            } else {
                io.setZero();
            }
        });
    }

    public Command passiveIntakeAuton() {
        return this.runOnce(() -> io.passiveIntakeIgnoringStall());
    }

    public Command intakeToStall() {
        return this.run(() -> {
            if (io.isStalling() || hasCoral) {
                io.passiveIntakeIgnoringStall();
                setCoralEntry(true);
            } else {
                io.intake();
            }
        });
    }

    public void setCoralEntry(Boolean coralState) {
        coralEntry.setBoolean(coralState);
        hasCoral = coralState;
    }

    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);
    private final MutAngle sysIdAngle = Radians.mutable(0);
    private final MutAngularVelocity sysIdAngularVelocity = RPM.mutable(0);

    public Command coralIntakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return coralIntakeTestRoutine.quasistatic(direction);
    }

    public Command coralIntakeSysIdDynamic(SysIdRoutine.Direction direction) {
        return coralIntakeTestRoutine.dynamic(direction);
    }

    private final Velocity<VoltageUnit> rampRate = Volts.of(0.25).per(Second);
    private final Voltage stepVoltage = Volts.of(3.5);
    private final Time timeout = Second.of(40);

    private SysIdRoutine coralIntakeTestRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(rampRate, stepVoltage, timeout),
            new SysIdRoutine.Mechanism(this::setVoltage, log -> {
                log.motor("coral-wrist-motor")
                        .voltage(sysIdAppliedVoltage.mut_replace(
                                // set speed is the value from -1.0 to 1.0; the phrase "set speed" comes from
                                // the description of the .get() method on the flex spark api
                                io.getAppliedVoltage(), Volts))
                        .angularPosition(sysIdAngle.mut_replace(io.getPosition(), Rotations))
                        .angularVelocity(
                                sysIdAngularVelocity.mut_replace(io.getAngularVelocity()));
                ;
            }, this, "coralWrist"));

    // only for sysid
    private void setVoltage(Voltage volts) {
        SmartDashboard.putNumber("coralIntake/SysIdRequestedVoltage", volts.in(Volts));
        io.setVoltage(volts);
    }

    @Override
    public void periodic() {
        hasCoral = coralEntry.getBoolean(hasCoral);
        io.updateStates(states);
        SmartDashboard.putNumber("coralIntake/velocity", states.velocity);
        SmartDashboard.putNumber("coralIntake/goalVelocity", states.goalVelocity);
        SmartDashboard.putNumber("coralIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("coralIntake/rightCurrent", states.rightCurrent);
        SmartDashboard.putNumber("coralIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/rightAppliedVoltage", states.rightAppliedVoltage);
        SmartDashboard.putBoolean("coralIntake/isStalling", io.isStalling());
        SmartDashboard.putBoolean("coralIntake/hasCoral", hasCoral);
    }
}
