package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gyro {
    private GyroIO io;

    public Gyro(GyroIO io) {
        this.io = io;
        // At boot, assume we are facing the red alliance wall. Unfortunately, we
        // usually don't have comms at boot, so we can't trust a
        // DriverStation.getAlliance() to not be empty.
        io.setYaw(Degrees.of(0));
    }

    public Angle getYaw() {
        return io.getYaw();
    }

    public Angle getYaw(boolean refresh) {
        return io.getYaw(refresh);
    }

    public Command setYaw(Angle yaw) {
        return Commands.runOnce(() -> io.setYaw(yaw));
    }

    public StatusSignal<edu.wpi.first.units.measure.AngularVelocity> getAngularVelocity() {
        return io.getAngularVelocity();
    }

    public boolean hasFault() {
        return io.hasFault();
    }
}
