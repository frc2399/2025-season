package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gyro {

    private GyroIO io;

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public double getYaw() {
        return io.getYaw();
    }

    public Command setYaw(double yaw) {
        return Commands.runOnce(() -> io.setYaw(yaw));
    }

    public StatusSignal<edu.wpi.first.units.measure.AngularVelocity> getAngularVelocity() {
        return io.getAngularVelocity();
    }

}
