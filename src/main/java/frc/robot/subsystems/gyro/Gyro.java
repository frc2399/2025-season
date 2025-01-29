package frc.robot.subsystems.gyro;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gyro {
    private GyroIO io;

    public Gyro(GyroIO io) {
        this.io = io;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            io.setYaw(Math.toRadians(0));
        } else if (ally.get() == Alliance.Blue) {
            io.setYaw(180.0);
        } else {
            io.setYaw(0.0);
        }
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
