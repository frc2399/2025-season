package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;

public class GyroPlacebo implements GyroIO {
    public double getYaw() {
        return 0.0;
    }

    public void setYaw(double yaw) {
    }

    public StatusSignal<AngularVelocity> getAngularVelocity() {
        return new StatusSignal<AngularVelocity>(null, null, null);
    }
}
