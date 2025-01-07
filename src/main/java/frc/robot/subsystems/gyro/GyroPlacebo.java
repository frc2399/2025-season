package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.gyro.Gyro.GyroIOInputs;

public class GyroPlacebo implements GyroIO {
    public double getYaw() {
        return 0.0;
    }

    public void setYaw(double yaw) {
    }

    public void updateInputs(GyroIOInputs inputs) {
    }

    public StatusSignal<AngularVelocity> getAngularVelocity() {
        return new StatusSignal<AngularVelocity>(null, 0, null, null, null, null, null);
    }
}
