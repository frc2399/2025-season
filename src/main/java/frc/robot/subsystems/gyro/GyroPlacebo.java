package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroPlacebo implements GyroIO {
    public Angle getYaw() {
        return Degrees.of(0.0);
    }

    public void setYaw(Angle yaw) {
    }

    public StatusSignal<AngularVelocity> getAngularVelocity() {
        return new StatusSignal<AngularVelocity>(null, null, null);
    }
}
