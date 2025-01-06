package frc.robot.subsystems.gyro;

import frc.robot.subsystems.gyro.Gyro.GyroIOInputs;

public class GyroPlacebo implements GyroIO {
    public double getYaw() {
        return 0.0;
    }

    public void setYaw(double yaw) {
    }

    public void updateInputs(GyroIOInputs inputs) {
    }
}
