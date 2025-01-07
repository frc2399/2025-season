package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

public class Gyro {
    private GyroIO io;

    public static class GyroIOInputs {
        public double yawPositionRad = 0.0;
    }

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public double getYaw() {
        return io.getYaw();
    }

    public void setYaw(double yaw) {
        io.setYaw(yaw);
    }

    public void updateInputs(GyroIOInputs inputs) {
        io.updateInputs(inputs);
    }

    public StatusSignal<edu.wpi.first.units.measure.AngularVelocity> getAngularVelocity() {
        return io.getAngularVelocity();
    }

}
