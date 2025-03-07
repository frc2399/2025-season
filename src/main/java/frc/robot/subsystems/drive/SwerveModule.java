package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOStates;

public class SwerveModule {

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModuleIOStates states = new SwerveModuleIOStates();

    private SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
        io.setDriveEncoderPosition(0);
        desiredState.angle = new Rotation2d(getTurnEncoderPosition());
    }

    public double getDriveEncoderSpeedMPS() {
        return io.getDriveEncoderSpeedMPS();
    }

    public double getTurnEncoderPosition() {
        return io.getTurnEncoderPosition();
    }

    public double getDriveEncoderPosition() {
        return io.getDriveEncoderPosition();
    }

    public void resetEncoders() {
        io.setDriveEncoderPosition(0);
    }

    public void updateStates() {
        io.updateStates(states);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(getDriveEncoderSpeedMPS(),
                new Rotation2d((getTurnEncoderPosition()) - io.getChassisAngularOffset()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                getDriveEncoderPosition(),
                new Rotation2d(getTurnEncoderPosition() - io.getChassisAngularOffset()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param newDesiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState newDesiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = newDesiredState.speedMetersPerSecond;
        correctedDesiredState.angle = newDesiredState.angle.plus(Rotation2d.fromRadians(io.getChassisAngularOffset()));

        // Optimize the reference state to avoid spinning further than 90 degrees.

        correctedDesiredState.optimize(new Rotation2d(getTurnEncoderPosition()));
        io.setDesiredDriveSpeedMPS(correctedDesiredState.speedMetersPerSecond);
        io.setDesiredTurnAngle(correctedDesiredState.angle.getRadians());
        desiredState = newDesiredState; // TODO: this seems weird
    }

}