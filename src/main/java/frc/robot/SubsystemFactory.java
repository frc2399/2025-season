package frc.robot;

import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardware;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroHardware;
import frc.robot.subsystems.gyro.GyroPlacebo;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
    
    private static final boolean isSim = Robot.isSimulation();

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;
        if (!isSim) {
            frontLeft = new SwerveModule(new SwerveModuleHardware(
                MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID, 
                FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardware(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardware(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardware(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        } else {
            frontLeft = new SwerveModule(new SwerveModulePlacebo());
            frontRight = new SwerveModule(new SwerveModulePlacebo());
            rearLeft = new SwerveModule(new SwerveModulePlacebo());
            rearRight = new SwerveModule(new SwerveModulePlacebo());
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        }
    }

    public Gyro buildGyro() {
        if (!isSim) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }
}
