package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeAlphaHardware;
import frc.robot.subsystems.coralIntake.CoralIntakeBetaHardware;
import frc.robot.subsystems.coralIntake.CoralIntakePlacebo;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristHardware;
import frc.robot.subsystems.coralWrist.CoralWristPlacebo;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardware;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.elevator.AlphaElevator;
import frc.robot.subsystems.elevator.ElevatorPlacebo;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.KrakenElevator;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroHardware;
import frc.robot.subsystems.gyro.GyroPlacebo;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    private static final String MOZART_SERIAL_NUMBER = "030ee8c8";
    private static final String ALPHA_SERIAL_NUMBER = "03260A64";
    private static final String BETA_SERIAL_NUMBER = "30FC267";
    private static final String COMP_SERIAL_NUMBER = "";

    private static final Distance ELEVATOR_ALPHA_MAX_HEIGHT = Inches.of(34.25);
    private static final Distance ELEVATOR_BETA_MAX_HEIGHT = Inches.of(50); 

    private enum RobotType {
        MOZART,
        SIM,
        ALPHA,
        BETA,
        COMP
    }

    private RobotType robotType;

    private String serialNum = System.getenv("serialnum");

    public SubsystemFactory() {
        if (serialNum.equals(ALPHA_SERIAL_NUMBER)) {
            robotType = RobotType.ALPHA;
        } else if (serialNum.equals(BETA_SERIAL_NUMBER)) {
            robotType = RobotType.BETA;
        } else if (serialNum.equals(COMP_SERIAL_NUMBER)) {
            robotType = RobotType.COMP;
        } else if (serialNum.equals(MOZART_SERIAL_NUMBER)) {
            robotType = RobotType.MOZART;
        } else {
            robotType = RobotType.SIM;
        }
    }

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;
        Distance trackWidth;
        if (robotType == RobotType.ALPHA) {
            trackWidth = Constants.DriveControlConstants.ALPHA_TRACK_WIDTH;
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
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro, trackWidth);
        } else if (robotType == RobotType.MOZART) {
            trackWidth = Constants.DriveControlConstants.MOZART_TRACK_WIDTH;
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
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro, trackWidth);
        } else {
            frontLeft = new SwerveModule(new SwerveModulePlacebo());
            frontRight = new SwerveModule(new SwerveModulePlacebo());
            rearLeft = new SwerveModule(new SwerveModulePlacebo());
            rearRight = new SwerveModule(new SwerveModulePlacebo());
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro, Inches.of(10));
            // 10 is a default value for sim lol
        }
    }

    public Gyro buildGyro() {
        if (robotType == RobotType.ALPHA || robotType == RobotType.MOZART) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }

    public CoralIntakeSubsystem buildCoralIntake() {
        if (robotType == RobotType.ALPHA) {
            return new CoralIntakeSubsystem(new CoralIntakeAlphaHardware());
        } else if (robotType == RobotType.BETA) {
            return new CoralIntakeSubsystem(new CoralIntakeBetaHardware());
        } else {
            return new CoralIntakeSubsystem(new CoralIntakePlacebo());
        }

    }

    public CoralWristSubsystem buildCoralWrist() {
        if (robotType == RobotType.ALPHA) {
            return new CoralWristSubsystem(new CoralWristHardware());
        } else {
            return new CoralWristSubsystem(new CoralWristPlacebo());
        }
    }

    protected ElevatorSubsystem buildElevator() {
        if (robotType == RobotType.ALPHA) {
            return new ElevatorSubsystem(new AlphaElevator(ELEVATOR_ALPHA_MAX_HEIGHT));
        } 
        
        if (robotType == RobotType.BETA) {
            return new ElevatorSubsystem(new KrakenElevator(ELEVATOR_BETA_MAX_HEIGHT)); 
        }
        else {
            return new ElevatorSubsystem(new ElevatorPlacebo());
        }
    }
}
