package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorConstants;

public class SwerveModuleHardware implements SwerveModuleIO {

    private final SparkMax drivingSparkMax;
    private final SparkMax turningSparkMax;

    private final RelativeEncoder drivingRelativeEncoder;
    private final SparkAbsoluteEncoder turningAbsoluteEncoder;

    private final SparkClosedLoopController drivingPidController;
    private final SparkClosedLoopController turningPidController;

    private double chassisAngularOffset;
    private String name;

    private static final SparkMaxConfig SPARK_MAX_CONFIG_DRIVING = new SparkMaxConfig();
    private static final SparkMaxConfig SPARK_MAX_CONFIG_TURNING = new SparkMaxConfig();

    // drivings are NEOs, turnings are NEO 550s
    // THIS IS 13 ON COMP BOT
    private static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    private static final boolean TURNING_ENCODER_INVERTED = true;
    private static final boolean DRIVING_ENCODER_INVERTED = false;

    // Calculations required for driving motor conversion factors and feed forward
    private static final Distance WHEEL_DIAMETER = Inches.of(3);
    private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);
   
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)




    
    private static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);


    private static final double DRIVE_WHEEL_FREE_SPEED_RPS = ((MotorConstants.NEO_FREE_SPEED_RPS
            * WHEEL_CIRCUMFERENCE.in(Meters))
            / DRIVING_MOTOR_REDUCTION);

    private static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER.in(Meters) * Math.PI)
            / DRIVING_MOTOR_REDUCTION / (260.0 / 254); // meters

    private static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60; // meters per
                                                                                                        // second


    private static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    private static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    private static final boolean TURNING_ENCODER_POSITION_WRAPPING = true;
    private static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    private static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    private static final double DRIVING_P = 0.2;
    private static final double DRIVING_I = 0;
    private static final double DRIVING_D = 0;
    private static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    private static final double DRIVING_MIN_OUTPUT = -1;
    private static final double DRIVING_MAX_OUTPUT = 1;

    private static final double TURNING_P = 1.0;
    private static final double TURNING_I = 0;
    private static final double TURNING_D = 0.001;
    private static final double TURNING_FF = 0;
    private static final double TURNING_MIN_OUTPUT = -1;
    private static final double TURNING_MAX_OUTPUT = 1;

    private static final SparkBaseConfig.IdleMode DRIVING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

    public SwerveModuleHardware(int drivingCanId, int turningCanId, double chassisAngularOffset, String name) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.name = name;

        SPARK_MAX_CONFIG_DRIVING.inverted(DRIVING_ENCODER_INVERTED).idleMode(DRIVING_MOTOR_IDLE_MODE);
        SPARK_MAX_CONFIG_DRIVING.encoder.positionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR);
        SPARK_MAX_CONFIG_DRIVING.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(DRIVING_P, DRIVING_I, DRIVING_D, DRIVING_FF)
                .outputRange(DRIVING_MIN_OUTPUT, DRIVING_MAX_OUTPUT);

        SPARK_MAX_CONFIG_TURNING.inverted(TURNING_ENCODER_INVERTED).idleMode(TURNING_MOTOR_IDLE_MODE);
        SPARK_MAX_CONFIG_TURNING.encoder.positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
        SPARK_MAX_CONFIG_TURNING.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(TURNING_P, TURNING_I, TURNING_D, TURNING_FF)
                .outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(TURNING_ENCODER_POSITION_WRAPPING)
                .positionWrappingInputRange(
                        TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        drivingSparkMax = new SparkMax(drivingCanId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);

        drivingSparkMax.configure(SPARK_MAX_CONFIG_DRIVING, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turningSparkMax.configure(SPARK_MAX_CONFIG_TURNING, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        drivingRelativeEncoder = drivingSparkMax.getEncoder();
        turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();

        drivingPidController = drivingSparkMax.getClosedLoopController();
        turningPidController = turningSparkMax.getClosedLoopController();

    }

    public void setDriveEncoderPosition(double position) {
        drivingRelativeEncoder.setPosition(position);
    };

    public double getDriveEncoderPosition() {
        return drivingRelativeEncoder.getPosition();
    };

    public void setDesiredDriveSpeedMPS(double speed) {
        drivingPidController.setReference(speed, ControlType.kVelocity);
    };

    public double getDriveEncoderSpeedMPS() {
        return drivingRelativeEncoder.getVelocity();
    };

    public double getTurnEncoderPosition() {
        return turningAbsoluteEncoder.getPosition();
    };

    public void setDesiredTurnAngle(double angle) {
        turningPidController.setReference(angle, ControlType.kPosition);
    };

    public double getDriveBusVoltage() {
        return drivingSparkMax.getBusVoltage();
    }

    public double getDriveOutput() {
        return drivingSparkMax.getAppliedOutput();
    }

    public double getTurnBusVoltage() {
        return turningSparkMax.getBusVoltage();
    }

    public double getTurnOutput() {
        return turningSparkMax.getAppliedOutput();
    }

    public String getName() {
        return name;
    }

    public double getChassisAngularOffset() {
        return chassisAngularOffset;
    }
}