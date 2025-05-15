package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.elevator.KrakenElevatorHardware.KrakenElevatorConstants;

public class SwerveModuleHardwareVortex implements SwerveModuleIO {

    private SparkFlex drivingSparkFlex;
    private SparkMax turningSparkMax;

    private final RelativeEncoder drivingRelativeEncoder;
    private final SparkAbsoluteEncoder turningAbsoluteEncoder;

    private final SparkClosedLoopController drivingPidController;
    private final SparkClosedLoopController turningPidController;

    private double chassisAngularOffset;
    private String name;
    private double desiredAngle;
    private double driveDesiredVelocity;
    private static final SparkFlexConfig sparkFlexConfigDriving = new SparkFlexConfig();
    private static final SparkMaxConfig sparkMaxConfigTurning = new SparkMaxConfig();

    // drivings are NEO Vortex, turnings are NEO 550s
    private static final int DRIVING_MOTOR_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    private static final boolean TURNING_ENCODER_INVERTED = true;
    private static final boolean DRIVING_MOTOR_INVERTED = false;
    private static final boolean TURNING_MOTOR_INVERTED = false;

    // Calculations required for driving motor conversion factors and feed forward
    private static final Distance WHEEL_DIAMETER = Inches.of(3);
    private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)

    private static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);

    private static final LinearVelocity DRIVE_WHEEL_FREE_SPEED = MetersPerSecond
            .of((MotorConstants.VORTEX_FREE_SPEED.in(RotationsPerSecond) *
                    WHEEL_CIRCUMFERENCE.in(Meters)) / (DRIVING_MOTOR_REDUCTION));

    private static final Distance DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER.times(Math.PI))
            .divide(DRIVING_MOTOR_REDUCTION); // meters
    private static final Distance DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR.divide(60); // meters
                                                                                                                // per
                                                                                                                // second

    private static final double TURNING_ENCODER_POSITION_FACTOR = Units.rotationsToRadians(1);
    private static final double TURNING_ENCODER_VELOCITY_FACTOR = Units.rotationsToRadians(1) / 60.0;

    private static final boolean TURNING_ENCODER_POSITION_WRAPPING = true;
    private static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    private static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    private static final double DRIVING_P = 0.4;
    private static final double DRIVING_I = 0;
    private static final double DRIVING_D = 0;
    private static final double DRIVING_FF = 1 / (DRIVE_WHEEL_FREE_SPEED.in(MetersPerSecond));
    private static final double DRIVING_MIN_OUTPUT = -1;
    private static final double DRIVING_MAX_OUTPUT = 1;

    private static final double TURNING_P = 1.0;
    private static final double TURNING_I = 0;
    private static final double TURNING_D = 0.001;
    private static final double TURNING_FF = 0;
    private static final double TURNING_MIN_OUTPUT = -1;
    private static final double TURNING_MAX_OUTPUT = 1;

    private static final double VOLTAGE_COMPENSATION = 12;

    private static final SparkBaseConfig.IdleMode DRIVING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

    public SwerveModuleHardwareVortex(int drivingCanId, int turningCanId, double chassisAngularOffset, String name) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.name = name;
        drivingSparkFlex = new SparkFlex(drivingCanId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);

        sparkFlexConfigDriving.inverted(DRIVING_MOTOR_INVERTED).idleMode(DRIVING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(
                        (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkFlexConfigDriving.encoder.positionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR.in(Meters))
                .velocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR.in(Meters));
        sparkFlexConfigDriving.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(DRIVING_P, DRIVING_I, DRIVING_D, DRIVING_FF)
                .outputRange(DRIVING_MIN_OUTPUT, DRIVING_MAX_OUTPUT);

        sparkMaxConfigTurning.inverted(TURNING_MOTOR_INVERTED).idleMode(TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(
                        (int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkMaxConfigTurning.absoluteEncoder.positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
        sparkMaxConfigTurning.absoluteEncoder.inverted(TURNING_ENCODER_INVERTED);
        sparkMaxConfigTurning.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(TURNING_P, TURNING_I, TURNING_D, TURNING_FF)
                .outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(TURNING_ENCODER_POSITION_WRAPPING)
                .positionWrappingInputRange(
                        TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        TURNING_ENCODER_POSITION_PID_MAX_INPUT);
        sparkMaxConfigTurning.signals.absoluteEncoderPositionPeriodMs(Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_MS);

        drivingSparkFlex.configure(sparkFlexConfigDriving, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turningSparkMax.configure(sparkMaxConfigTurning, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        drivingRelativeEncoder = drivingSparkFlex.getEncoder();
        turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();

        drivingPidController = drivingSparkFlex.getClosedLoopController();
        turningPidController = turningSparkMax.getClosedLoopController();
    }

    public void setDriveEncoderPosition(double position) {
        drivingRelativeEncoder.setPosition(position);
    };

    public double getDriveEncoderPosition() {
        double driveEncoderPosition = drivingRelativeEncoder.getPosition();
        if (Double.isNaN(driveEncoderPosition)) {
            return 0.0;
        } else {
            return driveEncoderPosition;
        }

    };

    public void setDesiredDriveSpeedMPS(double speed) {
        drivingPidController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        this.driveDesiredVelocity = speed;
    };

    public double getDriveEncoderSpeedMPS() {
        double driveVelocity = drivingRelativeEncoder.getVelocity();
        if (Double.isNaN(driveVelocity)) {
            return 0.0;
        } else {
            return driveVelocity;
        }
    };

    public double getTurnEncoderPosition() {
        double drivePosition = turningAbsoluteEncoder.getPosition();

        if (Double.isNaN(drivePosition)) {
            return 0.0;
        } else {
            return drivePosition;
        }
    };

    public void setDesiredTurnAngle(double angle) {
        turningPidController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        this.desiredAngle = angle;
    };

    public double getDriveBusVoltage() {
        return drivingSparkFlex.getBusVoltage();
    }

    public double getDriveOutput() {
        return drivingSparkFlex.getAppliedOutput();
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

    public void updateStates(SwerveModuleIOStates states) {
        states.desiredAngle = Units.radiansToDegrees(MathUtil.angleModulus(this.desiredAngle));
        states.turnAngle = Units.radiansToDegrees(MathUtil.angleModulus(getTurnEncoderPosition()));
        states.driveDesiredVelocity = this.driveDesiredVelocity;
        states.driveVelocity = getDriveEncoderSpeedMPS();
        states.driveEncoderPos = getDriveEncoderPosition();
        states.driveVoltage = drivingSparkFlex.getBusVoltage() * drivingSparkFlex.getAppliedOutput();
        states.turnVoltage = turningSparkMax.getBusVoltage() * turningSparkMax.getAppliedOutput();
        states.driveCurrent = drivingSparkFlex.getOutputCurrent();
        states.turnCurrent = turningSparkMax.getOutputCurrent();

        SmartDashboard.putNumber("Swerve/module " + name + "/turn desired angle(deg)", states.desiredAngle);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn angle(deg)",
                states.turnAngle);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive desired velocity(mps)",
                states.driveDesiredVelocity);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive velocity(mps)", states.driveVelocity);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive encoder position(m)",
                states.driveEncoderPos);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive voltage(volt)", states.driveVoltage);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn voltage(volt)", states.turnVoltage);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive current(amps)", states.driveCurrent);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn current(amps)", states.turnCurrent);
    }
}