package frc.robot.subsystems.algaeWrist;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeWristHardware implements AlgaeWristIO {

    private final double STATIC_FF_ALGAE = 0.0;
    private final double GRAVITY_FF_ALGAE = 0.0;
    private final double VELOCITY_FF_ALGAE = 0.0;

    private final ArmFeedforward algaeWristFeedFoward = new ArmFeedforward(STATIC_FF_ALGAE, GRAVITY_FF_ALGAE,
            VELOCITY_FF_ALGAE);

    private final SparkMax algaeWristSparkMax;

    private final SparkClosedLoopController algaeWristClosedLoopController;

    private final AbsoluteEncoder algaeWristAbsoluteEncoder;

    private static final SparkMaxConfig wristSparkMaxConfig = new SparkMaxConfig();

    private static final boolean MOTOR_INVERTED = false;
    private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;
    private static final double FF = 0;
    private static final double MIN_OUTPUT = -1;
    private static final double MAX_OUTPUT = 1;

    private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);

    public AlgaeWristHardware() {

        wristSparkMaxConfig.inverted(MOTOR_INVERTED).idleMode(IDLE_MODE)
                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        wristSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
        wristSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(P, I, D, FF)
                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        algaeWristSparkMax = new SparkMax(MotorIdConstants.ALGAE_WRIST_CAN_ID, MotorType.kBrushless);

        algaeWristAbsoluteEncoder = algaeWristSparkMax.getAbsoluteEncoder();

        algaeWristSparkMax.configure(wristSparkMaxConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        algaeWristClosedLoopController = algaeWristSparkMax.getClosedLoopController();

    }

    @Override
    public void goToSetpoint(Angle angle) {
        algaeWristClosedLoopController.setReference(angle.in(Radians), ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                algaeWristFeedFoward.calculate(angle.in(Radians), ENCODER_VELOCITY_FACTOR));
    }

    @Override
    public void setWristSpeed(double speed) {
        algaeWristSparkMax.set(speed
                + algaeWristFeedFoward.calculate(algaeWristAbsoluteEncoder.getPosition(), speed));
    }

    @Override
    public void updateStates(AlgaeWristIOStates states) {
        states.wristVelocity = algaeWristAbsoluteEncoder.getVelocity();
        states.wristAppliedVoltage = algaeWristSparkMax.getAppliedOutput() * algaeWristSparkMax.getBusVoltage();
        states.wristCurrent = algaeWristSparkMax.getOutputCurrent();
    }

}
