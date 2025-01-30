package frc.robot.subsystems.algaeWrist;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeWristHardware implements AlgaeWristIO {

    private final SparkMax algaeWristSparkMax;

    private final SparkClosedLoopController algaeWristClosedLoopController;

    private final AbsoluteEncoder algaeWristAbsoluteEncoder;

    private static final SparkMaxConfig wristSparkMaxConfig = new SparkMaxConfig();

    private static final boolean ENCODER_INVERTED = true;
    private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    private static final double P = 1.0;
    private static final double I = 0;
    private static final double D = 0.001;
    private static final double FF = 0;
    private static final double MIN_OUTPUT = -1;
    private static final double MAX_OUTPUT = 1;

    private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);

    public AlgaeWristHardware(int ALGAE_WRIST_CAN_ID) {

        wristSparkMaxConfig.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        wristSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
        wristSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(P, I, D, FF)
                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        algaeWristSparkMax = new SparkMax(ALGAE_WRIST_CAN_ID, MotorType.kBrushless);

        algaeWristAbsoluteEncoder = algaeWristSparkMax.getAbsoluteEncoder();

        algaeWristSparkMax.configure(wristSparkMaxConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        algaeWristClosedLoopController = algaeWristSparkMax.getClosedLoopController();

    }

    @Override
    public void setSpeed(double speed) {
        algaeWristClosedLoopController.setReference(
                speed * SpeedConstants.ALGAE_WRIST_MAX_SPEED_MPS.in(MetersPerSecond),
                ControlType.kVelocity);
    }

    @Override
    public double getVelocity() {
        return algaeWristAbsoluteEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return ((SparkBase) algaeWristAbsoluteEncoder).getOutputCurrent();
    }

    @Override
    public void updateStates(AlgaeWristIOStates states) {
        states.wristVelocity = getVelocity();
        states.wristAppliedVoltage = algaeWristSparkMax.getAppliedOutput() * algaeWristSparkMax.getBusVoltage();
        states.wristCurrent = algaeWristSparkMax.getOutputCurrent();
    }

}
