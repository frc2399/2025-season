package frc.robot.subsystems.algaeEjector;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;

public class AlgaeEjectorHardware implements AlgaeEjectorIO {

    @Override
    public void setSpeed(double speed) {
    }

    private static final SparkMaxConfig SPARK_MAX_CONFIG_TURNING = new SparkMaxConfig();
    private static final boolean TURNING_ENCODER_INVERTED = true;
    private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    private static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    private static final double TURNING_P = 1.0;
    private static final double TURNING_I = 0;
    private static final double TURNING_D = 0.001;
    private static final double TURNING_FF = 0;
    private static final double TURNING_MIN_OUTPUT = -1;
    private static final double TURNING_MAX_OUTPUT = 1;

    public AlgaeEjectorHardware() {

        SPARK_MAX_CONFIG_TURNING.inverted(TURNING_ENCODER_INVERTED).idleMode(TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
        SPARK_MAX_CONFIG_TURNING.encoder.positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
        SPARK_MAX_CONFIG_TURNING.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(TURNING_P, TURNING_I, TURNING_D, TURNING_FF).outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT);
    }

}