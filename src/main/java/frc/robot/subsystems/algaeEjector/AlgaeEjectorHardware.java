package frc.robot.subsystems.algaeEjector;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;

public class AlgaeEjectorHardware implements AlgaeEjectorIO {

        private static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
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

        public AlgaeEjectorHardware() {

                SPARK_MAX_CONFIG.inverted(ENCODER_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit(MotorConstants.NEO550_CURRENT_LIMIT);
                SPARK_MAX_CONFIG.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                SPARK_MAX_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(P, I, D, FF)
                                .outputRange(MIN_OUTPUT, MAX_OUTPUT);
        }

        @Override
        public void setSpeed(double speed) {
        }

}