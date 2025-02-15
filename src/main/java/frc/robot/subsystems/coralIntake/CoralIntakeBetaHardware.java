package frc.robot.subsystems.coralIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class CoralIntakeBetaHardware implements CoralIntakeIO {

    private static final boolean BETA_CORAL_INTAKE_MOTOR_INVERTED = false;
    private static final IdleMode BETA_CORAL_INTAKE_IDLE_MODE = IdleMode.kBrake;

    private static final double BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR = 2 * Math.PI; // radians
    private static final double BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60; // radians per second

    private static final double BETA_CORAL_INTAKE_P = 1.0;
    private static final double BETA_CORAL_INTAKE_I = 0;
    private static final double BETA_CORAL_INTAKE_D = 0;
    private static final double BETA_CORAL_INTAKE_FF = 0;
    private static final double BETA_CORAL_INTAKE_PID_MIN_OUTPUT = -1.0;
    private static final double BETA_CORAL_INTAKE_PID_MAX_OUTPUT = 1.0;

    private static final boolean BETA_CORAL_INTAKE_POSITION_WRAPPING_ENABLED = true;

    private final RelativeEncoder betaCoralIntakeEncoder;
    private final SparkClosedLoopController betaCoralIntakeClosedLoop;
    private final SparkFlexConfig betaCoralIntakeConfig = new SparkFlexConfig();

    private final SparkFlex betaCoralIntakeSparkFlex;

    public CoralIntakeBetaHardware() {
        betaCoralIntakeConfig.inverted(BETA_CORAL_INTAKE_MOTOR_INVERTED).idleMode(BETA_CORAL_INTAKE_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        betaCoralIntakeConfig.encoder.positionConversionFactor(BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR);
        betaCoralIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(BETA_CORAL_INTAKE_P, BETA_CORAL_INTAKE_I, BETA_CORAL_INTAKE_D, BETA_CORAL_INTAKE_FF)
                .outputRange(BETA_CORAL_INTAKE_PID_MIN_OUTPUT, BETA_CORAL_INTAKE_PID_MAX_OUTPUT)
                .positionWrappingEnabled(BETA_CORAL_INTAKE_POSITION_WRAPPING_ENABLED);

        betaCoralIntakeSparkFlex = new SparkFlex(MotorIdConstants.CORAL_BETA_INTAKE_CAN_ID,
                MotorType.kBrushless);

        betaCoralIntakeSparkFlex.configure(betaCoralIntakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        betaCoralIntakeEncoder = betaCoralIntakeSparkFlex.getEncoder();
        betaCoralIntakeClosedLoop = betaCoralIntakeSparkFlex.getClosedLoopController();
    }

    @Override
    public void intake() {
        betaCoralIntakeClosedLoop.setReference(SpeedConstants.BETA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void outtake() {
        betaCoralIntakeClosedLoop.setReference(SpeedConstants.BETA_CORAL_OUTTAKE_SPEED.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void setZero() {
        betaCoralIntakeClosedLoop.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void updateStates(CoralIntakeIOStates states) {
        states.velocity = betaCoralIntakeEncoder.getVelocity();
        states.leftCurrent = betaCoralIntakeSparkFlex.getOutputCurrent();
        states.rightCurrent = betaCoralIntakeSparkFlex.getOutputCurrent();
        states.leftAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput()
                * betaCoralIntakeSparkFlex.getBusVoltage();
        states.rightAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput()
                * betaCoralIntakeSparkFlex.getBusVoltage();
    }
}
