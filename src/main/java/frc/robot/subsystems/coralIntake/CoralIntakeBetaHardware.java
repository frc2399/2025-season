package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

//intake to stall
public class CoralIntakeBetaHardware implements CoralIntakeIO {

    private static final boolean BETA_CORAL_INTAKE_MOTOR_INVERTED = true;
    private static final IdleMode BETA_CORAL_INTAKE_IDLE_MODE = IdleMode.kBrake;

    // 5:1 gearbox ratio
    private static final double BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR = 1.0 / 5.0; // Rotations of output wheels
    private static final double BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR = BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR; // RPM

    private static final double BETA_CORAL_INTAKE_P = 0.001;
    private static final double BETA_CORAL_INTAKE_I = 0;
    private static final double BETA_CORAL_INTAKE_D = 0;
    private static final double BETA_CORAL_INTAKE_FF = 5.0 / MotorConstants.VORTEX_FREE_SPEED.in(RPM);

    private final RelativeEncoder betaCoralIntakeEncoder;
    private final SparkClosedLoopController betaCoralIntakeClosedLoop;
    private final SparkFlexConfig betaCoralIntakeConfig = new SparkFlexConfig();

    private final SparkFlex betaCoralIntakeSparkFlex;

    private double velocityGoal = 0;

    private static Debouncer CORAL_BETA_DEBOUNCER;
    private static Current coralIntakeStallThreshold;

    public CoralIntakeBetaHardware(Time debouncerTime, Current stallThreshold) {
        CORAL_BETA_DEBOUNCER = new Debouncer(debouncerTime.in(Seconds));
        coralIntakeStallThreshold = stallThreshold;
        betaCoralIntakeConfig.inverted(BETA_CORAL_INTAKE_MOTOR_INVERTED).idleMode(BETA_CORAL_INTAKE_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        betaCoralIntakeConfig.encoder.positionConversionFactor(BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR);
        betaCoralIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(BETA_CORAL_INTAKE_P, BETA_CORAL_INTAKE_I, BETA_CORAL_INTAKE_D, BETA_CORAL_INTAKE_FF);

        betaCoralIntakeConfig.signals
                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

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
        velocityGoal = SpeedConstants.BETA_CORAL_INTAKE_SPEED.in(RPM);
    }

    @Override
    public void setOuttakeSpeed(Setpoint setpoint) {
        double desiredVelocity = 0;
        if (setpoint == Setpoint.L_ONE) {
            desiredVelocity = SpeedConstants.BETA_CORAL_L1_OUTTAKE_SPEED.in(RPM);
        } else {
            desiredVelocity = SpeedConstants.BETA_CORAL_OUTTAKE_SPEED.in(RPM);
        }

        betaCoralIntakeClosedLoop.setReference(desiredVelocity, ControlType.kVelocity);
        velocityGoal = desiredVelocity;
    }

    @Override
    public void setZero() {
        betaCoralIntakeClosedLoop.setReference(0, ControlType.kVelocity);
        velocityGoal = 0;
    }

    @Override
    public boolean isStalling() {
        boolean isStalling = CORAL_BETA_DEBOUNCER
                .calculate(betaCoralIntakeSparkFlex.getOutputCurrent() > coralIntakeStallThreshold.in(Amps));
        return isStalling;
    }

    @Override
    public void passiveIntake() {
        if (!isStalling()) {
            betaCoralIntakeClosedLoop.setReference(SpeedConstants.BETA_CORAL_PASSIVE_SPEED.in(RPM),
                    ControlType.kVelocity);
        }
    }

    @Override
    public void updateStates(CoralIntakeIOStates states) {
        states.velocity = betaCoralIntakeEncoder.getVelocity();
        states.goalVelocity = velocityGoal;
        states.leftCurrent = betaCoralIntakeSparkFlex.getOutputCurrent();
        states.rightCurrent = betaCoralIntakeSparkFlex.getOutputCurrent();
        states.leftAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput()
                * betaCoralIntakeSparkFlex.getBusVoltage();
        states.rightAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput()
                * betaCoralIntakeSparkFlex.getBusVoltage();
    }
}
