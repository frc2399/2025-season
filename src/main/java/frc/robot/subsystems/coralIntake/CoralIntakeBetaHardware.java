package frc.robot.subsystems.coralIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.CommandFactory.Setpoint;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

//intake to stall
public class CoralIntakeBetaHardware implements CoralIntakeIO {

    private static final boolean BETA_CORAL_INTAKE_MOTOR_INVERTED = true;
    private static final IdleMode BETA_CORAL_INTAKE_IDLE_MODE = IdleMode.kBrake;

    // 5:1 gearbox ratio
    private static final double BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR = 2 * Math.PI / 5.0; // radians
    private static final double BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 5.0 / 60; // radians per
                                                                                                       // second

    private static final double BETA_CORAL_INTAKE_P = 0.000018;
    private static final double BETA_CORAL_INTAKE_I = 0;
    private static final double BETA_CORAL_INTAKE_D = 0;
    private static final double BETA_CORAL_INTAKE_FF = 0.001;
    private static final double BETA_CORAL_INTAKE_PID_MIN_OUTPUT = -1.0;
    private static final double BETA_CORAL_INTAKE_PID_MAX_OUTPUT = 1.0;

    private static final boolean BETA_CORAL_INTAKE_POSITION_WRAPPING_ENABLED = true;

    private final RelativeEncoder betaCoralIntakeEncoder;
    private final SparkClosedLoopController betaCoralIntakeClosedLoop;
    private final SparkFlexConfig betaCoralIntakeConfig = new SparkFlexConfig();

    private final SparkFlex betaCoralIntakeSparkFlex;

    private double velocityGoal = 0;

    private static final Time BETA_CORAL_DEBOUNCER_TIME = Seconds.of(0.18);
    private static final Current CORAL_INTAKE_STALL_THRESHOLD = Amps.of(16);
    private static final Debouncer CORAL_BETA_DEBOUNCER = new Debouncer(BETA_CORAL_DEBOUNCER_TIME.in(Seconds));

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
                .calculate(betaCoralIntakeSparkFlex.getOutputCurrent() > CORAL_INTAKE_STALL_THRESHOLD.in(Amps));
        return isStalling;
    }

    @Override
    public void passiveIntake() {
        if (!isStalling()) {
            betaCoralIntakeClosedLoop.setReference(SpeedConstants.BETA_CORAL_PASSIVE_SPEED.in(RPM), ControlType.kVelocity);
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
