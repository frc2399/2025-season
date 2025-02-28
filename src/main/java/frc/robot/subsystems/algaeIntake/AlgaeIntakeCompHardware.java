package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeIntakeCompHardware implements AlgaeIntakeIO {
    private final SparkMax algaeIntakeSparkMax;
    private final SparkClosedLoopController compAlgaeIntakeClosedLoop;
    private final RelativeEncoder compAlgaeIntakeRelativeEncoder;

    private static final SparkMaxConfig compAlgaeIntakeConfig = new SparkMaxConfig();

    private static final boolean COMP_ALGAE_INTAKE_MOTOR_INVERTED = false;
    private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final double COMP_ALGAE_INTAKE_POSITION_CONVERSION_FACTOR = 0;
    private static final double COMP_ALGAE_INTAKE_VELOCITY_CONVERSION_FACTOR = 0;

    private static final double COMP_ALGAE_INTAKE_P = 0.001;
    private static final double COMP_ALGAE_INTAKE_I = 0;
    private static final double COMP_ALGAE_INTAKE_D = 0;
    private static final double COMP_ALGAE_INTAKE_FeedForward = 0;

    private static final double COMP_ALGAE_INTAKE_MIN_INPUT = 1;
    private static final double COMP_ALGAE_INTAKE_MAX_OUTPUT = -1;

    private static final boolean COMP_ALGAE_INTAKE_POSITION_WRAPPING_ENABLED = true;

    public AlgaeIntakeCompHardware() {
        compAlgaeIntakeConfig.inverted(COMP_ALGAE_INTAKE_MOTOR_INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        compAlgaeIntakeConfig.encoder.positionConversionFactor(COMP_ALGAE_INTAKE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(COMP_ALGAE_INTAKE_VELOCITY_CONVERSION_FACTOR);
        compAlgaeIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(COMP_ALGAE_INTAKE_P, COMP_ALGAE_INTAKE_I, COMP_ALGAE_INTAKE_D, COMP_ALGAE_INTAKE_FeedForward)
                .outputRange(COMP_ALGAE_INTAKE_MIN_INPUT, COMP_ALGAE_INTAKE_MAX_OUTPUT)
                .positionWrappingEnabled(COMP_ALGAE_INTAKE_POSITION_WRAPPING_ENABLED);

        algaeIntakeSparkMax = new SparkMax(MotorIdConstants.ALGAE_BETA_INTAKE_CAN_ID, MotorType.kBrushless);
        algaeIntakeSparkMax.configure(compAlgaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        compAlgaeIntakeClosedLoop = algaeIntakeSparkMax.getClosedLoopController();
        compAlgaeIntakeRelativeEncoder = algaeIntakeSparkMax.getEncoder();
    }

    @Override
    public void setRollerSpeed(AngularVelocity speed) {
        compAlgaeIntakeClosedLoop.setReference(speed.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void intake() {
        setRollerSpeed(SpeedConstants.COMP_ALGAE_INTAKE_SPEED);
    }

    @Override
    public void outtake() {
        setRollerSpeed(SpeedConstants.COMP_ALGAE_OUTTAKE_SPEED);
    }

    @Override
    public void updateStates(AlgaeIntakeIOStates states) {
        states.intakeVelocity = getVelocity();
        states.leftAppliedVoltage = algaeIntakeSparkMax.getAppliedOutput()
                                * algaeIntakeSparkMax.getBusVoltage();
        states.leftCurrent = algaeIntakeSparkMax.getOutputCurrent();
    }

    private double getVelocity() {
        return compAlgaeIntakeRelativeEncoder.getVelocity();
    }

    @Override
    public boolean isStalling() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isStalling'");
    }
}
