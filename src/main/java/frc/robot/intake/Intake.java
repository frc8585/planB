package frc.robot.intake;

import static frc.robot.Constants.IntakeConstants.LEFT_RACK_ID;
import static frc.robot.Constants.IntakeConstants.RIGHT_RACK_ID;
import static frc.robot.Constants.IntakeConstants.LEFT_SPIN_ID;
import static frc.robot.Constants.IntakeConstants.RIGHT_SPIN_ID;
import static frc.robot.Constants.IntakeConstants.RACK_GAINS;
import static frc.robot.Constants.IntakeConstants.RACK_DIFF_GAINS;
import static frc.robot.Constants.IntakeConstants.RACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.RACK_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.RACK_MOTION_MAGIC;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POS;
import static frc.robot.Constants.IntakeConstants.STOW_POS;
import static frc.robot.Constants.IntakeConstants.SPIN_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.SPIN_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.SPIN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.UNJAM_SPIN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.ZEROING_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.RACK_STALL_VEL;
import static frc.robot.Constants.IntakeConstants.SPIN_STALL_CURRENT;
import static frc.robot.Constants.IntakeConstants.RACK_STALL_CURRENT;
import static frc.robot.Constants.IntakeConstants.STOW_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.SPIN_STALL_ANGULAR_VELOCITY;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final SimpleDifferentialMechanism<TalonFX> diffMechRack;
    private final SimpleDifferentialMechanism<TalonFX> diffMechSpin;

    private final TalonFXConfiguration rackConfig;
    private final TalonFXConfiguration spinConfig;
    private final DifferentialMotorConstants<TalonFXConfiguration> diffConfigRack;
    private final DifferentialMotorConstants<TalonFXConfiguration> diffConfigSpin;

    private final DifferentialMotionMagicVoltage rackPositionOut = new DifferentialMotionMagicVoltage(0, 0);
    private final DifferentialVoltage rackVoltageOut = new DifferentialVoltage(0, 0);
    private final DifferentialVoltage spinVoltageOut = new DifferentialVoltage(0, 0);

    public Intake(){
        rackConfig = new TalonFXConfiguration()
            .withSlot0(RACK_GAINS)
            .withSlot1(RACK_DIFF_GAINS)
            .withMotorOutput(RACK_OUTPUT_CONFIGS)
            .withCurrentLimits(RACK_CURRENT_LIMITS)
            .withMotionMagic(RACK_MOTION_MAGIC)
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(DEPLOY_POS)
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(STOW_POS))
            .withFeedback(new FeedbackConfigs()
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(1));

        diffConfigRack = new DifferentialMotorConstants<TalonFXConfiguration>()
            .withCANBusName(Constants.CAN_FD_BUS.getName())
            .withLeaderId(LEFT_RACK_ID)
            .withFollowerId(RIGHT_RACK_ID)
            .withSensorToDifferentialRatio(1)
            .withAlignment(MotorAlignmentValue.Opposed)
            .withLeaderInitialConfigs(rackConfig)
            .withFollowerInitialConfigs(new TalonFXConfiguration().withFeedback(rackConfig.Feedback))
            .withFollowerUsesCommonLeaderConfigs(true);

        spinConfig = new TalonFXConfiguration()
            .withCurrentLimits(SPIN_CURRENT_LIMITS)
            .withMotorOutput(SPIN_OUTPUT_CONFIGS);

        diffConfigSpin = new DifferentialMotorConstants<TalonFXConfiguration>()
            .withCANBusName(Constants.CAN_FD_BUS.getName())
            .withLeaderId(LEFT_SPIN_ID)
            .withFollowerId(RIGHT_SPIN_ID)
            .withSensorToDifferentialRatio(1)
            .withAlignment(MotorAlignmentValue.Opposed)
            .withLeaderInitialConfigs(spinConfig)
            .withFollowerInitialConfigs(new TalonFXConfiguration().withFeedback(spinConfig.Feedback))
            .withFollowerUsesCommonLeaderConfigs(true);

        diffMechSpin = new SimpleDifferentialMechanism<TalonFX>(TalonFX::new, diffConfigSpin);
        diffMechRack = new SimpleDifferentialMechanism<TalonFX>(TalonFX::new, diffConfigRack);

        diffMechRack.setPosition(0, 0);
    }

    public Command deploy() {
        return this.runEnd(
            () -> {
                setRackPosition(DEPLOY_POS);
                setSpinOutput(SPIN_VOLTAGE);
            },
            this::stopIntake
        )
        .until(this::spinStalled).andThen(unjamSpin())
        .until(this::rackStalled).andThen(unjamRackDeploying())
        .withName("Deploy Right Intake");
    }

    /**
     * Intake 反轉吐球指令 (供 LB 鍵 Unjam 使用)
     */
    public Command reverseCommand() {
        return this.runEnd(
            () -> setSpinOutput(SPIN_VOLTAGE.unaryMinus()), // 反轉滾輪
            this::stopIntake                                // 放開時停止
        ).withName("Intake: Reverse");
    }

    public Command zeroSequence() {
        return Commands.sequence(
            this.runOnce(() -> setRackOutput(ZEROING_VOLTAGE)),
            Commands.waitSeconds(0.1),
            Commands.waitUntil(this::rackStalled),
            this.runOnce(() -> {
                stopIntake();
                zeroPosition();
                setRackPosition(STOW_POS);
            })
        ).withName("Zero Right Intake");
    }

    private Command unjamSpin() {
        return Commands.sequence(
            this.runOnce(() -> setSpinOutput(UNJAM_SPIN_VOLTAGE.unaryMinus())),
            Commands.waitUntil(() -> !spinStalled()),
            this.runOnce(this::stopIntake)
        ).withName("Intake:Unjam Spin Right");
    }

    private Command unjamRackDeploying() {
        return Commands.sequence(
            this.runOnce(() -> setRackPosition(STOW_POS)),
            Commands.waitUntil(() -> !rackStalled()),
            this.runOnce(() -> setRackPosition(DEPLOY_POS))
        ).withName("Intake:Unjam Rack Deploying Right");
    }

    public boolean rackStalled() {
        double current = diffMechRack.getLeader().getStatorCurrent().getValueAsDouble();
        double velocity = diffMechRack.getAverageVelocity().getValueAsDouble();
        
        return current >= RACK_STALL_CURRENT.in(Amps)
            && Math.abs(velocity) < RACK_STALL_VEL.in(RadiansPerSecond);
    }

    public boolean isStowed() {
        return diffMechRack.getAveragePosition().getValue()
            .isNear(STOW_POS, STOW_TOLERANCE);
    }

    public boolean spinStalled() {
        double current = diffMechSpin.getLeader().getStatorCurrent().getValueAsDouble();
        double velocity = diffMechSpin.getLeader().getVelocity().getValueAsDouble();
        
        return current >= SPIN_STALL_CURRENT.in(Amps)
            && Math.abs(velocity) < SPIN_STALL_ANGULAR_VELOCITY.in(RadiansPerSecond);
    }

    public void setRackPosition(Angle position) {
        diffMechRack.setControl(rackPositionOut.withAveragePosition(position));
    }

    public void setRackOutput(Voltage out) {
        diffMechRack.setControl(rackVoltageOut.withAverageOutput(out));
    }

    public void setSpinOutput(Voltage out) {
        diffMechSpin.setControl(spinVoltageOut.withAverageOutput(out));
    }

    public void stopSpin() {
        diffMechSpin.setCoastOut();
    }

    public void stopRack() {
        diffMechRack.setCoastOut();
    }

    public void stopIntake() {
        stopSpin();
        stopRack();
    }

    public void zeroPosition() {
        diffMechRack.setPosition(0, 0);
    }
}