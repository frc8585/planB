package frc.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // --- 硬體宣告 ---
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;
    private final TalonFX feederMotor;
    private final SimpleDifferentialMechanism<TalonFX> diffMechPivot;

    // --- 組態設定宣告 ---
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration pivotConfig;
    private final DifferentialMotorConstants<TalonFXConfiguration> diffConfigPivot;
    private final TalonFXConfiguration feederConfig;

    // --- 控制請求 ---
    private final VelocityVoltage leftFlywheelVelOut = new VelocityVoltage(0);
    private final VelocityVoltage rightFlywheelVelOut = new VelocityVoltage(0);
    private final VoltageOut feederVoltageOut = new VoltageOut(0);
    private final VoltageOut leftFlywheelVoltOut = new VoltageOut(0);
    private final VoltageOut rightFlywheelVoltOut = new VoltageOut(0);

    private final DifferentialMotionMagicVoltage pivotPositionOut = new DifferentialMotionMagicVoltage(0, 0);
    private final DifferentialVoltage pivotVoltageOut = new DifferentialVoltage(0, 0);

    // 記錄目標狀態
    private AngularVelocity targetLeftVel = RotationsPerSecond.of(0);
    private AngularVelocity targetRightVel = RotationsPerSecond.of(0);

    public Shooter() {
        leftFlywheelMotor = new TalonFX(FLYWHEEL_LEFT_ID, Constants.CAN_FD_BUS.getName());
        rightFlywheelMotor = new TalonFX(FLYWHEEL_RIGHT_ID, Constants.CAN_FD_BUS.getName());
        feederMotor = new TalonFX(FEEDER_ID, Constants.CAN_FD_BUS.getName());

        flywheelConfig = new TalonFXConfiguration()
            .withSlot0(FLYWHEEL_GAINS)
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(FLYWHEEL_OUTPUT_CONFIGS.NeutralMode))
            .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1));

        leftFlywheelMotor.getConfigurator().apply(flywheelConfig);
        rightFlywheelMotor.getConfigurator().apply(flywheelConfig);

        pivotConfig = new TalonFXConfiguration()
            .withSlot0(PIVOT_GAINS)
            .withSlot1(PIVOT_DIFF_GAINS)
            .withMotorOutput(PIVOT_OUTPUT_CONFIGS)
            .withCurrentLimits(PIVOT_CURRENT_LIMITS)
            .withMotionMagic(PIVOT_MOTION_MAGIC)
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1));

        diffConfigPivot = new DifferentialMotorConstants<TalonFXConfiguration>()
            .withCANBusName(Constants.CAN_FD_BUS.getName())
            .withLeaderId(PIVOT_LEFT_ID)
            .withFollowerId(PIVOT_RIGHT_ID)
            .withSensorToDifferentialRatio(1)
            .withAlignment(MotorAlignmentValue.Opposed)
            .withLeaderInitialConfigs(pivotConfig)
            .withFollowerInitialConfigs(new TalonFXConfiguration().withFeedback(pivotConfig.Feedback))
            .withFollowerUsesCommonLeaderConfigs(true);

        diffMechPivot = new SimpleDifferentialMechanism<TalonFX>(TalonFX::new, diffConfigPivot);

        feederConfig = new TalonFXConfiguration()
            .withCurrentLimits(FEEDER_CURRENT_LIMITS)
            .withMotorOutput(FEEDER_OUTPUT_CONFIGS);

        feederMotor.getConfigurator().apply(feederConfig);

        diffMechPivot.setPosition(0, 0);
    }

    /* ==================== Commands ==================== */

    /**
     * 發射：設定仰角與飛輪轉速，飛輪與仰角都到位才啟動餵料輪。
     * 飛輪卡死時自動反轉排除，放開按鍵就停止。
     */
    public Command shoot(AngularVelocity leftVel, AngularVelocity rightVel, Angle pivotAngle) {
        final Timer timer = new Timer();

        return Commands.sequence(
            this.startRun(
                timer::restart,
                () -> {
                    setPivotPosition(pivotAngle);
                    setFlywheelVelocities(leftVel, rightVel);

                    if (isFlywheelAtVelocity() && isPivotAtPosition(pivotAngle)) {
                        setFeederVoltage(FEEDER_SHOOT_VOLTAGE);
                    } else {
                        stopFeeder();
                    }
                }
            )
            // 起動後 1 秒內不判定卡死：飛輪剛起轉時電流高、轉速低，會被誤判
            .until(() -> timer.hasElapsed(1.0) && flywheelStalled()),
            unjamShooter()
        )
        .finallyDo(this::stopShooting)
        .withName("Shooter: Shoot");
    }

    public Command zeroPivotSequence() {
        return Commands.sequence(
            this.runOnce(() -> setPivotOutput(PIVOT_ZEROING_VOLTAGE)),
            Commands.waitSeconds(0.1),
            Commands.waitUntil(this::pivotStalled),
            this.runOnce(() -> {
                stopPivot();
                diffMechPivot.setPosition(0, 0);
                setPivotPosition(PIVOT_STOW_POS);
            })
        ).withName("Shooter: Zero Pivot");
    }

    private Command unjamShooter() {
        return Commands.sequence(
            this.runOnce(() -> {
                setFlywheelVoltage(Volts.of(-3), Volts.of(-3));
                setFeederVoltage(FEEDER_SHOOT_VOLTAGE.unaryMinus());
            }),
            Commands.waitSeconds(0.5),
            this.runOnce(this::stopShooting)
        ).withName("Shooter: Unjam");
    }

    /* ==================== Status ==================== */

    public boolean isFlywheelAtVelocity() {
        double leftCurr = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightCurr = rightFlywheelMotor.getVelocity().getValueAsDouble();

        return Math.abs(leftCurr - targetLeftVel.in(RotationsPerSecond)) < FLYWHEEL_TOLERANCE.in(RotationsPerSecond)
            && Math.abs(rightCurr - targetRightVel.in(RotationsPerSecond)) < FLYWHEEL_TOLERANCE.in(RotationsPerSecond);
    }

    public boolean isPivotAtPosition(Angle targetAngle) {
        return diffMechPivot.getAveragePosition().getValue().isNear(targetAngle, PIVOT_TOLERANCE);
    }

    public boolean flywheelStalled() {
        double current = leftFlywheelMotor.getStatorCurrent().getValueAsDouble();
        double velocity = leftFlywheelMotor.getVelocity().getValueAsDouble();
        return current >= FLYWHEEL_STALL_CURRENT.in(Amps) && Math.abs(velocity) < FLYWHEEL_STALL_VEL.in(RotationsPerSecond);
    }

    public boolean pivotStalled() {
        double current = diffMechPivot.getLeader().getStatorCurrent().getValueAsDouble();
        double velocity = diffMechPivot.getAverageVelocity().getValueAsDouble();
        return current >= PIVOT_STALL_CURRENT.in(Amps) && Math.abs(velocity) < PIVOT_STALL_VEL.in(RadiansPerSecond);
    }

    /* ==================== Control ==================== */

    public void setFlywheelVelocities(AngularVelocity leftVel, AngularVelocity rightVel) {
        this.targetLeftVel = leftVel;
        this.targetRightVel = rightVel;
        leftFlywheelMotor.setControl(leftFlywheelVelOut.withVelocity(leftVel));
        rightFlywheelMotor.setControl(rightFlywheelVelOut.withVelocity(rightVel));
    }

    public void setFlywheelVoltage(Voltage leftVoltage, Voltage rightVoltage) {
        leftFlywheelMotor.setControl(leftFlywheelVoltOut.withOutput(leftVoltage));
        rightFlywheelMotor.setControl(rightFlywheelVoltOut.withOutput(rightVoltage));
    }

    public void setPivotPosition(Angle position) {
        diffMechPivot.setControl(pivotPositionOut.withAveragePosition(position));
    }

    public void setPivotOutput(Voltage voltage) {
        diffMechPivot.setControl(pivotVoltageOut.withAverageOutput(voltage));
    }

    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setControl(feederVoltageOut.withOutput(voltage));
    }

    public void stopFlywheel() {
        targetLeftVel = RotationsPerSecond.of(0);
        targetRightVel = RotationsPerSecond.of(0);
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
    }

    public void stopPivot() {
        diffMechPivot.setCoastOut();
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    /** 射擊結束：只停飛輪與餵料輪，仰角維持在最後的位置，不放 Coast */
    public void stopShooting() {
        stopFlywheel();
        stopFeeder();
    }

    public void stopAll() {
        stopShooting();
        stopPivot();
    }

    public double getLeftFlywheelRps() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public double getRightFlywheelRps() {
        return rightFlywheelMotor.getVelocity().getValueAsDouble();
    }
}