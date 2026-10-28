package frc.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.FEEDER_CURRENT_LIMITS;
import static frc.robot.Constants.ShooterConstants.FEEDER_ID;
import static frc.robot.Constants.ShooterConstants.FEEDER_OUTPUT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FEEDER_SHOOT_VOLTAGE;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMITS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_GAINS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_OUTPUT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STALL_CURRENT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STALL_VEL;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.TARGET_SHOOT_VELOCITY;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;
    private final TalonFX feederMotor;

    private final VelocityVoltage flywheelVelocityOut = new VelocityVoltage(0);
    private final VoltageOut feederVoltageOut = new VoltageOut(0);

    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration rightFlywheelConfig;
    private final TalonFXConfiguration feederConfig;

    private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

    public Shooter() {
        leftFlywheelMotor = new TalonFX(FLYWHEEL_LEFT_ID, Constants.CAN_FD_BUS.getName());
        rightFlywheelMotor = new TalonFX(FLYWHEEL_RIGHT_ID, Constants.CAN_FD_BUS.getName());
        feederMotor = new TalonFX(FEEDER_ID, Constants.CAN_FD_BUS.getName());

        // 左飛輪基本設定
        flywheelConfig = new TalonFXConfiguration()
            .withSlot0(FLYWHEEL_GAINS)
            .withMotorOutput(FLYWHEEL_OUTPUT_CONFIGS)
            .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
            .withFeedback(new FeedbackConfigs()
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(1));

        leftFlywheelMotor.getConfigurator().apply(flywheelConfig);

        // 右飛輪設定（獨立建立組態並設定反轉）
        rightFlywheelConfig = new TalonFXConfiguration()
            .withSlot0(FLYWHEEL_GAINS)
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(FLYWHEEL_OUTPUT_CONFIGS.NeutralMode))
            .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
            .withFeedback(new FeedbackConfigs()
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(1));

        rightFlywheelMotor.getConfigurator().apply(rightFlywheelConfig);
        
        // 修正：使用帶有 MotorAlignmentValue 的 Follower 建構子
        rightFlywheelMotor.setControl(new Follower(leftFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // 餵料馬達設定
        feederConfig = new TalonFXConfiguration()
            .withCurrentLimits(FEEDER_CURRENT_LIMITS)
            .withMotorOutput(FEEDER_OUTPUT_CONFIGS);

        feederMotor.getConfigurator().apply(feederConfig);
    }

    public Command shoot() {
        return this.runEnd(
            () -> {
                setFlywheelVelocity(TARGET_SHOOT_VELOCITY);
                if (isFlywheelAtVelocity()) {
                    setFeederVoltage(FEEDER_SHOOT_VOLTAGE);
                } else {
                    stopFeeder();
                }
            },
            this::stopAll
        )
        .until(this::flywheelStalled).andThen(unjamShooter())
        .withName("Shooter: Shoot");
    }

    private Command unjamShooter() {
        return Commands.sequence(
            this.runOnce(() -> {
                setFlywheelVoltage(Volts.of(-3));
                setFeederVoltage(FEEDER_SHOOT_VOLTAGE.unaryMinus());
            }),
            Commands.waitSeconds(0.5),
            this.runOnce(this::stopAll)
        ).withName("Shooter: Unjam");
    }

    public boolean isFlywheelAtVelocity() {
        double currentVel = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double targetVel = targetVelocity.in(RotationsPerSecond);
        return Math.abs(currentVel - targetVel) < FLYWHEEL_TOLERANCE.in(RotationsPerSecond);
    }

    public boolean flywheelStalled() {
        double current = leftFlywheelMotor.getStatorCurrent().getValueAsDouble();
        double velocity = leftFlywheelMotor.getVelocity().getValueAsDouble();

        return current >= FLYWHEEL_STALL_CURRENT.in(Amps)
            && Math.abs(velocity) < FLYWHEEL_STALL_VEL.in(RotationsPerSecond);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        this.targetVelocity = velocity;
        leftFlywheelMotor.setControl(flywheelVelocityOut.withVelocity(velocity));
    }

    public void setFlywheelVoltage(Voltage voltage) {
        leftFlywheelMotor.setControl(new VoltageOut(voltage.in(Volts)));
    }

    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setControl(feederVoltageOut.withOutput(voltage));
    }

    public void stopFlywheel() {
        leftFlywheelMotor.stopMotor();
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    public void stopAll() {
        stopFlywheel();
        stopFeeder();
    }
}