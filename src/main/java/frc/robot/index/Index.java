package frc.robot.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
    // --- 硬體宣告 ---
    private final TalonFX indexMotor;

    // --- 控制請求 ---
    private final VoltageOut voltageOut = new VoltageOut(0);

    public Index() {
        // 1. 初始化馬達 (使用 CAN FD 總線)
        indexMotor = new TalonFX(INDEX_MOTOR_ID, Constants.CAN_FD_BUS.getName());

        // 2. 套用組態設定
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(INDEX_OUTPUT_CONFIGS)
            .withCurrentLimits(INDEX_CURRENT_LIMITS)
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(1));

        indexMotor.getConfigurator().apply(config);
    }

    /**
     * 直接指定電壓控制 Index 運轉
     */
    public void setVoltage(Voltage voltage) {
        indexMotor.setControl(voltageOut.withOutput(voltage.in(Volts)));
    }

    /**
     * 停止 Index 馬達
     */
    public void stop() {
        indexMotor.stopMotor();
    }

    /* ==================== Commands (指令封裝) ==================== */

    /**
     * 順時針正轉送球指令（按住時執行，放開時停止）
     */
    public Command runIndexCommand() {
        return this.startEnd(
            () -> setVoltage(INDEX_RUN_VOLTAGE),
            this::stop
        ).withName("Index: Run Forward");
    }

    /**
     * 反轉吐球指令（用於 Unjam / 清除卡球）
     */
    public Command reverseIndexCommand() {
        return this.startEnd(
            () -> setVoltage(INDEX_REVERSE_VOLTAGE),
            this::stop
        ).withName("Index: Run Reverse");
    }
}