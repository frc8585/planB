package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// 匯入你的各個子系統
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.vision.Vision;
import frc.robot.index.Index; // 匯入剛剛寫好的 Index 子系統

public class RobotContainer {
    // --- 1. 宣告所有子系統 ---
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Index m_index = new Index(); // 宣告 Index 輸送馬達
    
    private final Vision m_vision = new Vision((pose, timestamp, stdDevs) -> {
        // 視覺座標融合 (若有實作可取消註解)
        // m_drive.addVisionMeasurement(pose, timestamp, stdDevs);
    });

    // --- 2. 宣告控制器 (Driver: 0, Operator: 1) ---
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // ==================== A. 底盤預設搖桿操控 ====================
        m_drive.setDefaultCommand(
            m_drive.arcadeDriveCommand(
                () -> -driverController.getLeftY(),  // 前進 / 後退
                () -> driverController.getRightX()   // 轉向
            )
        );

        // ==================== B. LT 鍵：放下 Intake 並開始吸球 ====================
        // 同時可以讓 Index 稍微跟著正轉帶入球 (選填，若不需要可拿掉 .alongWith)
        operatorController.leftTrigger(0.5).whileTrue(
            m_intake.deploy().alongWith(
                m_index.runIndexCommand()
            )
        );

        // ==================== C. LB 鍵：Index 反轉吐球 (Unjam / Outtake) ====================
        // 按住 LB 時 Index 會反轉吐球
        operatorController.leftBumper().whileTrue(
            m_index.reverseIndexCommand()
        );

        // ==================== D. RT 鍵：Index 順時針正轉送球（支援自動瞄準與射擊） ====================
        // 按住 RT 時：
        // 1. 底盤自動透過 Vision 瞄準 HUB 
        // 2. Shooter 預熱飛輪與仰角
        // 3. Index 直接啟動順時針正轉將球送進去！
        operatorController.rightTrigger(0.5).whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50), // 左飛輪轉速
                RotationsPerSecond.of(50), // 右飛輪轉速
                Rotations.of(0.25)         // 目標仰角
            ).alongWith(
                m_index.runIndexCommand(), // RT 同時觸發 Index 正轉送球
                m_drive.arcadeDriveCommand(
                    () -> -driverController.getLeftY(),
                    () -> m_vision.getTargetYaw(0) * 0.03 // 視覺自動對準
                )
            )
        );

        // ==================== E. RB 鍵：原地直接射擊 (不需視覺瞄準，包含 Index 正轉) ====================
        operatorController.rightBumper().whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50),
                RotationsPerSecond.of(50),
                Rotations.of(0.25)
            ).alongWith(
                m_index.runIndexCommand() // RB 同時觸發 Index 正轉送球
            )
        );
    }

    /**
     * 自主移動階段指令 (Autonomous)
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured.");
    }
}