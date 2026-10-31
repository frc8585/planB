package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// 匯入你的各個子系統
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.vision.Vision;
import frc.robot.index.Index;

public class RobotContainer {
    // --- 1. 宣告所有子系統 ---
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Index m_index = new Index();
    
    private final Vision m_vision = new Vision((pose, timestamp, stdDevs) -> {
        // 視覺座標融合 (若有實作可取消註解)
        // m_drive.addVisionMeasurement(pose, timestamp, stdDevs);
    });

    // --- 2. 宣告控制器與 Auto 選擇器 ---
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // 2. 建立按鈕與控制綁定
        configureBindings();

        // 3. 建立網頁端（Shuffleboard / SmartDashboard）的自動程式下拉選單
        autoChooser = AutoBuilder.buildAutoChooser("Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        operatorController.leftTrigger(0.5).whileTrue(
            m_intake.deploy().alongWith(
                m_index.runIndexCommand()
            )
        );

        // ==================== C. LB 鍵：Intake 與 Index 同時反轉吐球 (Unjam 卡彈解除) ====================
        operatorController.leftBumper().whileTrue(
            m_intake.reverseCommand().alongWith(
                m_index.reverseIndexCommand()
            )
        );

        // ==================== D. RT 鍵：自動瞄準 HUB 並射擊 + Index 正轉 ====================
        operatorController.rightTrigger(0.5).whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50),
                RotationsPerSecond.of(50),
                Rotations.of(0.25)
            ).alongWith(
                m_index.runIndexCommand(),
                m_drive.arcadeDriveCommand(
                    () -> -driverController.getLeftY(),
                    () -> m_vision.getHubTargetYaw(0).orElse(0.0) * 0.03
                )
            )
        );

        // ==================== E. RB 鍵：原地直接射擊 + Index 正轉 ====================
        operatorController.rightBumper().whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50),
                RotationsPerSecond.of(50),
                Rotations.of(0.25)
            ).alongWith(
                m_index.runIndexCommand()
            )
        );

        // ==================== F. Y 鍵：Intake 單獨收回 ====================
        operatorController.y().onTrue(m_intake.stow());
    }

    /**
     * 自主移動階段指令 (Autonomous)
     * 會從 Shuffleboard 上的下拉選單抓取玩家選定的 Auto 點圖執行
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}