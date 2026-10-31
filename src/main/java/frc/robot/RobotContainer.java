package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.drive.Drive;
import frc.robot.index.Index;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.vision.Vision;

public class RobotContainer {
    // --- 子系統 ---
    private final Drive m_drive = new Drive();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Index m_index = new Index();

    private final Vision m_vision = new Vision((pose, timestamp, stdDevs) -> {
        // 視覺定位融合（Drive 有 pose estimator 之後再接）
        // m_drive.addVisionMeasurement(pose, timestamp, stdDevs);
    });

    // --- 控制器與 Auto 選擇器 ---
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    private void configureBindings() {
        // ==================== A. 底盤預設搖桿操控 ====================
        m_drive.setDefaultCommand(
            m_drive.arcadeDriveCommand(
                () -> -driverController.getLeftY(),
                () -> driverController.getRightX()
            )
        );

        // ==================== B. LT：放下 Intake 並吸球（Index 同時轉） ====================
        operatorController.leftTrigger(0.5).whileTrue(
            m_intake.deploy().alongWith(m_index.runIndexCommand())
        );

        // ==================== C. LB：Intake 與 Index 反轉吐球 ====================
        operatorController.leftBumper().whileTrue(
            m_intake.reverseCommand().alongWith(m_index.reverseIndexCommand())
        );

        // ==================== D. RT：視覺自動瞄準 HUB 並射擊 ====================
        operatorController.rightTrigger(0.5).whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50),
                RotationsPerSecond.of(50),
                Rotations.of(0.25)
            ).alongWith(
                m_index.runIndexCommand(),
                m_drive.aimDriveCommand(
                    () -> -driverController.getLeftY(),
                    () -> m_vision.getHubTargetYaw(0).orElse(0.0) * 0.03
                )
            )
        );

        // ==================== E. RB：不靠視覺，原地直接射擊 ====================
        operatorController.rightBumper().whileTrue(
            m_shooter.shoot(
                RotationsPerSecond.of(50),
                RotationsPerSecond.of(50),
                Rotations.of(0.25)
            ).alongWith(m_index.runIndexCommand())
        );

        // ==================== F. Y：Intake 單獨收回 ====================
        operatorController.y().onTrue(m_intake.stow());
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward 2s",
            m_drive.run(() -> m_drive.driveArcade(0.4, 0.0))
                .withTimeout(2.0)
                .andThen(m_drive.runOnce(m_drive::stop)));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}