package frc.robot.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // --- 硬體宣告 (使用 Phoenix 5 的 WPI_TalonSRX) ---
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX leftFollower;
    private final WPI_TalonSRX rightLeader;
    private final WPI_TalonSRX rightFollower;

    // --- WPILib 差速底盤運動學 helper ---
    private final DifferentialDrive differentialDrive;

    public Drive() {
        // 1. 初始化馬達 (Talon SRX 連接在 roboRIO 的標準 CAN 總線上)
        leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
        leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
        rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
        rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

        // 2. 恢復原廠預設設定，避免殘留設定影響
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // 3. 設定主從跟隨 (Follower 模式)
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // 4. 設定馬達轉向 (CIM 差動底盤通常一側需要反轉，讓前進時左右輪同向)
        leftLeader.setInverted(false);
        leftFollower.setInverted(false);
        rightLeader.setInverted(true);
        rightFollower.setInverted(true);

        // 5. 設定煞車模式 (Brake 模式讓機器人停止時不易被推動)
        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        // 6. 設定電流限制保護 CIM 馬達
        leftLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        leftLeader.enableCurrentLimit(true);
        rightLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        rightLeader.enableCurrentLimit(true);

        // 7. 初始化 WPILib DifferentialDrive
        // 傳入左右 Leader 的控制 lambda (接收 -1.0 到 1.0 的百分比輸出)
        differentialDrive = new DifferentialDrive(
            (output) -> leftLeader.set(ControlMode.PercentOutput, output),
            (output) -> rightLeader.set(ControlMode.PercentOutput, output)
        );

        differentialDrive.setSafetyEnabled(false);
    }

    /**
     * Arcade 駕駛模式（一隻搖桿控制前進/後退與轉向）
     * @param xSpeed 前進/後退速度 (-1.0 到 1.0)
     * @param zRotation 轉向速度 (-1.0 到 1.0)
     */
    public void driveArcade(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Tank 駕駛模式（左右搖桿分別控制左右輪）
     */
    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * 停止底盤
     */
    public void stop() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
    }

    /* ==================== Commands (指令封裝) ==================== */

    /**
     * 綁定手把操作的預設底盤指令
     */
    public Command arcadeDriveCommand(
        java.util.function.DoubleSupplier xSpeedSupplier,
        java.util.function.DoubleSupplier zRotationSupplier
    ) {
        return this.run(() -> {
            // 加入平方運算（Squared Inputs），讓低速時操控更細緻平滑
            double xSpeed = Math.copySign(Math.pow(xSpeedSupplier.getAsDouble(), 2), xSpeedSupplier.getAsDouble());
            double zRotation = Math.copySign(Math.pow(zRotationSupplier.getAsDouble(), 2), zRotationSupplier.getAsDouble());
            
            driveArcade(xSpeed, zRotation);
        }).withName("Drive: ArcadeDrive");
    }
}