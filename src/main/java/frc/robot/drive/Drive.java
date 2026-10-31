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

    // --- WPILib 差速底盤控制器 ---
    private final DifferentialDrive differentialDrive;

    public Drive() {
        // 1. 初始化馬達
        leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
        leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
        rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
        rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

        // 2. 設定主從跟隨 (Follower 模式)
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // 3. 設定馬達轉向 (依實際配線調整 true/false)
        leftLeader.setInverted(false);
        leftFollower.setInverted(false);
        rightLeader.setInverted(true);
        rightFollower.setInverted(true);

        // 4. 設定煞車模式
        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        // 5. 設定電流限制
        leftLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        leftLeader.enableCurrentLimit(true);
        rightLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        rightLeader.enableCurrentLimit(true);

        // 6. 初始化 WPILib DifferentialDrive
        differentialDrive = new DifferentialDrive(
            (output) -> leftLeader.set(ControlMode.PercentOutput, output),
            (output) -> rightLeader.set(ControlMode.PercentOutput, output)
        );
        differentialDrive.setSafetyEnabled(false);
    }

    /* ==================== 駕駛控制方法 ==================== */

    public void driveArcade(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void stop() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
    }

    /* ==================== Commands (指令封裝) ==================== */

    public Command arcadeDriveCommand(
        java.util.function.DoubleSupplier xSpeedSupplier,
        java.util.function.DoubleSupplier zRotationSupplier
    ) {
        // 直接使用 this.run() 呼叫 differentialDrive 的 arcadeDrive 方法
        return this.run(() -> {
            // 取得搖桿數值並加上平方率（讓低速操控更線性）
            double xSpeed = Math.copySign(Math.pow(xSpeedSupplier.getAsDouble(), 2), xSpeedSupplier.getAsDouble());
            double zRotation = Math.copySign(Math.pow(zRotationSupplier.getAsDouble(), 2), zRotationSupplier.getAsDouble());
            
            // 透過 differentialDrive 進行控制
            differentialDrive.arcadeDrive(xSpeed, zRotation);
        }).withName("Drive: ArcadeDrive");
    }
}