package frc.robot.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // --- 硬體宣告 (使用 Phoenix 5 的 WPI_TalonSRX) ---
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX leftFollower;
    private final WPI_TalonSRX rightLeader;
    private final WPI_TalonSRX rightFollower;

    // --- WPILib 差速底盤運動學與定位 helper ---
    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveOdometry odometry;

    // 模擬內建 Gyro（若有實際 NavX / Pigeon 可替換，此處先用內建或預設 0 度）
    private double simulatedHeading = 0.0;

    public Drive() {
        // 1. 初始化馬達
        leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
        leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
        rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
        rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

        // 2. 恢復原廠預設設定
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // 3. 設定主從跟隨 (Follower 模式)
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // 4. 設定馬達轉向
        leftLeader.setInverted(false);
        leftFollower.setInverted(false);
        rightLeader.setInverted(true);
        rightFollower.setInverted(true);

        // 5. 設定煞車模式
        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        // 6. 設定電流限制
        leftLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        leftLeader.enableCurrentLimit(true);
        rightLeader.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
        rightLeader.enableCurrentLimit(true);

        // 7. 初始化 WPILib DifferentialDrive
        differentialDrive = new DifferentialDrive(
            (output) -> leftLeader.set(ControlMode.PercentOutput, output),
            (output) -> rightLeader.set(ControlMode.PercentOutput, output)
        );
        differentialDrive.setSafetyEnabled(false);

        // 8. 初始化運動學與里程計 (假設輪距 trackwidth = 0.6公尺，可至 Constants 調整)
        kinematics = new DifferentialDriveKinematics(0.6);
        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(0.0), 
            0.0, 
            0.0
        );
    }

    @Override
    public void periodic() {
        // 每個周期更新底盤里程計 (利用 Encoder 數值更新全場座標)
        // 註：若 TalonSRX 有接 Encoder，可將 getSelectedSensorPosition 轉換為公尺帶入
        odometry.update(
            Rotation2d.fromDegrees(simulatedHeading),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );
    }

    /* ==================== PathPlanner 必備介面方法 ==================== */

    /**
     * 1. 取得機器人目前座標
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * 2. 重置機器人座標
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(simulatedHeading), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
    }

    /**
     * 3. 取得機器人相對速度 (ChassisSpeeds)
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(),
            getRightVelocityMetersPerSecond()
        );
        return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    /**
     * 4. 接收 PathPlanner 算出的速度指令並驅動馬達 (支援自動導航)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        
        // 將左右輪速度轉為百分比輸出 (-1.0 到 1.0) 傳給底盤
        double maxSpeed = 3.0; // 假設最大速度 3 m/s
        double leftOutput = wheelSpeeds.leftMetersPerSecond / maxSpeed;
        double rightOutput = wheelSpeeds.rightMetersPerSecond / maxSpeed;

        differentialDrive.tankDrive(leftOutput, rightOutput, false);
    }

    /* ==================== 輔助計算方法 (Encoder 換算) ==================== */
    private double getLeftDistanceMeters() {
        // 範例：將 TalonSRX 脈衝數轉換為公尺 (可依實際齒輪比與輪徑調整)
        return (leftLeader.getSelectedSensorPosition() / 4096.0) * (Math.PI * 0.1);
    }

    private double getRightDistanceMeters() {
        return (rightLeader.getSelectedSensorPosition() / 4096.0) * (Math.PI * 0.1);
    }

    private double getLeftVelocityMetersPerSecond() {
        return (leftLeader.getSelectedSensorVelocity() * 10.0 / 4096.0) * (Math.PI * 0.1);
    }

    private double getRightVelocityMetersPerSecond() {
        return (rightLeader.getSelectedSensorVelocity() * 10.0 / 4096.0) * (Math.PI * 0.1);
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
        return this.run(() -> {
            double xSpeed = Math.copySign(Math.pow(xSpeedSupplier.getAsDouble(), 2), xSpeedSupplier.getAsDouble());
            double zRotation = Math.copySign(Math.pow(zRotationSupplier.getAsDouble(), 2), zRotationSupplier.getAsDouble());
            
            driveArcade(xSpeed, zRotation);
        }).withName("Drive: ArcadeDrive");
    }
}