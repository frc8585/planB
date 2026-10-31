package frc.robot.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX leftFollower;
    private final WPI_TalonSRX rightLeader;
    private final WPI_TalonSRX rightFollower;

    private final DifferentialDrive differentialDrive;

    public Drive() {
        leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
        leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
        rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
        rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

        // 轉向：只設 Leader，Follower 一律跟隨 Leader
        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.setInverted(InvertType.FollowMaster);

        for (WPI_TalonSRX m : new WPI_TalonSRX[] { leftLeader, leftFollower, rightLeader, rightFollower }) {
            m.setNeutralMode(NeutralMode.Brake);
            // 電流限制：四顆都要設（Follower 是獨立的控制器）
            m.configContinuousCurrentLimit(CURRENT_LIMIT_AMPS);
            m.configPeakCurrentLimit(CURRENT_LIMIT_AMPS + 10);
            m.configPeakCurrentDuration(100); // ms
            m.enableCurrentLimit(true);
        }

        differentialDrive = new DifferentialDrive(
            (output) -> leftLeader.set(ControlMode.PercentOutput, output),
            (output) -> rightLeader.set(ControlMode.PercentOutput, output)
        );
        differentialDrive.setSafetyEnabled(false);
    }

    /* ==================== 駕駛控制方法 ==================== */

    /** 直接控制（不再額外平方）。自動程式用 */
    public void driveArcade(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation, false);
    }

    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
    }

    public void stop() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
    }

    /* ==================== Commands ==================== */

    /** 搖桿駕駛：前進與轉向都做平方，讓低速更好控制 */
    public Command arcadeDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier zRotationSupplier) {
        return this.run(() -> {
            double x = xSpeedSupplier.getAsDouble();
            double z = zRotationSupplier.getAsDouble();
            // 這裡已手動平方，所以呼叫時 squareInputs 要傳 false，否則會變四次方
            differentialDrive.arcadeDrive(Math.copySign(x * x, x), Math.copySign(z * z, z), false);
        }).withName("Drive: ArcadeDrive");
    }

    /** 自動瞄準：前進仍平方，轉向「不」平方（輸出與誤差成正比），並限制最大轉速 */
    public Command aimDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier zRotationSupplier) {
        return this.run(() -> {
            double x = xSpeedSupplier.getAsDouble();
            double z = MathUtil.clamp(zRotationSupplier.getAsDouble(), -0.5, 0.5);
            differentialDrive.arcadeDrive(Math.copySign(x * x, x), z, false);
        }).withName("Drive: Aim");
    }
}