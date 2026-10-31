package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static final CANBus CAN_FD_BUS = new CANBus("base1");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

    public static final class DriveConstants {
        // --- 底盤馬達 CAN ID ---
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        // --- 連續電流與電流限制 ---
        public static final int CURRENT_LIMIT_AMPS = 30;
    }

    public static final class IndexConstants {
    public static final int INDEX_MOTOR_ID = 15;

    // --- 馬達輸出與轉向設定 (順時針正轉) ---
    public static final MotorOutputConfigs INDEX_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive); // 設定順時針為正向

    // --- 電流限制 (Kraken X44) ---
    public static final CurrentLimitsConfigs INDEX_CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40.0)
        .withStatorCurrentLimitEnable(true);

    // --- 預設運作電壓 ---
    public static final Voltage INDEX_RUN_VOLTAGE = Volts.of(8.0);   // 正轉送球電壓
    public static final Voltage INDEX_REVERSE_VOLTAGE = Volts.of(-6.0); // 反轉吐球電壓
    }
    
    public static class IntakeConstants {
        public static final int LEFT_SPIN_ID = 10;
        public static final int RIGHT_SPIN_ID = 11;
        public static final int LEFT_RACK_ID = 12;
        public static final int RIGHT_RACK_ID = 13;

        public static final Slot0Configs RACK_GAINS = new Slot0Configs()
                .withKP(3.0)
                .withKD(0.5)
                .withKA(0.0)
                .withKV(0.3)
                .withKS(0.5);

        public static final Slot1Configs RACK_DIFF_GAINS = new Slot1Configs().withKP(0.5);

        public static final MotorOutputConfigs RACK_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final MotorOutputConfigs SPIN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final CurrentLimitsConfigs RACK_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(100).withSupplyCurrentLowerLimit(30);

        public static final CurrentLimitsConfigs SPIN_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(65).withSupplyCurrentLowerLimit(30);

        public static final MotionMagicConfigs RACK_MOTION_MAGIC = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(50))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(60));

        public static final Angle STOW_POS = Degrees.of(0);//收納位置
        public static final Angle DEPLOY_POS = Degrees.of(0);//完全展開位置
        public static final Voltage SPIN_VOLTAGE = Volts.of(8);//進料電壓
        public static final Voltage REVERSE_SPIN_VOLTAGE = Volts.of(6);//反轉電壓
        public static final Voltage UNJAM_SPIN_VOLTAGE = Volts.of(6);//防卡球電壓
        public static final Angle STOW_TOLERANCE = Degrees.of(3);//收納誤差
        public static final Angle DEPLOY_TOLERANCE = Degrees.of(3);//展開誤差
        public static final Angle DECOUPLE_DISTANCE = Degrees.of(0);//脫鉤角度(無)

        public static final AngularVelocity MIN_SWITCH_ROBOT_VELOCITY = RadiansPerSecond.of(0);//切換狀態時車速限制

        public static final Current RACK_STALL_CURRENT = Amps.of(20);//支架堵轉判定電流
        public static final AngularVelocity RACK_STALL_VEL = RadiansPerSecond.of(0.3);//支架堵轉判定速度
        public static final Voltage ZEROING_VOLTAGE = Volts.of(-2);//歸零校正電壓

        public static final Current SPIN_STALL_CURRENT = Amps.of(20);//滾輪卡彈判定電流
        public static final AngularVelocity SPIN_STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0.3);//滾輪卡彈判定轉速
    }    
    
    public static final class ShooterConstants {
        public static final int FLYWHEEL_LEFT_ID = 20;
        public static final int FLYWHEEL_RIGHT_ID = 21;
        public static final int FEEDER_ID = 22;
        public static final int PIVOT_LEFT_ID = 23;
        public static final int PIVOT_RIGHT_ID = 24;
    
        public static final Slot0Configs FLYWHEEL_GAINS = new Slot0Configs()
            .withKP(0.1).withKI(0.0).withKD(0.0).withKV(0.12);
    
        public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);
    
        public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40.0).withStatorCurrentLimitEnable(true);
    
        public static final AngularVelocity FLYWHEEL_TOLERANCE = RotationsPerSecond.of(3);
        public static final AngularVelocity FLYWHEEL_STALL_VEL = RotationsPerSecond.of(2);
        public static final Current FLYWHEEL_STALL_CURRENT = Amps.of(35);
    
        public static final Slot0Configs PIVOT_GAINS = new Slot0Configs()
            .withKP(1.0).withKI(0.0).withKD(0.0);
    
        public static final Slot1Configs PIVOT_DIFF_GAINS = new Slot1Configs()
            .withKP(0.5).withKI(0.0).withKD(0.0);
    
        public static final MotorOutputConfigs PIVOT_OUTPUT_CONFIGS = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
    
        public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30.0).withStatorCurrentLimitEnable(true);
    
        public static final MotionMagicConfigs PIVOT_MOTION_MAGIC = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(50)
            .withMotionMagicAcceleration(100);
    
        public static final Angle PIVOT_STOW_POS = Rotations.of(0);
        public static final Angle PIVOT_TOLERANCE = Rotations.of(0.02);
        public static final AngularVelocity PIVOT_STALL_VEL = RadiansPerSecond.of(0.1);
        public static final Current PIVOT_STALL_CURRENT = Amps.of(25);
    
        public static final Voltage PIVOT_ZEROING_VOLTAGE = Volts.of(-2);
    
        public static final MotorOutputConfigs FEEDER_OUTPUT_CONFIGS = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    
        public static final CurrentLimitsConfigs FEEDER_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40.0)
            .withStatorCurrentLimitEnable(true);
    
        public static final Voltage FEEDER_SHOOT_VOLTAGE = Volts.of(8);
    }

    public static final class VisionConstants {
    // 3 顆 OV9281 相機名稱（需與 PhotonVision 介面中的名稱相符）
    public static final String[] CAMERA_NAMES = { "camera_front", "camera_left", "camera_right" };

    // 相機在機器人座標系下的位置 (X前, Y左, Z上；單位公尺) 與角度 (roll, pitch, yaw；單位「弧度」)
    // ⚠ 數值是示意，請用捲尺量實際安裝位置
    public static final Transform3d[] CAMERA_TRANSFORMS = {
        new Transform3d(new Translation3d(0.3, 0.0, 0.5),  new Rotation3d(0, 0, 0)),                    // 前
        new Transform3d(new Translation3d(0.0, 0.3, 0.5),  new Rotation3d(0, 0, Math.toRadians(90))),   // 左
        new Transform3d(new Translation3d(0.0, -0.3, 0.5), new Rotation3d(0, 0, Math.toRadians(-90)))   // 右
    };

    // 場地 AprilTag 地圖（要跟 PhotonVision 網頁設定的 layout 一致：welded 或 AndyMark）
    public static final AprilTagFieldLayout APRIL_TAGS =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // 過濾閥值
    public static final double MAX_AMBIGUITY = 0.2;
    public static final Distance MAX_Z_HEIGHT = Meters.of(0.75);

    // 標準差基準
    public static final double[] LINEAR_STD_DEV_BASELINES = { 0.1, 0.1, 0.1 };
    public static final double ANGULAR_STD_DEV_BASELINE = 0.1;

    // 瞄準用 tag：只用面向己方聯盟區的那一面，依優先順序排列（10、26 與 HUB 中心對齊，優先使用）
    public static final List<Integer> RED_AIM_TAG_IDS  = List.of(10, 9);
    public static final List<Integer> BLUE_AIM_TAG_IDS = List.of(26, 25);

    // HUB 中心座標（公尺），之後做「用位置算角度」的瞄準可以直接用
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.61, 4.02);
    public static final Translation2d RED_HUB_CENTER  = new Translation2d(11.90, 4.02);

    // 相機資料超過這個秒數沒更新，就當作看不到目標
    public static final double MAX_RESULT_AGE_SECONDS = 0.3;
}
}