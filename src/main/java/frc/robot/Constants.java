package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static final CANBus CAN_FD_BUS = new CANBus("base1");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

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

        public static final Angle STOW_POS = Degrees.of(0);
        public static final Angle DEPLOY_POS = Degrees.of(0);
        public static final Voltage SPIN_VOLTAGE = Volts.of(0);
        public static final Voltage REVERSE_SPIN_VOLTAGE = Volts.of(0);
        public static final Voltage UNJAM_SPIN_VOLTAGE = Volts.of(0);
        public static final Angle STOW_TOLERANCE = Degrees.of(0);
        public static final Angle DEPLOY_TOLERANCE = Degrees.of(0);
        public static final Angle DECOUPLE_DISTANCE = Degrees.of(0);

        public static final AngularVelocity MIN_SWITCH_ROBOT_VELOCITY = RadiansPerSecond.of(0);

        public static final Current RACK_STALL_CURRENT = Amps.of(40);
        public static final AngularVelocity RACK_STALL_VEL = RadiansPerSecond.of(0);
        public static final Voltage ZEROING_VOLTAGE = Volts.of(0);

        public static final Current SPIN_STALL_CURRENT = Amps.of(20);
        public static final AngularVelocity SPIN_STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
    }    
}
