// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
public void robotPeriodic() {
    // 執行指令排程器 (原本就在的)
    CommandScheduler.getInstance().run();

    // ==========================================
    // 傳送資料到 NetworkTables (Elastic 讀得到)
    // ==========================================
    
    // 1. 取得目前機器人電池電壓
    double voltage = edu.wpi.first.wpilibj.RobotController.getInputVoltage();
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Battery Voltage", voltage);

    // 2. 取得 FRC 官方場地提供的比賽剩餘時間 (自動倒數計時)
    // 警告：在練習模式或未連上場地時，這數值通常會是 -1.0，但比賽時會自動變成 15秒/150秒 倒數
    double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Match Countdown", matchTime);
}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
