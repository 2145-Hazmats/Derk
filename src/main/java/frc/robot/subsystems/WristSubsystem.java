// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX m_Wrist = new WPI_TalonSRX(Constants.WristConstants.kMotorID);
  private final Encoder m_WristEncoder = new Encoder(0, 1,true); 

  public WristSubsystem() {
    // Change distance to angle instead of pulses
    m_WristEncoder.setDistancePerPulse(Constants.WristConstants.PulsesToAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("WristDistance", m_WristEncoder.getDistance());
    SmartDashboard.putBoolean("WristDirection", m_WristEncoder.getDirection());
  }

  // Turns wrist motor with parameter speed
  public void WristTurn(double speed) {
    // Set limit at 0 degrees
    if (m_WristEncoder.getDistance() < 0 && speed > 0) {
      m_Wrist.stopMotor();
    }
    // Set limit at 360 degrees
    else if (m_WristEncoder.getDistance() > 360 && speed < 0) {
      m_Wrist.stopMotor();
    }
    // Spin based on user input. Up = clockwise
    // Because this is negative the < and > are swapped
    else {
      m_Wrist.set(-speed*Constants.WristConstants.ManualSpeed);
    }
  }

  // Turns wrist motor to an angle parameter then stops
  public void WristToAngle(double angle) {
    // Stop if angle is close enough
    if ((m_WristEncoder.getDistance() - Math.abs(angle)) <= 5 && (Math.abs(angle) - m_WristEncoder.getDistance() <= 5)) {
      m_Wrist.stopMotor();
    }
    // If it needs to go counter-clockwise... go counter-clockwise
    else if ((m_WristEncoder.getDistance() - angle) > 0) {
      m_Wrist.set(-Constants.WristConstants.TurnToSpeed);
    }
    // If it needs to go clockwise... go clockwise
    else if ((m_WristEncoder.getDistance() - angle) < 0) {
      m_Wrist.set(Constants.WristConstants.TurnToSpeed);
    }
  }

}