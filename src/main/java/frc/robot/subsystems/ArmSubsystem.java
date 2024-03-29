// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FakePID;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_Arm = new CANSparkMax(Constants.ArmConstants.kMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_ArmEncoder = m_Arm.getEncoder();

  public ArmSubsystem() {
    // reset encoder to 0 when code is deployed. Not when the robot is enabled/disabled btw
    m_ArmEncoder.setPosition(0.0);
    // Set conversion factor
    m_ArmEncoder.setPositionConversionFactor(Constants.ArmConstants.EncoderToAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(("Arm Angle"), m_ArmEncoder.getPosition());
  }

  // Runs the arm manually. Positive speed is clockwise
  public void ArmTurnMethod(double speed) {
    m_Arm.set(speed*Constants.ArmConstants.ManualSpeed);
  }

  public Command ArmTurnToAngle(double angle) {
    return new FakePID(
      angle,
      m_Arm::set,
      m_ArmEncoder::getPosition,
      Constants.ArmConstants.TurnToSpeed,
      Constants.ArmConstants.MaxErrorSize,
      Constants.ArmConstants.SlowMultiplier,
      false,
      this);
  }

}