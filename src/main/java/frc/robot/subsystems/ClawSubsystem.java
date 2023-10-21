// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

  private final CANSparkMax m_Claw = new CANSparkMax(Constants.ClawConstants.kMotorID, MotorType.kBrushless);
  private final RelativeEncoder ClawEncoder = m_Claw.getEncoder();
  
  public ClawSubsystem() {
    ClawEncoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  public void SetClawSpeed(double speed) {
    m_Claw.set(speed);
    SmartDashboard.putNumber("Claw Speed", speed);
  }
  
  public Command SetClawSpeedCommand(double speed) {
    return new StartEndCommand(() -> this.SetClawSpeed(speed), () -> this.SetClawSpeed(0), this);
  }

}
