// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax m_Elevator = new CANSparkMax(Constants.ElevatorConstants.kMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_ElevatorEncoder = m_Elevator.getEncoder();
  
  public ElevatorSubsystem() {
    // reset encoder to 0 when code is deployed. Not when the robot is enabled/disabled btw
    m_ElevatorEncoder.setPosition(0.0);
    // Set conversion factor
    m_ElevatorEncoder.setPositionConversionFactor(Constants.ElevatorConstants.EncoderToAngle);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber(("Elevator Position"), m_ElevatorEncoder.getPosition());
  }

  // Moves the elevator manually. Positive speed is forward
  public void ElevatorTurnMethod(double speed) {
    m_Elevator.set(-speed*Constants.ElevatorConstants.ManualSpeed);
  }

  // Moves the elevator manually. Positive speed is forward. Slow mode
  public void ElevatorTurnMethod(double speed, boolean slow) {
    if (slow == true) {
      m_Elevator.set(-(speed/2)*Constants.ElevatorConstants.ManualSpeed);
    }
    else {
      m_Elevator.set(-speed*Constants.ElevatorConstants.ManualSpeed);
    }
  }

  /*
  // Move the elevator to a specified distance. Slows down based on the current arm distance
  public void ElevatorTurnToDistance(double distance) {
    // Stop if angle is close enough
    if (Math.abs(m_ElevatorEncoder.getPosition() - distance) <= Constants.ElevatorConstants.DegreeOfError) {
        m_Elevator.stopMotor();
    }
    // Move some direction
    else if ((m_ElevatorEncoder.getPosition() - distance) > 0) {
        m_Elevator.set(-Constants.ElevatorConstants.TurnToSpeed*
      Math.min(1.0, Math.max((m_ElevatorEncoder.getPosition() - distance)/Constants.ElevatorConstants.SlowMultiplier, 0.0)));
    }
    // Move the other direction
    else if ((m_ElevatorEncoder.getPosition() - distance) < 0) {
        m_Elevator.set(Constants.ElevatorConstants.TurnToSpeed*
      Math.min(1.0, Math.max((distance - m_ElevatorEncoder.getPosition())/Constants.ElevatorConstants.SlowMultiplier, 0.0)));
    }
    // Display SmartDashboard
    SmartDashboard.putNumber(("Goal:"), distance);
    SmartDashboard.putNumber(("Speed:"), Constants.ElevatorConstants.TurnToSpeed*Math.min(1.0, Math.max((distance - m_ElevatorEncoder.getPosition())/Constants.ElevatorConstants.SlowMultiplier, -1.0))*100);
  }
  */

}
