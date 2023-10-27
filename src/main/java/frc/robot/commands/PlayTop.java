// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;


public class PlayTop extends SequentialCommandGroup {
  public PlayTop(ElevatorSubsystem m_Elevator, ArmSubsystem m_Arm, ClawSubsystem m_Claw) {
    addCommands(
      new ParallelCommandGroup(
        // Rotate Arm
        Commands.run(
            () -> {
                m_Arm.ArmTurnToAngle(Constants.CommandGroupConstants.TopArmAngle);
            },
            m_Arm).withTimeout(2.3),
        // Extend Elevator
        Commands.runOnce(
            () -> {
                m_Elevator.ElevatorTurnToDistance(Constants.CommandGroupConstants.TopElevatorPosition);
            },
            m_Elevator).withTimeout(2.3)
    ),
      // Rotate Arm
      Commands.run(
            () -> {
                m_Arm.ArmTurnToAngle(Constants.CommandGroupConstants.TopArmAngle - Constants.CommandGroupConstants.DownAngle);
            },
            m_Arm).withTimeout(0.5),
      // Release Cone
      m_Claw.SetClawSpeedCommand(0.5).withTimeout(0.25),
      // Wait Buffer
      new WaitCommand(0.5),
      // Reset Arm
      Commands.run(
            () -> {
                m_Arm.ArmTurnToAngle(0);
            },
            m_Arm).withTimeout(1.0),
      // Reset Elevator
      Commands.runOnce(
        () -> {
            m_Elevator.ElevatorTurnToDistance(1.0);
        },
        m_Elevator).withTimeout(1.5),
      // Wait Buffer
      new WaitCommand(1.5)
      );
  }
}