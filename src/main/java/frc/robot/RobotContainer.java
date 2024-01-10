// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.PlayAndLeave;
import frc.robot.autos.TestPathPlanner;
//import frc.robot.commands.UNUSED_PlayMiddle;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // Subsystems and Commands
  private final Drivetrain s_Swerve = new Drivetrain();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  //private final WristSubsystem m_WristSubsystem = new WristSubsystem();

  // Auton chooser
  public SendableChooser<Command> m_Chooser = new SendableChooser<>();
  // Auto commands
  public final Command c_TestPathPlanner = new TestPathPlanner(s_Swerve);
  public final Command c_PlayAndLeave = new PlayAndLeave(s_Swerve, m_ArmSubsystem, m_ElevatorSubsystem, m_ClawSubsystem);

  // Xbox Controllers
  private final CommandXboxController m_DriverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_CoDriverController =
    new CommandXboxController(OperatorConstants.kCoDriverControllerPort);
  private final CommandXboxController m_3DriverController =
    new CommandXboxController(OperatorConstants.k3DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      /*
      new TeleopSwerve(
        s_Swerve,
          () -> -m_DriverController.getLeftY(), // move up/down
          () -> -m_DriverController.getLeftX(), // move left/right
          () -> -m_DriverController.getRightX(), // rotate
          () -> m_DriverController.back().getAsBoolean(), // robot centric button
          () -> m_DriverController.rightBumper().getAsBoolean() // slow button
        ));
      */
      new TeleopSwerve(
        s_Swerve,
          () -> -m_3DriverController.getLeftY(), // move up/down
          () -> -m_3DriverController.getLeftX(), // move left/right
          () -> -m_3DriverController.getRightX(), // rotate
          () -> m_3DriverController.back().getAsBoolean(), // robot centric button
          () -> m_3DriverController.rightBumper().getAsBoolean() // slow button
        ));
    m_ArmSubsystem.setDefaultCommand(
      Commands.run(
      () ->
      m_ArmSubsystem.ArmTurnMethod(m_CoDriverController.getLeftY()), m_ArmSubsystem));
    /*
    m_WristSubsystem.setDefaultCommand(
      Commands.run(
      () ->
      m_WristSubsystem.WristTurn(m_CoDriverController.getRightX()), m_WristSubsystem));
    */
    
    m_ElevatorSubsystem.setDefaultCommand(
      Commands.run(
      () ->
      m_ElevatorSubsystem.ElevatorTurnMethod(m_CoDriverController.getRightY(), m_CoDriverController.b().getAsBoolean()), m_ElevatorSubsystem));
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // ArmSubsystem controls
    m_CoDriverController.a().onTrue(m_ArmSubsystem.ArmTurnToAngle(0.0));
    m_CoDriverController.x().onTrue(m_ArmSubsystem.ArmTurnToAngle(-45.0));

    m_CoDriverController.y().onTrue(m_ElevatorSubsystem.ElevatorTurnToDistance(5.0));
    m_CoDriverController.b().onTrue(m_ElevatorSubsystem.ElevatorTurnToDistance(2.0));

    // ClawSubsystem controls
    m_CoDriverController.leftBumper().whileTrue(m_ClawSubsystem.SetClawSpeedCommand(-1.0));
    m_CoDriverController.rightBumper().whileTrue(m_ClawSubsystem.SetClawSpeedCommand(1.0));

    // Swerve controls
    m_DriverController.start().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // Driver 3 controls
    m_3DriverController.leftBumper().whileTrue(m_ClawSubsystem.SetClawSpeedCommand(-1.0));
    m_3DriverController.rightBumper().whileTrue(m_ClawSubsystem.SetClawSpeedCommand(1.0));
  }

  // Return auton command to main
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

}
