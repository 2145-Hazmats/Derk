// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {
  // Operator Constants
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  // Arm Constants
  public static class ArmConstants {
    public static final int kMotorID = 22;
    public static final double EncoderToAngle = 180/124.6; // Encoder multiplier

    public static final double ManualSpeed = 0.6;
    // TurnToAngle constants
    public static final double TurnToSpeed = 0.50; // initial speed of TurnToAngle
    public static final double DegreeOfError = 0.5; // when TurnToAngle should stop the motor
    public static final double SlowMultiplier = 20.0; // higher = smoother but slower
  }

  // Claw Constants
  public static class ClawConstants{
    public static final int kMotorID = 23;
  }

  // Elevator Constants
  public static class ElevatorConstants{
    public static final int kMotorID = 21;
    public static final double EncoderToAngle = 0.235619449; // Encoder multiplier

    public static final double ManualSpeed = 0.60;
    // TurnToAngle constants
    public static final double TurnToSpeed = 0.60; // initial speed of TurnToAngle
    public static final double DegreeOfError = 0.5; // when TurnToAngle should stop the motor
    public static final double SlowMultiplier = 20.0; // higher = smoother but slower
  }

  // Wrist Constants
  public static class WristConstants {
    public static final int kMotorID = 20;
    public static final double PulsesToAngle = 8.108108108; // For converting pulses to angle

    public static final double ManualSpeed = 0.20;
    public static final double TurnToSpeed = 0.20;
  }

  // CommandGroup Constants
  public static class CommandGroupConstants {
    public static final double TopArmAngle = -46.0;
    public static final double TopElevatorPosition = 6.7;
    public static final double MiddleArmAngle = -40.0;
    public static final double MiddleElevatorPosition = 1.0;
    public static final double DownAngle = 6.0;
  }

  // Swerve Constants
  public static final class Swerve {
    // Control Constants
    public static final double stickDeadband = 0.0; // Modified in Driver Station
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // Drivetrain Constants
    public static final double trackWidth = Units.inchesToMeters(18.75);
    public static final double wheelBase = Units.inchesToMeters(18.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (50.0/14.0) * (19.0/25.0) * (45.0/15.0); // 8.14:1
    public static final double angleGearRatio = (150.0 / 7.0);

    // Drivetrain kinematics
    public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      );

    // Swerve Voltage Compensation
    public static final double voltageComp = 12.0;

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    // Angle Motor PID Values
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    // Drive Motor PID Values
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    // Drive Motor Characterization Values
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    // Drive Motor Conversion Factors
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // Swerve Profiling Values
    public static final double maxSpeed = 3.6576; // meters per second
    public static final double maxAngularVelocity = 6;

    // Neutral Modes
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    // Motor Inverts
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    // Angle Encoder Invert
    public static final boolean canCoderInvert = false;

    // Module Specific Constants
    // Front Left Module - Module 0
    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(261.2109375);//261.298828125
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Front Right Module - Module 1
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(37.001953125);//35.947265625
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Left Module - Module 2
    public static final class Mod2 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(44.560546875);//44.6484375
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Right Module - Module 3
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(13.271484375);//13.623046875
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  
  }

}
