package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PlayAndLeave extends SequentialCommandGroup {
  public PlayAndLeave(Drivetrain s_Swerve) {
    // loads path given a file name and velocity + acceleration constraints
    PathPlannerTrajectory path = PathPlanner.loadPath("examplepath", new PathConstraints(2, 3));

    // Make new PPSwerveControllerCommand with the given constructor parameters
    PPSwerveControllerCommand swervePathCommand = 
    new PPSwerveControllerCommand(
      path,
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(1.0, 0, 0),
      new PIDController(1.0, 0, 0),
      new PIDController(1.0, 0, 0),
      s_Swerve::setModuleStates,
      true,
      s_Swerve);
    
    // Auton commands
    addCommands(
      // Reset odometry based on initial pose of path
      new InstantCommand(() -> s_Swerve.resetOdometry(path.getInitialPose())),
      new WaitCommand(1),
      // Run the actual path planner
      swervePathCommand,
      // 0.1 Seconds after path ends, feed 0 into motors
      new WaitCommand(0.1),
      new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true))
    );
  
  }

}