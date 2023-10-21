package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Drivetrain s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  // SlewRateLimiters limit the change in speed for motors. Higher = Floatier. Lower = Better for equipment.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

  public TeleopSwerve(
    Drivetrain s_Swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup,
    BooleanSupplier slowSpeedSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
  }

  @Override
  public void execute() {

    double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.2 : 1.0;

    /* Get Values, Deadband*/
    double translationVal =
      translationLimiter.calculate(
        speedMultiplier *
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
      strafeLimiter.calculate(
        speedMultiplier *
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
      rotationLimiter.calculate(
        speedMultiplier *
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));


    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity,
      !robotCentricSup.getAsBoolean(),
      true);
  
  }

}