package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FakePID extends CommandBase{
    double distance;
    Consumer<Double> motorConsumer;
    Supplier<Double> encoderSupplier;
    double speedMultiplier;
    double slow;
    double maxErrorSize;
    boolean reversed;

    /**
    * Turns the motor to a specified distance. De/Accelerates based on distance.
    *
    * @param distance The distance to go to.
    * @param motorConsumer A function that sets the motor speed with parameter (double speed).
    * @param encoderSupplier A function that supplies the encoder distance.
    * @param speedMultiplier a multiplier for speed from 0.0 to 1.0.
    * @param slow Higher = slow earlier but -deacceleration. Lower = slow later but +deacceleration.
    * @param maxErrorSize The size of acceptable error at the end before turning the motor off.
    * @param reversed Normally clockwise is positive. Otherwise, make this true.
    */
    public FakePID(
        double distance,
        Consumer<Double> motorConsumer,
        Supplier<Double> encoderSupplier,
        double speedMultiplier,
        double slow,
        double maxErrorSize,
        boolean reversed,
        Subsystem... requirements) {
            this.distance = distance;
            this.motorConsumer = motorConsumer;
            this.encoderSupplier = encoderSupplier;
            this.speedMultiplier = speedMultiplier;
            this.slow = slow;
            this.maxErrorSize = maxErrorSize;
            this.reversed = reversed;

            addRequirements(requirements);
    }

    // You can use the above constructor as an example and put different constructors for the class here
    // Other constructors use the same intialize(), execute(), end(), and isFinished() declared below

    @Override
    public void initialize() {
        // Run one time when the command starts
    }

    @Override
    public void execute() {
        // Runs periodically

        // Spin clockwise
        if ((encoderSupplier.get() - distance) < 0) {
            if (!reversed) { motorConsumer.accept(speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, 0.0))); }
            else { motorConsumer.accept(-speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, 0.0))); }
        }
        // Spin counter-clockwise
        else if ((encoderSupplier.get() - distance) > 0) {
            if (!reversed) { motorConsumer.accept(-speedMultiplier*Math.min(1.0, Math.max((encoderSupplier.get() - distance)/slow, 0.0))); }
            else { motorConsumer.accept(speedMultiplier*Math.min(1.0, Math.max((encoderSupplier.get() - distance)/slow, 0.0))); }
        }

        // Display SmartDashboard
        SmartDashboard.putNumber(("Goal:"), distance);
        SmartDashboard.putNumber("Distance:", encoderSupplier.get());
        SmartDashboard.putNumber(("Speed:"), speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, -1.0))*100);
    }

    @Override
    public void end(boolean interrupted) {
        // Run one time after isFinished() returns true
        motorConsumer.accept(0.0);
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should finish
        return (Math.abs(encoderSupplier.get() - distance) <= maxErrorSize);
    }

}