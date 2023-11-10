package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class HazmatLib extends CommandBase{
    public HazmatLib() {}

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
    
    //public static <T extends com.revrobotics.CANSparkMax> void TurnToDistanceFakePID( - DELETE if already works
    public static void MethodFakePID(
            double distance,
            //T motorT, - DELETE if already works
            Consumer<Double> motorConsumer,
            Supplier<Double> encoderSupplier,
            double speedMultiplier,
            double slow,
            double maxErrorSize,
            boolean reversed) {
        // Stop if angle is close enough
        if (Math.abs(encoderSupplier.get() - distance) <= maxErrorSize) {
            //motorT.stopMotor(); - DELETE if already works
            motorConsumer.accept(0.0);
        }
        /*
        // Spin clockwise
        else if (((encoderSupplier.get() - distance) < 0 && !reversed) || ((encoderSupplier.get() - distance) > 0 && reversed)) {
            motorConsumer.accept(speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, 0.0)));
        }
        // Spin Counter-clockwise
        else if (((encoderSupplier.get() - distance) < 0 && reversed) || ((encoderSupplier.get() - distance) > 0 && !reversed)) {
            motorConsumer.accept(-speedMultiplier*Math.min(1.0, Math.max((encoderSupplier.get() - distance)/slow, 0.0)));
        }
        */
        // Spin clockwise
        else if ((encoderSupplier.get() - distance) < 0) {
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
        SmartDashboard.putNumber(("Speed:"), speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, -1.0))*100);
    }


    
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
    * @param requirements The subsystems to require.
    */
    
    //public static <T extends com.revrobotics.CANSparkMax> void TurnToDistanceFakePID( - DELETE if already works
    public Command CommandFakePID(
            double distance,
            //T motorT, - DELETE if already works
            Consumer<Double> motorConsumer,
            Supplier<Double> encoderSupplier,
            double speedMultiplier,
            double slow,
            double maxErrorSize,
            boolean reversed,
            Subsystem... requirements) {
        addRequirements(requirements);

        // Stop if angle is close enough
        if (Math.abs(encoderSupplier.get() - distance) <= maxErrorSize) {
            //motorT.stopMotor(); - DELETE if already works
            motorConsumer.accept(0.0);
        }
        // Spin clockwise
        else if ((encoderSupplier.get() - distance) < 0) {
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
        SmartDashboard.putNumber(("Speed:"), speedMultiplier*Math.min(1.0, Math.max((distance - encoderSupplier.get())/slow, -1.0))*100);
        // Returns something
        return null;
    }

}