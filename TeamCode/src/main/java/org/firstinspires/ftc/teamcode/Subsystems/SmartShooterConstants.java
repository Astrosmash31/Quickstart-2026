package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.LookupTable.Pair;

import java.util.List;

public class SmartShooterConstants {
    // Note - any changes to these constants will require recalibration!
    public static final String MOTOR_NAME = "shooter";
    public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final double VELOCITY_P = 1.0;
    public static final double VELOCITY_I = 0.0;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_F = 14.5;
    public static double POSITION_P = 5.0;
    public static final List<Pair> LOOKUP_TABLE = List.of(
        // Create list of distance / velocity pairs by calibrating your shooter
        // Be sure to measure the distance the ball travels before landing at the height
        // of the goal (plus a little margin)!
        // Notes:
        // - we want the distance to be the "input" and velocity to be the "output"; this
        // will allow us to as what velocity we should use based on an input distance request.
        // - Distances should be in the same units used for measuring the distance between
        // the robot and the goal
        // - Velocities should be in encoder ticks per second
        new Pair(0, 0),
        new Pair(1, 10),
        new Pair(2, 20),
        new Pair(5, 100)
    );
}
