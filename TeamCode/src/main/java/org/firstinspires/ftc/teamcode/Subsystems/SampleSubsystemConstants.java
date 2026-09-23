package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SampleSubsystemConstants {
    //Add subsystem constants here.  Use this to avoid magic numbers
    public static final int MOTOR_NAME_THRESHOLD = 5;
    public static final DcMotor.ZeroPowerBehavior MOTOR_NAME_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotorSimple.Direction MOTOR_NAME_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final double MOTOR_NAME_VELOCITY_P = 1.0;
    public static final double MOTOR_NAME_VELOCITY_I = 0.0;
    public static final double MOTOR_NAME_VELOCITY_D = 0.0;
    public static final double MOTOR_NAME_VELOCITY_F = 14.5;
    public static final int MOTOR_NAME_POSITION_TOLERANCE = 5;
    public static final double MOTOR_NAME_VELOCITY_TICKS_PER_S = 2200;
    public static final int MOTOR_NAME_A_POSITION = 500;
    public static final int MOTOR_NAME_B_POSITION = 1000;
    public static double MOTOR_NAME_POSITION_P = 5.0;
}
