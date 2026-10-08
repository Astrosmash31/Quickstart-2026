package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Aims a servo turret at a fixed field point using the robot pose from Pedro Pathing.
 * All angles are in radians. Turret angle 0 = pointing straight out the front of the robot.
 */
public class turretAim {

    // ---- Tune these ----
    public static double CENTER_POS = 0.5;     // servo position when turret faces forward
    // Servo position change per radian of TURRET rotation. This assumes direct drive and that
    // 0..1 spans the full 270 deg (configure the servo as "Full Range Servo"). If geared,
    // divide by the (servo turns / turret turns) ratio. Calibrate on the real robot.
    public static double POS_PER_RAD = 1.0 / Math.toRadians(270.0);
    public static double MIN_ANGLE = Math.toRadians(-135); // physical limits of the turret
    public static double MAX_ANGLE = Math.toRadians(135);

    private final Servo servoA;
    private final Servo servoB; // second servo of the geared pair; set its direction in the constructor
    private double targetX;
    private double targetY;

    public enum Alliance { RED, BLUE }

    // ---- HIVE geometry (Pedro coords, inches) ----
    // Measured HIVE centers. The HIVEs sit side by side along X; each beam runs along Y,
    // with a CELL opening at each end.
    public static double BLUE_HIVE_X = 84.135;
    public static double BLUE_HIVE_Y = 70.754;
    public static double RED_HIVE_X = 57.931;
    public static double RED_HIVE_Y = 71.168;
    public static double OPENING_OFFSET = 20.7; // ESTIMATE along the beam (Y). ~21.5 in half-span, shortened by the 15 deg tilt. Verify on the field.

    // Device names, must match the Robot Configuration on the Driver Hub exactly
    public static final String SERVO_1_NAME = "turretServo1";
    public static final String SERVO_2_NAME = "turretServo2";

    /** Convenience constructor: pulls both servos from the hardwareMap by name. */
    public turretAim(HardwareMap hardwareMap, double targetX, double targetY) {
        this(hardwareMap.get(Servo.class, SERVO_1_NAME),
                hardwareMap.get(Servo.class, SERVO_2_NAME),
                targetX, targetY);
    }

    public turretAim(Servo servoA, Servo servoB, double targetX, double targetY) {
        this.servoA = servoA;
        this.servoB = servoB;
        this.targetX = targetX;
        this.targetY = targetY;
        // If the second servo faces the opposite way, uncomment:
        // this.servoB.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Aim at your own alliance's HIVE.
     * @param upCellOnHighY true if the CELL currently pointing up is on the +Y end of the beam
     */
    public void setTargetHive(Alliance alliance, boolean upCellOnHighY) {
        double cx = (alliance == Alliance.RED) ? RED_HIVE_X : BLUE_HIVE_X;
        double cy = (alliance == Alliance.RED) ? RED_HIVE_Y : BLUE_HIVE_Y;
        setTarget(cx, cy + (upCellOnHighY ? OPENING_OFFSET : -OPENING_OFFSET));
    }

    /** Change the point being aimed at (e.g. switch between hives). */
    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /** Field-frame angle from the robot to the target. */
    public double getFieldAngleToTarget(double x, double y) {
        return Math.atan2(targetY - y, targetX - x);
    }

    /** Angle the turret needs relative to the robot's front, wrapped to +/- PI (not yet clamped). */
    public double getDesiredTurretAngle(double x, double y, double heading) {
        return AngleUnit.normalizeRadians(getFieldAngleToTarget(x, y) - heading);
    }

    /** True if the desired angle is inside the turret's physical range. */
    public boolean isTargetInRange(double x, double y, double heading) {
        double a = getDesiredTurretAngle(x, y, heading);
        return a >= MIN_ANGLE && a <= MAX_ANGLE;
    }

    /** Clamp an angle to the turret's physical limits. */
    public double clampAngle(double angle) {
        return Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    /** Convert a turret angle to a servo position (0..1). */
    public double angleToServoPosition(double angle) {
        return Range.clip(CENTER_POS + angle * POS_PER_RAD, 0.0, 1.0);
    }

    /** Send a turret angle to both servos. */
    public void setTurretAngle(double angle) {
        double pos = angleToServoPosition(clampAngle(angle));
        servoA.setPosition(pos);
        servoB.setPosition(pos);
    }

    /** Point the turret straight ahead. */
    public void center() {
        setTurretAngle(0);
    }

    /** One call to aim: compute the angle from the pose and command the servos. */
    public void aim(double x, double y, double heading) {
        setTurretAngle(getDesiredTurretAngle(x, y, heading));
    }
}