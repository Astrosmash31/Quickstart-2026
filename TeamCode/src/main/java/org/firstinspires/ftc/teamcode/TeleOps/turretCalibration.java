package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.turretAim;

/**
 * Calibrates the turret.
 *
 * RAW mode (default): move the servos by hand-picked positions and mark known angles.
 *   D-pad up/down      position +/- 0.01
 *   Left/right bumper  position -/+ 0.001 (fine)
 *   Y                  go to 0.5
 *   A                  mark "turret facing straight forward"            -> CENTER_POS
 *   B                  mark "turret rotated +90 deg (to the robot's left)" -> POS_PER_RAD
 *   D-pad left         mark the physical limit on the LEFT  (+ angle)   -> MAX_ANGLE
 *   D-pad right        mark the physical limit on the RIGHT (- angle)   -> MIN_ANGLE
 *
 * ANGLE mode (press X to toggle): command turretAim directly, using the values just calibrated.
 *   D-pad up/down      angle +/- 5 deg
 *   Left/right bumper  angle -/+ 1 deg
 *   Y                  0 deg
 *
 * Marked values are applied live to turretAim's static fields so ANGLE mode tests them right away.
 * To keep them, copy the "Copy into turretAim" lines into turretAim.java.
 *
 * Move slowly near the ends of travel; don't drive the turret into a hard stop.
 */
@TeleOp(name = "Turret Calibration", group = "Utility")
public class turretCalibration extends OpMode {

    // Set true if the second servo faces the opposite way (or put REVERSE in turretAim's constructor instead)
    private static final boolean REVERSE_SERVO_2 = false;

    private Servo servo1;
    private Servo servo2;
    private turretAim turret;

    private boolean angleMode = false;
    private double rawPos = 0.5;
    private double angleDeg = 0.0;

    private double centerPos = Double.NaN;
    private double plus90Pos = Double.NaN;
    private double leftLimitPos = Double.NaN;
    private double rightLimitPos = Double.NaN;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, turretAim.SERVO_1_NAME);
        servo2 = hardwareMap.get(Servo.class, turretAim.SERVO_2_NAME);
        if (REVERSE_SERVO_2) servo2.setDirection(Servo.Direction.REVERSE);
        turret = new turretAim(hardwareMap, 0, 0);
    }

    @Override
    public void start() {
        setRaw(rawPos);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            angleMode = !angleMode;
            if (!angleMode) setRaw(rawPos);
        }

        if (angleMode) {
            if (gamepad1.dpadUpWasPressed()) angleDeg += 5;
            if (gamepad1.dpadDownWasPressed()) angleDeg -= 5;
            if (gamepad1.rightBumperWasPressed()) angleDeg += 1;
            if (gamepad1.leftBumperWasPressed()) angleDeg -= 1;
            if (gamepad1.yWasPressed()) angleDeg = 0;
            turret.setTurretAngle(Math.toRadians(angleDeg));
        } else {
            if (gamepad1.dpadUpWasPressed()) rawPos += 0.01;
            if (gamepad1.dpadDownWasPressed()) rawPos -= 0.01;
            if (gamepad1.rightBumperWasPressed()) rawPos += 0.001;
            if (gamepad1.leftBumperWasPressed()) rawPos -= 0.001;
            if (gamepad1.yWasPressed()) rawPos = 0.5;
            rawPos = Range.clip(rawPos, 0.0, 1.0);
            setRaw(rawPos);

            if (gamepad1.aWasPressed()) centerPos = rawPos;
            if (gamepad1.bWasPressed()) plus90Pos = rawPos;
            if (gamepad1.dpadLeftWasPressed()) leftLimitPos = rawPos;
            if (gamepad1.dpadRightWasPressed()) rightLimitPos = rawPos;
            applyCalibration();
        }

        telemetry.addData("Mode (X toggles)", angleMode ? "ANGLE" : "RAW");
        if (angleMode) {
            telemetry.addData("Commanded angle (deg)", angleDeg);
            telemetry.addData("Servo position sent",
                    turret.angleToServoPosition(turret.clampAngle(Math.toRadians(angleDeg))));
        } else {
            telemetry.addData("Raw servo position", rawPos);
        }
        telemetry.addLine();
        telemetry.addData("Marked center", fmt(centerPos));
        telemetry.addData("Marked +90 deg", fmt(plus90Pos));
        telemetry.addData("Marked left limit", fmt(leftLimitPos));
        telemetry.addData("Marked right limit", fmt(rightLimitPos));
        telemetry.addLine();
        telemetry.addLine("Copy into turretAim:");
        telemetry.addData("CENTER_POS", fmt(turretAim.CENTER_POS));
        telemetry.addData("POS_PER_RAD", fmt(turretAim.POS_PER_RAD));
        telemetry.addData("MIN_ANGLE (deg)", fmt(Math.toDegrees(turretAim.MIN_ANGLE)));
        telemetry.addData("MAX_ANGLE (deg)", fmt(Math.toDegrees(turretAim.MAX_ANGLE)));
    }

    private void setRaw(double pos) {
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }

    /** Turns the marked positions into turretAim's constants and applies them live. */
    private void applyCalibration() {
        if (!Double.isNaN(centerPos)) {
            turretAim.CENTER_POS = centerPos;
        }
        if (!Double.isNaN(centerPos) && !Double.isNaN(plus90Pos) && plus90Pos != centerPos) {
            turretAim.POS_PER_RAD = (plus90Pos - centerPos) / (Math.PI / 2.0);
        }
        if (!Double.isNaN(centerPos) && !Double.isNaN(plus90Pos) && plus90Pos != centerPos) {
            if (!Double.isNaN(leftLimitPos)) {
                turretAim.MAX_ANGLE = (leftLimitPos - turretAim.CENTER_POS) / turretAim.POS_PER_RAD;
            }
            if (!Double.isNaN(rightLimitPos)) {
                turretAim.MIN_ANGLE = (rightLimitPos - turretAim.CENTER_POS) / turretAim.POS_PER_RAD;
            }
        }
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "not set" : String.format("%.4f", v);
    }
}