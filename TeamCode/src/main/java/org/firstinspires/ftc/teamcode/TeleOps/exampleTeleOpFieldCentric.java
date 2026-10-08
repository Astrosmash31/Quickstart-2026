package org.firstinspires.ftc.teamcode.TeleOps;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Subsystems.hiveLimelight;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.turretAim; // NEW
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

@TeleOp(name = "RoboCentric TeleOp")
public class exampleTeleOpFieldCentric extends OpMode {

    // Turn on after you've verified which tags the camera sees (see hiveLimelight.VISIBLE_CELL_IS_LOW)
    private static final boolean USE_CAMERA_UP_CELL = false;

    private Follower follower;
    private hiveLimelight hiveCam;
    // NEW: turret state
    private turretAim turret;
    private turretAim.Alliance alliance = turretAim.Alliance.BLUE; // change in init with X (blue) / B (red)
    private boolean upCellOnHighY = false; // which end of our HIVE beam is pointing up; toggle with right bumper

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        turret.setTargetHive(alliance, upCellOnHighY);
        hiveCam.setAlliance(alliance); hiveCam.start();
    }
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        turret = new turretAim(hardwareMap, 0, 0); // NEW: target is set in start()
        turret.center();
        hiveCam = new hiveLimelight(hardwareMap, alliance);

    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) alliance = turretAim.Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = turretAim.Alliance.RED;
        telemetry.addData("Alliance (X = Blue, B = Red)", alliance);
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        hiveCam.update();
        if (USE_CAMERA_UP_CELL && hiveCam.getUpCell() != hiveLimelight.UpCell.UNKNOWN) {
            upCellOnHighY = (hiveCam.getUpCell() == hiveLimelight.UpCell.HIGH_Y_END);
            turret.setTargetHive(alliance, upCellOnHighY);
        }

        double dist = hiveCam.getDistanceInches();
        follower.manual(powers);
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object

        // NEW: flip which HIVE CELL we're aiming at, then aim the turret
        if (gamepad1.rightBumperWasPressed()) {
            upCellOnHighY = !upCellOnHighY;
            turret.setTargetHive(alliance, upCellOnHighY);
        }
        turret.aim(robotPose.x(), robotPose.y(), robotPose.heading());

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        // NEW
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Up CELL on +Y end", upCellOnHighY);
        telemetry.addData("Target in turret range", turret.isTargetInRange(robotPose.x(), robotPose.y(), robotPose.heading()));
        telemetry.addData("Cam up cell", hiveCam.getUpCell());
        telemetry.addData("Cam tags", hiveCam.getVisibleIdsString());
        telemetry.addData("Cam dist (in)", dist);
    }

    @Override
    public void stop() {
        hiveCam.stop();
    }
}