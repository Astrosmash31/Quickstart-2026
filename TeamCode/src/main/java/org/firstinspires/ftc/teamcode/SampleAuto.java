package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class SampleAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private PathState pathState;

    public enum PathState {
        DRIVE_FORWARD,
        TURN_LEFT,
        DRIVE_DOWN,
        RETURN_HOME,
        DONE
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.00000000000001, 6.94086826347305, Math.toRadians(90)));

        paths = new Paths(follower);
        pathState = PathState.DRIVE_FORWARD;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain driveForward;
        public PathChain turnLeft;
        public PathChain driveDown;
        public PathChain returnHome;

        public Paths(Follower follower) {
            driveForward = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.212, 5.882),
                            new Pose(56.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            turnLeft = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 36.000),
                            new Pose(11.015, 35.799)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            driveDown = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.015, 35.799),
                            new Pose(11.473, 6.787)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            returnHome = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.473, 6.787),
                            new Pose(55.956, 6.177)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_FORWARD:
                if (!follower.isBusy()) {
                    follower.followPath(paths.driveForward, true);
                    setPathState(PathState.TURN_LEFT);
                }
                break;

            case TURN_LEFT:
                if (!follower.isBusy()) {
                    follower.followPath(paths.turnLeft, true);
                    setPathState(PathState.DRIVE_DOWN);
                }
                break;

            case DRIVE_DOWN:
                if (!follower.isBusy()) {
                    follower.followPath(paths.driveDown, true);
                    setPathState(PathState.RETURN_HOME);
                }
                break;

            case RETURN_HOME:
                if (!follower.isBusy()) {
                    follower.followPath(paths.returnHome, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                panelsTelemetry.debug("Status", "Autonomous Complete");
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        panelsTelemetry.debug("State Change", "→ " + newState.name());
    }
}