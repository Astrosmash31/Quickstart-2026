package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Sagan Auto", group = "Autonomous")
@Configurable // Panels
public class Sagan_Auto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public enum PathState {
        GRAB1,
        GRABANDSHOOT2,
        RETURNBACK1,
        DONE
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(91.79865269461078, 34.94086826347305, Math.toRadians(0)));
        pathState = PathState.GRAB1;
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Grab1;
        public PathChain Grabandshoot2;
        public PathChain ReturnBack1;

        public Paths(Follower follower) {
            Grab1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(90.528, 4.823),
                                    new Pose(91.799, 34.941)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(91.799, 34.941),
                                    new Pose(133.027, 35.163)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Grabandshoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.027, 35.163),
                                    new Pose(73.373, 55.508),
                                    new Pose(71.208, 72.453)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierCurve(
                                    new Pose(71.208, 72.453),
                                    new Pose(73.373, 55.508),
                                    new Pose(128.400, 58.498)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ReturnBack1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.400, 58.498),
                                    new Pose(70.962, 72.445)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case GRAB1:
                {
                    follower.followPath(paths.Grab1, true);
                    setPathState(PathState.GRABANDSHOOT2);
                }
                break;

            case GRABANDSHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Grabandshoot2, true);
                    setPathState(PathState.RETURNBACK1);
                }
                break;

            case RETURNBACK1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReturnBack1, true);
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