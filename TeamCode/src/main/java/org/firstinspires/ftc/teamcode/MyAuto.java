package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "My Auto", group = "Examples")
public class MyAuto extends OpMode {

    private Follower follower;

    private PathChain grab1, grab2, returnPath;

    public void buildPaths() {

        grab1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.151, 7.908),
                                new Pose(86.243, 34.717)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(86.243, 34.717),
                                new Pose(130.539, 35.019)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grab2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.539, 35.019),
                                new Pose(85.206, 46.766),
                                new Pose(79.496, 80.923)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(79.496, 80.923),
                                new Pose(64.694, 53.705),
                                new Pose(120.986, 58.922)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        returnPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.986, 58.922),
                                new Pose(79.544, 80.918)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, grab1,true),

                follow(follower, grab2, true),

                follow(follower, returnPath, true)
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.151, 7.908, Math.toRadians(0)));
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("x",       follower.getPose().getX());
        telemetry.addData("y",       follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}