package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class autoTemplate extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);
    private final Pose controlPose = p.of(36, 60, 45);
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);
    /* Creating a Line
Let's break down the parts of the path quickly:
line() creates a straight path from startPose to park.

The .linear() part is our Interpolation.
It changes the robot's heading from the heading of startPose to the heading of park
while it moves along the path.
You can play around with this on the visualizer.
 */
    private Path linePark() {
    return line(startPose, park).linear(startPose, park);
    }

/*
To make a Curve
The first and last Poses are the start and end of the curve.
Any Poses between them are control points that change the shape of the curve.
 */

    private Path curvePark() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }

/* now let us make multiple paths to use in the scheduler */
private Path startToScore() {
    return line(startPose, scorePose).linear(startPose, scorePose);
}
private Path park(){
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);//sets the starting point of your Robot
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }



}
