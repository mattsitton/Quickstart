package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// --- FIXED IMPORTS ---
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;

// Import Constants from your Quickstart
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;

@Autonomous(name = "Sample Actions Tutorial")
public class SampleActionsTutorial extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private enum PathState {
        DRIVE_FROM_START,
        SHOOT_PRELOAD,
        DRIVE_FROM_SHOOT,
        END
    }
    private PathState pathState;

    // --- Poses ---
    // Update these coordinates for your field!
    private final Pose startPose = new Pose(42.2, 9, Math.toRadians(270));
    private final Pose shootPose = new Pose(57.98843930635838, 19.699421965317924, Math.toRadians(270));
    private final Pose endPose   = new Pose(41.0635838150289, 33.57225433526012, Math.toRadians(270));

    // --- Path Objects ---
    private Path startToShoot, shootToEnd;

    // --- Mechanism Logic ---
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;

    public void buildPaths() {
        // Path 1: Back up from start to shooting position
        // FIXED: Use Pose objects directly. No need for "new Point(...)"
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setConstantHeadingInterpolation(startPose.getHeading());

        // Path 2: Drive from shoot position to parking/end
        // FIXED: Use Pose objects directly
        shootToEnd = new Path(new BezierLine(shootPose, endPose));
        shootToEnd.setConstantHeadingInterpolation(shootPose.getHeading());
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Use Constants.createFollower to apply your PIDFs and settings
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        shooter.init(hardwareMap);

        pathState = PathState.DRIVE_FROM_START;
        shotsTriggered = false;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PathState.DRIVE_FROM_START);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        pathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter Busy", shooter.isBusy());
        // Simple telemetry for position
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void pathUpdate() {
        switch (pathState) {
            case DRIVE_FROM_START:
                follower.followPath(startToShoot);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(4);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(shootToEnd);
                        setPathState(PathState.DRIVE_FROM_SHOOT);
                    }
                }
                break;

            case DRIVE_FROM_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;

            case END:
                break;
        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }
}