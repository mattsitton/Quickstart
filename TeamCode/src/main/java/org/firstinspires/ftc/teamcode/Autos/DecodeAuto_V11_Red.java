package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DECODE Auto: V11 RED (Mirrored)", group = "Autonomous")
public class DecodeAuto_V11_Red extends OpMode {

    private Follower follower;
    private LauncherSubsystem launcher;
    private Timer opModeTimer;

    private int pathState;
    private final int START = 0;
    private final int DRIVE_TO_SHOOT = 1;
    private final int SHOOTING = 2;
    private final int PARKING = 3;
    private final int DONE = 99;

    // --- COORDINATES (Mirrored using: ((72 - x) * 2) + x) ---

    // Start: Old X=56 -> New X=88. Heading mirrored (180 - 0)
    private final Pose startPose = new Pose(88, 89, Math.toRadians(180));

    // Shoot: Old X=54.7 -> New X=89.3. Heading mirrored (180 - 112 = 68)
    private final Pose shootPose = new Pose(89.3, 20.15, Math.toRadians(68));

    // Park: Old X=57 -> New X=87. Heading mirrored (180 - 360/0 = 180)
    private final Pose parkPose = new Pose(87, 57, Math.toRadians(180));

    private PathChain pathStartToShoot, pathPark;

    @Override
    public void init() {
        // 1. Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 2. Initialize your Custom Launcher Subsystem
        launcher = new LauncherSubsystem(hardwareMap);

        // 3. Initialize Timer and Paths
        opModeTimer = new Timer();
        buildPaths();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(START);
    }

    @Override
    public void loop() {
        // --- CRITICAL UPDATES ---
        follower.update();
        launcher.update(); // Keeps the RPM state machine running!

        // Run the Mission Logic
        autonomousLogic();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("Launcher Status", launcher.getStatus());
        telemetry.addData("Current RPM", "%.1f", launcher.getCurrentRPM());
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }

    private void autonomousLogic() {
        switch (pathState) {
            case START:
                // Move from Wall to Shooting Position
                follower.followPath(pathStartToShoot, true);
                setPathState(DRIVE_TO_SHOOT);
                break;

            case DRIVE_TO_SHOOT:
                // Wait until robot arrives at the spot
                if (!follower.isBusy()) {
                    // Trigger the 3-shot Smart Sequence
                    launcher.fireShots(3);
                    setPathState(SHOOTING);
                }
                break;

            case SHOOTING:
                // Wait for the Launcher Subsystem to finish all 3 shots
                if (!launcher.isBusy()) {
                    // Once done, drive to parking spot
                    follower.followPath(pathPark, true);
                    setPathState(PARKING);
                }
                break;

            case PARKING:
                // Wait for parking move to finish
                if (!follower.isBusy()) {
                    setPathState(DONE);
                }
                break;

            case DONE:
                // Mission Complete
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        opModeTimer.resetTimer();
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot
        pathStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Path 2: Shoot -> Park
        pathPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}