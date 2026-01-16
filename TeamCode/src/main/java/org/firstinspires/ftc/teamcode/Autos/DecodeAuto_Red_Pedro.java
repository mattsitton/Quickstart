package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DECODE Auto: Red (Standard) [FIXED]", group = "Autonomous")
public class DecodeAuto_Red_Pedro extends OpMode {

    private Follower follower;
    private DcMotorEx launcherMotor;
    private DcMotor intakeMotor;
    private CRServo feeder;
    private Timer opModeTimer, fireTimer;

    private final double FEED_TIME = 0.25;
    private final double SHOT_DELAY = 0.8;
    private final double LAUNCHER_VELOCITY = 2600;
    private final double INTAKE_POWER = 1.0;
    private final double INTAKE_DRIVE_SPEED = 0.5;

    private int pathState;
    private final int START=0, DRIVE_TO_SHOOT=1, SHOOTING_1=2, DRIVE_TO_INTAKE=3, DRIVE_TO_SHOOT_2=4, SHOOTING_2=5, DRIVE_TO_PARK=6, DONE=99;
    private int shotsToFire = 0;
    private boolean isFiring = false;
    private enum FireState { IDLE, PUSH, WAIT }
    private FireState currentFireState = FireState.IDLE;

    // Coordinates (Red Curve)
    private final Pose startPose = new Pose(82, 9.4, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(87.21, 17.63, Math.toRadians(70));
    private final Pose intakeInterPose = new Pose(102.95, 35.46, Math.toRadians(70));
    private final Pose intakeDonePose = new Pose(126.73, 35.12, Math.toRadians(70));
    private final Pose shootPose2 = new Pose(87.31, 17.57, Math.toRadians(70));
    private final Pose parkPose = new Pose(87.44, 38.53, Math.toRadians(90));

    // Control Points (as Pose)
    private final Pose intakeControlPoint = new Pose(93.63, 35.36, 0);
    private final Pose returnControlPoint = new Pose(94.96, 45.11, 0);

    private PathChain pathStartToShoot, pathShootToIntake, pathIntakeToShoot, pathShootToPark;

    @Override public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0, 12));
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        opModeTimer = new Timer(); fireTimer = new Timer();
        buildPaths();
    }

    @Override public void start() { opModeTimer.resetTimer(); setPathState(START); }
    @Override public void loop() { follower.update(); handleFiringLogic(); autonomousLogic(); telemetry.addData("State", pathState); telemetry.update(); }

    private void autonomousLogic() {
        switch (pathState) {
            case START:
                launcherMotor.setVelocity(LAUNCHER_VELOCITY);
                follower.followPath(pathStartToShoot, true);
                setPathState(DRIVE_TO_SHOOT);
                break;
            case DRIVE_TO_SHOOT:
                if (!follower.isBusy()) {
                    startFiringSequence(4);
                    setPathState(SHOOTING_1);
                }
                break;
            case SHOOTING_1:
                if (!isFiring) {
                    intakeMotor.setPower(INTAKE_POWER);
                    follower.setMaxPower(INTAKE_DRIVE_SPEED);
                    follower.followPath(pathShootToIntake, true);
                    setPathState(DRIVE_TO_INTAKE);
                }
                break;
            case DRIVE_TO_INTAKE:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.setMaxPower(1.0);
                    launcherMotor.setVelocity(LAUNCHER_VELOCITY);
                    follower.followPath(pathIntakeToShoot, true);
                    setPathState(DRIVE_TO_SHOOT_2);
                }
                break;
            case DRIVE_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    startFiringSequence(4);
                    setPathState(SHOOTING_2);
                }
                break;
            case SHOOTING_2:
                if (!isFiring) {
                    launcherMotor.setPower(0);
                    follower.followPath(pathShootToPark, true);
                    setPathState(DRIVE_TO_PARK);
                }
                break;
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) setPathState(DONE);
                break;
        }
    }

    private void startFiringSequence(int shots) { shotsToFire = shots; currentFireState = FireState.PUSH; fireTimer.resetTimer(); isFiring = true; }

    private void handleFiringLogic() {
        if (!isFiring) return;
        switch (currentFireState) {
            case PUSH: feeder.setPower(1.0); if (fireTimer.getElapsedTime() / 1e9 > FEED_TIME) { currentFireState = FireState.WAIT; fireTimer.resetTimer(); } break;
            case WAIT: feeder.setPower(0); if (fireTimer.getElapsedTime() / 1e9 > SHOT_DELAY) { shotsToFire--; if (shotsToFire > 0) currentFireState = FireState.PUSH; else { isFiring = false; currentFireState = FireState.IDLE; } fireTimer.resetTimer(); } break;
        }
    }

    private void setPathState(int pState) { pathState = pState; opModeTimer.resetTimer(); }

    private void buildPaths() {
        // FIX: Added '0.5' endTime for Linear Heading Interpolation
        pathStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading(), 0.5)
                .build();

        pathShootToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, intakeControlPoint, intakeInterPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(intakeInterPose, intakeDonePose))
                .setTangentHeadingInterpolation()
                .build();

        // FIX: Replaced setReversed() with Constant Heading Interpolation
        // Since you are driving back to the shoot pose but want to keep the shoot heading (~70 deg),
        // ConstantHeadingInterpolation is the most reliable way to do this in v2.
        pathIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(intakeDonePose, returnControlPoint, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .build();

        pathShootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, parkPose))
                .setTangentHeadingInterpolation()
                .build();
    }
}