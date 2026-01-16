package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "DECODE Auto: Blue (Forward)", group = "Autonomous")
public class DecodeAuto_Forward_Blue extends OpMode {

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

    // Coordinates (Blue Linear)
    private final Pose startPose = new Pose(21.53, 122.27, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(58.17, 85.19, Math.toRadians(136));
    private final Pose intakePose = new Pose(18.84, 85.31, Math.toRadians(136));
    private final Pose shootPose2 = new Pose(58.08, 85.33, Math.toRadians(136));
    private final Pose parkPose = new Pose(58, 61, Math.toRadians(136));
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
        pathStartToShoot = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose1)).setLinearHeadingInterpolation(Math.toRadians(143), shootPose1.getHeading()).build();
        pathShootToIntake = follower.pathBuilder().addPath(new BezierLine(shootPose1, intakePose)).setTangentHeadingInterpolation().build();

        pathIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose2.getHeading())
                .setReversed() // FIX: Removed "true"
                .build();

        pathShootToPark = follower.pathBuilder().addPath(new BezierLine(shootPose2, parkPose)).setTangentHeadingInterpolation().build();
    }
}