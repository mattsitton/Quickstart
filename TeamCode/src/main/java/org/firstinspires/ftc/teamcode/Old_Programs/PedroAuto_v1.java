package org.firstinspires.ftc.teamcode.Old_Programs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

// IMPORTANT: Import the Constants class we just fixed
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto - Back Shooter", group = "Pedro")
public class PedroAuto_v1 extends OpMode {

    // --- Pedro Pathing ---
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // --- Hardware (Matches MasterTeleOp_V6) ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder, intake;
    private Servo liftL, liftR;

    // --- Mechanism Constants ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static int TARGET_RPM_AUTO = 300;
    public static int RPM_TOLERANCE = 15;

    // --- Poses ---
    // Start Pose: Facing Forward (270 deg) at the wall
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    // Score Pose: Back of robot faces the basket (Basket is top-left)
    // Robot Angle: ~315 Degrees (-45) puts the BACK pointing at the basket.
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    // Pickup Pose: Facing the sample on the floor
    private final Pose pickupPose = new Pose(37, 121, Math.toRadians(0));

    // --- Paths ---
    private Path startToScore, scoreToPickup, pickupToScore;

    // --- State Machine ---
    private int pathState;

    // Timer for mechanism delays
    private final com.qualcomm.robotcore.util.ElapsedTime actionTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    public void buildPaths() {
        // Path 1: Drive to Score
        // We use 'new BezierLine(Pose, Pose)' directly to avoid "Point" errors
        startToScore = new Path(new BezierLine(startPose, scorePose));
        startToScore.setConstantHeadingInterpolation(scorePose.getHeading());

        // Path 2: Drive to Pickup Sample
        scoreToPickup = new Path(new BezierLine(scorePose, pickupPose));
        scoreToPickup.setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading());

        // Path 3: Drive back to Score
        pickupToScore = new Path(new BezierLine(pickupPose, scorePose));
        pickupToScore.setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading());
    }

    @Override
    public void init() {
        // 1. Initialize Follower using your custom Constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        // 2. Initialize Hardware (Same setup as your TeleOp)
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PIDF from V6
        PIDFCoefficients pidf = new PIDFCoefficients(50, 0, 0, 12);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(CRServo.class, "intake");

        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL.setPosition(0.0);
        liftR.setPosition(0.0);

        // Initialize Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // REQUIRED: Update the follower every loop
        follower.update();

        // Run our auto logic
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Flywheel RPM", getFlywheelRPM());
        telemetry.update();
    }

    // --- Main Logic State Machine ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start -> Score
                follower.followPath(startToScore);
                setFlywheelRPM(TARGET_RPM_AUTO); // Spin up while moving
                setPathState(1);
                break;

            case 1: // Wait for Arrival at Score Pose
                if (!follower.isBusy()) {
                    // Once there, check if flywheel is ready
                    if (isFlywheelReady()) {
                        startFeeders(); // FIRE!
                        actionTimer.reset();
                        setPathState(2);
                    }
                }
                break;

            case 2: // Wait for Shot to finish
                if (actionTimer.seconds() > 0.4) { // 0.4s to clear the intake
                    stopFeeders();
                    setPathState(3);
                }
                break;

            case 3: // Score -> Pickup
                follower.followPath(scoreToPickup);
                launcherMotor.setPower(0); // Stop Flywheel

                // Turn on Intake
                intake.setPower(1.0);
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);

                setPathState(4);
                break;

            case 4: // Wait for Arrival at Pickup
                if (!follower.isBusy()) {
                    // Assume we grabbed the sample, move back to score
                    setPathState(5);
                }
                break;

            case 5: // Pickup -> Score
                intake.setPower(0);
                stopFeeders();

                follower.followPath(pickupToScore);
                setFlywheelRPM(TARGET_RPM_AUTO); // Spin up again
                setPathState(6);
                break;

            case 6: // Wait for Arrival & Shoot
                if (!follower.isBusy() && isFlywheelReady()) {
                    startFeeders();
                    actionTimer.reset();
                    setPathState(7);
                }
                break;

            case 7: // Finish
                if (actionTimer.seconds() > 0.4) {
                    stopFeeders();
                    launcherMotor.setPower(0);
                    setPathState(-1); // End state
                }
                break;
        }
    }

    // --- Helper Methods ---

    private void setFlywheelRPM(int rpm) {
        launcherMotor.setVelocity((rpm / 60.0) * FLWHEEL_TICKS_PER_REV);
    }

    private double getFlywheelRPM() {
        return (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
    }

    private boolean isFlywheelReady() {
        return Math.abs(getFlywheelRPM() - TARGET_RPM_AUTO) <= RPM_TOLERANCE;
    }

    private void startFeeders() {
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
        intake.setPower(1.0);
    }

    private void stopFeeders() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        intake.setPower(0);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}