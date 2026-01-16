package org.firstinspires.ftc.teamcode.Old_Programs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "DECODE Auto: Fixed V3", group = "Autonomous")
public class DecodeAuto_V3 extends OpMode {

    // =========================================================
    // 1. HARDWARE & SUBSYSTEMS
    // =========================================================
    private Follower follower;
    private LauncherSubsystem launcher;
    private Timer opModeTimer;

    // =========================================================
    // 2. STATE MACHINE
    // =========================================================
    private int pathState;
    private final int START = 0;
    private final int DRIVE_TO_SHOOT_1 = 1;
    private final int SHOOT_1 = 2;
    private final int DRIVE_TO_INTAKE = 3;
    private final int INTAKE_SWEEP = 4;
    private final int DRIVE_TO_SHOOT_2 = 5;
    private final int SHOOT_2 = 6;
    private final int TURN_AND_PARK = 7;
    private final int DONE = 99;

    // =========================================================
    // 3. POSES (COORDINATES)
    // =========================================================

    // Start: Back wall
    private final Pose startPose = new Pose(52, 9.2, Math.toRadians(90));

    // Shoot Pos: Aimed at goal (Blue Net)
    private final Pose shootPose = new Pose(55, 20, Math.toRadians(113));

    // Intake Start: Facing Back (180), ready to sweep
    private final Pose intakeStartPose = new Pose(41, 39, Math.toRadians(180));

    // Intake End: Drive "Forward" (relative to robot) towards X=22
    private final Pose intakeEndPose = new Pose(22, 39, Math.toRadians(180));

    // Park: Face 90 degrees
    private final Pose parkPose = new Pose(57, 32, Math.toRadians(90));

    // Paths
    private PathChain pathStartToShoot, pathShootToIntake, pathIntakeSweep, pathIntakeToShoot, pathPark;

    @Override
    public void init() {
        // --- Initialize Pedro Pathing ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // --- Initialize Mechanisms ---
        launcher = new LauncherSubsystem(hardwareMap);
        opModeTimer = new Timer();

        // --- Build Paths ---
        buildPaths();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(START);
    }

    @Override
    public void loop() {
        // Update Systems
        follower.update();
        launcher.update();

        // Run State Machine
        autonomousLogic();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Launcher", launcher.getStatus());
        telemetry.update();
    }

    // =========================================================
    // 4. AUTONOMOUS LOGIC
    // =========================================================
    private void autonomousLogic() {
        switch (pathState) {
            case START:
                // Action: Move Forward & Aim
                follower.followPath(pathStartToShoot, true);
                setPathState(DRIVE_TO_SHOOT_1);
                break;

            case DRIVE_TO_SHOOT_1:
                // Wait until we arrive
                if (!follower.isBusy()) {
                    // Action: Fire 3 Shots (Preload)
                    launcher.fireShots(3);
                    setPathState(SHOOT_1);
                }
                break;

            case SHOOT_1:
                // Wait for shots to finish
                if (!launcher.isBusy()) {
                    // Action: Line up for Intake
                    follower.followPath(pathShootToIntake, true);
                    setPathState(DRIVE_TO_INTAKE);
                }
                break;

            case DRIVE_TO_INTAKE:
                if (!follower.isBusy()) {
                    // Action: Start Intake & Drive through balls
                    launcher.startIntake();
                    follower.followPath(pathIntakeSweep, true);
                    setPathState(INTAKE_SWEEP);
                }
                break;

            case INTAKE_SWEEP:
                if (!follower.isBusy()) {
                    // Done sweeping. Stop Intake & Go Back to Shoot
                    launcher.stopIntake();
                    follower.followPath(pathIntakeToShoot, true);
                    setPathState(DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    // Action: Shoot the 3 balls we collected
                    launcher.fireShots(3);
                    setPathState(SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (!launcher.isBusy()) {
                    // Action: Park
                    follower.followPath(pathPark, true);
                    setPathState(TURN_AND_PARK);
                }
                break;

            case TURN_AND_PARK:
                if (!follower.isBusy()) {
                    setPathState(DONE);
                }
                break;

            case DONE:
                // Idle
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        opModeTimer.resetTimer();
    }

    // =========================================================
    // 5. PATH BUILDING
    // =========================================================
    private void buildPaths() {
        // 1. Start -> Shoot
        // Uses Pose objects directly (No Point/BezierPoint class needed)
        pathStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // 2. Shoot -> Intake Start
        Pose controlPose = new Pose(50, 35, 0);
        pathShootToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPose, intakeStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStartPose.getHeading())
                .build();

        // 3. Intake Sweep
        pathIntakeSweep = follower.pathBuilder()
                .addPath(new BezierLine(intakeStartPose, intakeEndPose))
                .setConstantHeadingInterpolation(intakeStartPose.getHeading())
                .build();

        // 4. Intake -> Shoot (Return)
        pathIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeEndPose, shootPose))
                .setLinearHeadingInterpolation(intakeEndPose.getHeading(), shootPose.getHeading())
                .build();

        // 5. Park
        pathPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    // =========================================================
    // 6. LAUNCHER SUBSYSTEM (Updated for V6 Hardware)
    // =========================================================
    public static class LauncherSubsystem {
        // Hardware
        private DcMotorEx launcherMotor;
        private CRServo feeder, intake;
        private Servo loader, liftL, liftR;

        // Tuning
        private final double FLWHEEL_TICKS_PER_REV = 534.8;
        private final double TARGET_RPM = 300;
        private final double LOADER_INSERT = 0.5;
        private final double LOADER_RETRACT = 0.0;
        private final double SHOT_DELAY_MS = 800; // 0.8 seconds

        // State Logic
        private int shotsToFire = 0;
        private Timer mechTimer = new Timer();
        private enum State { IDLE, SPINNING_UP, PUSHING, RETRACTING, INTAKING }
        private State state = State.IDLE;

        public LauncherSubsystem(HardwareMap hardwareMap) {
            // Launcher Setup
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(50, 0, 0, 12));

            // Servo Setup
            feeder = hardwareMap.get(CRServo.class, "feeder");
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);

            intake = hardwareMap.get(CRServo.class, "intake");

            loader = hardwareMap.get(Servo.class, "loader");
            loader.setPosition(LOADER_RETRACT);

            // Lift Safety
            try {
                liftL = hardwareMap.get(Servo.class, "liftL");
                liftR = hardwareMap.get(Servo.class, "liftR");
                liftL.setPosition(0.0);
                liftR.setPosition(0.0);
            } catch (Exception e) {}
        }

        public void fireShots(int count) {
            shotsToFire = count;
            state = State.SPINNING_UP;
            mechTimer.resetTimer();
        }

        public void startIntake() {
            state = State.INTAKING;
        }

        public void stopIntake() {
            state = State.IDLE;
        }

        public boolean isBusy() {
            return state != State.IDLE && state != State.INTAKING;
        }

        public String getStatus() {
            return state.toString();
        }

        public void update() {
            switch (state) {
                case IDLE:
                    launcherMotor.setPower(0);
                    feeder.setPower(0);
                    intake.setPower(0);
                    loader.setPosition(LOADER_RETRACT);
                    break;

                case INTAKING:
                    launcherMotor.setPower(0);
                    feeder.setPower(1.0); // Intake ON
                    intake.setPower(1.0);
                    loader.setPosition(LOADER_RETRACT);
                    break;

                case SPINNING_UP:
                    feeder.setPower(0);
                    intake.setPower(0);
                    loader.setPosition(LOADER_RETRACT);

                    double vel = (TARGET_RPM / 60.0) * FLWHEEL_TICKS_PER_REV;
                    launcherMotor.setVelocity(vel);

                    // Wait 500ms for spin up
                    if (mechTimer.getElapsedTime() > 500) {
                        state = State.PUSHING;
                        mechTimer.resetTimer();
                    }
                    break;

                case PUSHING:
                    loader.setPosition(LOADER_INSERT); // Push ball in
                    // Quick push (250ms)
                    if (mechTimer.getElapsedTime() > 250) {
                        state = State.RETRACTING;
                        mechTimer.resetTimer();
                    }
                    break;

                case RETRACTING:
                    loader.setPosition(LOADER_RETRACT);
                    // Wait for delay before next shot
                    if (mechTimer.getElapsedTime() > SHOT_DELAY_MS) {
                        shotsToFire--;
                        if (shotsToFire > 0) {
                            state = State.PUSHING; // Fire next
                        } else {
                            state = State.IDLE; // Done
                        }
                        mechTimer.resetTimer();
                    }
                    break;
            }
        }
    }
}