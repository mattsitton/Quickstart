
// MasterTeleOp_V7 {

package org.firstinspires.ftc.teamcode.Old_Programs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "MasterTeleOp_V6 (No Intake)", group = "TeleOp")
public class MasterTeleOp_Pedro_V7 extends OpMode {

    // =========================================================
    //                TUNING VARIABLES
    // =========================================================

    // --- Pedro Pathing ---
    private Follower follower;

    // --- TARGETS ---
    private final Pose DRIVE_TARGET = new Pose(72, 72, Math.toRadians(135));
    private final Pose FACE_TARGET  = new Pose(0, 144, 0);

    // --- Auto-Align PID ---
    public static double AIM_kP = 1.5;

    // --- Flywheel PID & Physics ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static double LAUNCHER_kP = 50;
    public static double LAUNCHER_kI = 0;
    public static double LAUNCHER_kD = 0;
    public static double LAUNCHER_kF = 12;

    // --- FEEDER TUNING ---
    public static double FEED_PULSE_TIME = 0.25;
    public static double SHOT_DELAY      = 0.8;

    // --- Flywheel Target RPMs ---
    public static int TARGET_RPM_UP    = 300;
    public static int TARGET_RPM_RIGHT = 280;
    public static int TARGET_RPM_DOWN  = 260;
    public static int TARGET_RPM_LEFT  = 240;
    public static int RPM_TOLERANCE    = 15;

    // --- AUX SERVO TUNING ---
    public static double LIFT_L_HOME   = 0.0;
    public static double LIFT_L_ACTIVE = 0.5;
    public static double LIFT_R_HOME   = 0.0;
    public static double LIFT_R_ACTIVE = 0.5;

    // =========================================================
    //                  HARDWARE & STATE
    // =========================================================

    // --- Hardware ---
    private DcMotorEx launcherMotor;
    private CRServo feeder; // No intake, no loader
    private Servo liftL, liftR;
    private VoltageSensor batteryVoltageSensor;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
    private final int BLUE_GOAL_ID = 20, RED_GOAL_ID = 24;

    // --- State Variables ---
    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;

    // Flywheel State
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private boolean flywheelAtSpeed = false;

    // Vision Global State
    private boolean tagVisible = false;
    private int tagID = -1;

    // Auto-Fire State Machine
    private int autoShotsCount = 0;
    private ElapsedTime autoFireTimer = new ElapsedTime();
    private enum FireState { IDLE, FEEDING, WAITING }
    private FireState currentFireState = FireState.IDLE;

    @Override
    public void init() {
        // --- 1. Initialize Pedro Pathing Follower ---
        follower = Constants.createFollower(hardwareMap);

        // --- 2. Initialize Mechanisms ---
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD, LAUNCHER_kF));

        // --- SERVO SETUP ---
        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL.setPosition(LIFT_L_HOME);
        liftR.setPosition(LIFT_R_HOME);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- 3. Initialize Vision ---
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Initialized V6 (No Intake)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // 0. Update Voltage
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double voltComp = 12.0 / currentVoltage;

        // 1. Inputs & Toggles
        if (gamepad1.right_bumper && !rightBumperPressed) {
            fieldCentric = !fieldCentric;
        }
        rightBumperPressed = gamepad1.right_bumper;

        // --- GAMEPAD 1 A: RESET POSE ---
        if (gamepad1.a) {
            follower.setStartingPose(new Pose(0,0,0));
        }

        // 2. Systems
        handleDrive();
        handleFeederAndFire();
        handleFlywheel(voltComp);
        handleLifts();
        updateVisionData();

        // 3. Update Pedro
        follower.update();

        // 4. Telemetry
        updateTelemetry();
    }

    // --- Helper Methods ---

    private void handleDrive() {
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // GP2 A: Auto-Drive to (72, 72)
        if (gamepad2.a) {
            follower.holdPoint(DRIVE_TARGET);
        }
        // GP1 Y: Auto-Face
        else if (gamepad1.y) {
            double dx = FACE_TARGET.getX() - follower.getPose().getX();
            double dy = FACE_TARGET.getY() - follower.getPose().getY();
            double targetHeading = Math.atan2(dy, dx);
            double error = AngleUnit.normalizeRadians(targetHeading - follower.getPose().getHeading());
            rot = Range.clip(error * AIM_kP, -1, 1);
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
        else {
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
    }

    private void handleFeederAndFire() {
        // PRIORITY 1: Auto-Fire (GP2 A held)
        if (gamepad2.a) {
            if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                autoShotsCount = 3;
                currentFireState = FireState.FEEDING;
                autoFireTimer.reset();
            }

            switch (currentFireState) {
                case FEEDING:
                    feeder.setPower(1.0);
                    if (autoFireTimer.seconds() > FEED_PULSE_TIME) {
                        currentFireState = FireState.WAITING;
                        autoFireTimer.reset();
                    }
                    break;
                case WAITING:
                    feeder.setPower(0);
                    if (autoFireTimer.seconds() > SHOT_DELAY) {
                        autoShotsCount--;
                        if (autoShotsCount > 0) {
                            currentFireState = FireState.FEEDING;
                        } else {
                            currentFireState = FireState.IDLE;
                        }
                        autoFireTimer.reset();
                    }
                    break;
                case IDLE:
                    feeder.setPower(0);
                    break;
            }
        }
        // PRIORITY 2: Manual Feed (GP1 X)
        else if (gamepad1.x) {
            currentFireState = FireState.IDLE;
            autoShotsCount = 0;
            feeder.setPower(1.0);
        }
        // PRIORITY 3: Manual Feed (GP2 B)
        else if (gamepad2.b) {
            currentFireState = FireState.IDLE;
            autoShotsCount = 0;
            feeder.setPower(1.0);
        }
        else {
            currentFireState = FireState.IDLE;
            autoShotsCount = 0;
            feeder.setPower(0);
        }
    }

    private void handleLifts() {
        if (gamepad2.left_stick_y < -0.5) {
            liftL.setPosition(LIFT_L_ACTIVE);
            liftR.setPosition(LIFT_R_ACTIVE);
        } else if (gamepad2.left_stick_y > 0.5) {
            liftL.setPosition(LIFT_L_HOME);
            liftR.setPosition(LIFT_R_HOME);
        }
    }

    private void handleFlywheel(double voltComp) {
        double manual = gamepad1.left_trigger;

        if (manual > 0.1) {
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(Range.clip((manual / 0.8) * voltComp, 0, 1));
            currentTargetRPM = 0;
            flywheelAtSpeed = false;
            return;
        }

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad1.dpad_up)    currentTargetRPM = TARGET_RPM_UP;
        if (gamepad1.dpad_right) currentTargetRPM = TARGET_RPM_RIGHT;
        if (gamepad1.dpad_down)  currentTargetRPM = TARGET_RPM_DOWN;
        if (gamepad1.dpad_left)  currentTargetRPM = TARGET_RPM_LEFT;

        if (currentTargetRPM > 0) {
            launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
            currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
            flywheelAtSpeed = (Math.abs(currentFlywheelRPM - currentTargetRPM) <= RPM_TOLERANCE);
        } else {
            launcherMotor.setPower(0);
            flywheelAtSpeed = false;
        }
    }

    private void updateVisionData() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && (detection.id == BLUE_GOAL_ID || detection.id == RED_GOAL_ID)) {
                tagVisible = true;
                tagID = detection.id;
                break;
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        if (gamepad2.a) {
            telemetry.addData("Status", ">>> AUTO-PILOT ENGAGED <<<");
        } else {
            telemetry.addData("Status", "Manual Control");
        }
        telemetry.addData("Flywheel", "Tgt:%d | Act:%.0f", currentTargetRPM, currentFlywheelRPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}