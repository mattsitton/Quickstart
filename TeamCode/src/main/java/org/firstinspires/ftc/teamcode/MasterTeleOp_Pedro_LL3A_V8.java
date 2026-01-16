package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.TreeMap;
import java.util.Map;

@TeleOp(name = "MasterTeleOp_Pedro_V8_NoDriveEnc", group = "TeleOp")
public class MasterTeleOp_Pedro_LL3A_V8 extends OpMode {

    // =========================================================
    //                 SECTION 1: TUNING & CONFIG
    // =========================================================

    // --- DRIVE & AUTO-ALIGN ---
    private final Pose DRIVE_TARGET = new Pose(72, 72, Math.toRadians(134));
    public static double AIM_kP = 0.03;
    public static double MAX_AUTO_TURN_SPEED = 0.6;
    public static double MACRO_ARRIVAL_TOLERANCE = 2.0;

    // --- LAUNCHER PHYSICS (Encoder Enabled) ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static double LAUNCHER_kP = 50;
    public static double LAUNCHER_kI = 0;
    public static double LAUNCHER_kD = 0;
    public static double LAUNCHER_kF = 12;

    // --- FIRING TIMING ---
    public static double FEED_PULSE_TIME = 0.25;
    public static double MACRO_SHOT_DELAY = 0.8;

    // --- RPM PRESETS ---
    public static int TARGET_RPM_UP    = 300;
    public static int TARGET_RPM_RIGHT = 280;
    public static int TARGET_RPM_DOWN  = 260;
    public static int TARGET_RPM_LEFT  = 240;

    // --- RPM DISTANCE TABLE ---
    private final TreeMap<Double, Integer> rpmTable = new TreeMap<>();
    {
        rpmTable.put(24.0, 220); //24 Inches from the target
        rpmTable.put(48.0, 260); //48 Inches from target
        rpmTable.put(72.0, 300); //72 Inches form target
        rpmTable.put(96.0, 340); // 96 Inches fron target
    }

    // --- VISION CALCULATOR ---
    public static double CAMERA_HEIGHT_INCH = 12.0; // Need to measure
    public static double CAMERA_PITCH_DEG   = 20.0; //ToDO: Measure angles, target height, and camera height
    public static double TARGET_TAG_HEIGHT  = 44.5;

    // --- AUX SERVOS ---
    public static double LIFT_L_HOME   = 0.0; // lift ToDO tune
    public static double LIFT_L_ACTIVE = 0.5;
    public static double LIFT_R_HOME   = 0.0;
    public static double LIFT_R_ACTIVE = 0.5;

    // =========================================================
    //                 SECTION 2: HARDWARE & STATE
    // =========================================================

    private DcMotorEx launcherMotor; //control hub port 2
    private CRServo feeder;  //TODO : Add second servo port 1 one control hub
    private Servo liftL, liftR; // ports 3 and 4
    private VoltageSensor batteryVoltageSensor;
    private Limelight3A limelight;
    private Follower follower; // Handles Pinpoint internally

    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private int autoShotsCount = 0;
    private ElapsedTime autoFireTimer = new ElapsedTime();
    private enum FireState { IDLE, FEEDING, WAITING }
    private FireState currentFireState = FireState.IDLE;

    private boolean tagVisible = false;
    private double targetTx = 0;
    private double targetTy = 0;
    private double calculatedDistance = 0.0;
    private int tagID = -1;
    private String detectedMotif = "None";

    private final int OBELISK_GPP = 21;
    private final int OBELISK_PGP = 22;
    private final int OBELISK_PPG = 23;

    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;

    // =========================================================
    //                 SECTION 3: INITIALIZATION
    // =========================================================

    @Override
    public void init() {
        // 1. Pedro Follower (Handles Drive & Pinpoint)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0)); // add starting pose 

        // --- CRITICAL: FORCE DRIVE MOTORS TO NO ENCODER MODE ---
        forceDriveMotorsToNoEncoder();

        // 2. Launcher Motor (Using Encoder)
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD, LAUNCHER_kF));

        // 3. Feeder & Lifts
        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL.setPosition(LIFT_L_HOME);
        liftR.setPosition(LIFT_R_HOME);

        // 4. Sensors
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Initialized: V8 (Drive Encoders DISABLED)");
        telemetry.update();
    }

    // Helper to ensure drive motors don't use broken encoders
    private void forceDriveMotorsToNoEncoder() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRight");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    // =========================================================
    //                 SECTION 4: MAIN LOOP
    // =========================================================

    @Override
    public void loop() {
        double voltComp = 12.0 / batteryVoltageSensor.getVoltage();
        updateVisionLogic();

        handleDrive();
        handleFlywheel(voltComp);
        handleFeederAndFire();
        handleLifts();

        follower.update();
        updateTelemetry();
    }

    private void handleDrive() {
        if (gamepad2.right_bumper && !rightBumperPressed) fieldCentric = !fieldCentric;
        rightBumperPressed = gamepad2.right_bumper;

        if (gamepad2.y) {
            follower.setStartingPose(new Pose(0,0,0));
        }

        if (gamepad2.b && tagVisible) {
            syncPedroToLimelight();
        }

        double fwd = -gamepad1.left_stick_y;
        double str = -gamepad1.left_stick_x;
        double rot = -gamepad1.right_stick_x;

        if (gamepad2.a) {
            follower.holdPoint(DRIVE_TARGET);
        } else if (gamepad1.b && tagVisible) {
            double error = -targetTx;
            rot = Range.clip(error * AIM_kP, -MAX_AUTO_TURN_SPEED, MAX_AUTO_TURN_SPEED);
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        } else {
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
    }

    private void handleFlywheel(double voltComp) {
        double manualTrigger = gamepad1.left_trigger;

        if (manualTrigger > 0.1) {
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(Range.clip((manualTrigger / 0.8) * voltComp, 0, 1));
            currentTargetRPM = 0;
        } else {
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.left_bumper && tagVisible) {
                currentTargetRPM = getAutoRPM(calculatedDistance);
            } else if (gamepad1.dpad_up)    currentTargetRPM = TARGET_RPM_UP;
            else if (gamepad1.dpad_right)   currentTargetRPM = TARGET_RPM_RIGHT;
            else if (gamepad1.dpad_down)    currentTargetRPM = TARGET_RPM_DOWN;
            else if (gamepad1.dpad_left)    currentTargetRPM = TARGET_RPM_LEFT;

            if (currentTargetRPM > 0) {
                launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
            } else {
                launcherMotor.setPower(0);
            }
        }
        currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
    }

    private void handleFeederAndFire() {
        if (gamepad2.a) {
            double distToTarget = Math.hypot(DRIVE_TARGET.getX() - follower.getPose().getX(),
                    DRIVE_TARGET.getY() - follower.getPose().getY());
            if (distToTarget < MACRO_ARRIVAL_TOLERANCE) {
                if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                    autoShotsCount = 4;
                    currentFireState = FireState.FEEDING;
                    autoFireTimer.reset();
                }
            }
        }
        else if (gamepad1.x || gamepad2.x) {
            if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                autoShotsCount = 4;
                currentFireState = FireState.FEEDING;
                autoFireTimer.reset();
            }
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
                if (autoFireTimer.seconds() > MACRO_SHOT_DELAY) {
                    autoShotsCount--;
                    if (autoShotsCount > 0) currentFireState = FireState.FEEDING;
                    else currentFireState = FireState.IDLE;
                    autoFireTimer.reset();
                }
                break;
            case IDLE:
                feeder.setPower(0);
                break;
        }
    }

    private void handleLifts() {
        if (gamepad2.dpad_up) { liftL.setPosition(LIFT_L_ACTIVE); liftR.setPosition(LIFT_R_ACTIVE); }
        else if (gamepad2.dpad_down) { liftL.setPosition(LIFT_L_HOME); liftR.setPosition(LIFT_R_HOME); }
    }

    private void updateVisionLogic() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tagVisible = true;
            targetTx = result.getTx();
            targetTy = result.getTy();
            double angle = Math.toRadians(CAMERA_PITCH_DEG + targetTy);
            calculatedDistance = (TARGET_TAG_HEIGHT - CAMERA_HEIGHT_INCH) / Math.tan(angle);
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (!fids.isEmpty()) tagID = fids.get(0).getFiducialId();
        } else tagVisible = false;
    }

    private int getAutoRPM(double distance) {
        Map.Entry<Double, Integer> floor = rpmTable.floorEntry(distance);
        Map.Entry<Double, Integer> ceiling = rpmTable.ceilingEntry(distance);
        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        double ratio = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return (int) (floor.getValue() + ratio * (ceiling.getValue() - floor.getValue()));
    }

    private void syncPedroToLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = (botpose.getPosition().x * 39.3701) + 72.0;
                double y = (botpose.getPosition().y * 39.3701) + 72.0;
                double h = Math.toRadians(botpose.getOrientation().getYaw());
                follower.setPose(new Pose(x, y, h));
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Target RPM", currentTargetRPM);
        telemetry.addData("Actual RPM", "%.0f", currentFlywheelRPM);
        telemetry.addData("Loc", follower.getPose().toString());
        telemetry.update();
    }
}