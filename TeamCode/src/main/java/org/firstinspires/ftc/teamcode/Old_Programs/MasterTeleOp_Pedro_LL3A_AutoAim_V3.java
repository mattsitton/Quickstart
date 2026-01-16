package org.firstinspires.ftc.teamcode.Old_Programs;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.TreeMap;
import java.util.Map;

@TeleOp(name = "MasterTeleOp_Pedro_LL3A_TagsOnly_V3", group = "TeleOp")
public class MasterTeleOp_Pedro_LL3A_AutoAim_V3 extends OpMode {

    // =========================================================
    //                TUNING VARIABLES
    // =========================================================

    private Follower follower;
    private final Pose DRIVE_TARGET = new Pose(72, 72, Math.toRadians(135));

    // --- MACRO TUNING ---
    public static double MACRO_ARRIVAL_TOLERANCE = 2.0; // Inches
    public static double MACRO_SHOT_DELAY = 0.8;        // Seconds between shots

    // --- PINPOINT ODOMETRY OFFSETS ---
    public static double PINPOINT_X_OFFSET_MM = 0.0;
    public static double PINPOINT_Y_OFFSET_MM = 0.0;

    // --- Auto-Align PID ---
    public static double AIM_kP = 0.03;
    public static double MAX_AUTO_TURN_SPEED = 0.6;

    // --- Flywheel Physics ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static double LAUNCHER_kP = 50;
    public static double LAUNCHER_kI = 0;
    public static double LAUNCHER_kD = 0;
    public static double LAUNCHER_kF = 12;

    // --- AUTO-AIM CALCULATOR VARIABLES ---
    public static double CAMERA_HEIGHT_INCH = 12.0;
    public static double CAMERA_PITCH_DEG   = 20.0;
    public static double TARGET_TAG_HEIGHT  = 44.5;

    // RPM Lookup Table
    private final TreeMap<Double, Integer> rpmTable = new TreeMap<>();
    {
        rpmTable.put(24.0, 220);
        rpmTable.put(48.0, 260);
        rpmTable.put(72.0, 300);
        rpmTable.put(96.0, 340);
    }

    // --- FEEDER TUNING ---
    public static double FEED_PULSE_TIME = 0.25;

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

    private DcMotorEx launcherMotor;
    private CRServo feeder;
    private Servo liftL, liftR;
    private VoltageSensor batteryVoltageSensor;
    private GoBildaPinpointDriver pinpoint;

    private Limelight3A limelight;
    private final int OBELISK_GPP = 21;
    private final int OBELISK_PGP = 22;
    private final int OBELISK_PPG = 23;

    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;

    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private boolean flywheelAtSpeed = false;

    private boolean tagVisible = false;
    private double targetTx = 0;
    private double targetTy = 0;
    private double calculatedDistance = 0.0;
    private int tagID = -1;
    private String detectedMotif = "None";

    private int autoShotsCount = 0;
    private ElapsedTime autoFireTimer = new ElapsedTime();
    private enum FireState { IDLE, FEEDING, WAITING }
    private FireState currentFireState = FireState.IDLE;

    @Override
    public void init() {
        // 1. Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        // 2. Pinpoint Offsets
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            telemetry.addLine("Pinpoint Configured!");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Pinpoint not found!");
        }

        // 3. Mechanisms
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD, LAUNCHER_kF));

        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL.setPosition(LIFT_L_HOME);
        liftR.setPosition(LIFT_R_HOME);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 4. Limelight (Pipeline 0 ONLY)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Initialized V3 (Tags Only)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double voltComp = 12.0 / currentVoltage;

        updateVisionLogic();

        // Toggle Field Centric (GP2 RB)
        if (gamepad2.right_bumper && !rightBumperPressed) {
            fieldCentric = !fieldCentric;
        }
        rightBumperPressed = gamepad2.right_bumper;

        // Reset Pose (GP2 Back)
        if (gamepad2.back) {
            follower.setStartingPose(new Pose(0,0,0));
        }

        // Re-Localize / Vision Sync (GP2 Start)
        if (gamepad2.start && tagVisible) {
            syncPedroToLimelight();
        }

        handleDrive();
        handleFeederAndFire();
        handleFlywheel(voltComp);
        handleLifts();

        follower.update();
        updateTelemetry();
    }

    private void handleDrive() {
        double fwd = -gamepad1.left_stick_y;
        double str = -gamepad1.left_stick_x;
        double rot = -gamepad1.right_stick_x;

        // Auto-Drive Macro (GP2 A)
        if (gamepad2.a) {
            follower.holdPoint(DRIVE_TARGET);
        }
        // Auto-Align (GP1 B)
        else if (gamepad1.b && tagVisible) {
            double error = -targetTx;
            rot = Range.clip(error * AIM_kP, -MAX_AUTO_TURN_SPEED, MAX_AUTO_TURN_SPEED);
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
        else {
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
    }

    private void updateVisionLogic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tagVisible = true;
            targetTx = result.getTx();
            targetTy = result.getTy();

            double angleToGoalRadians = Math.toRadians(CAMERA_PITCH_DEG + targetTy);
            calculatedDistance = (TARGET_TAG_HEIGHT - CAMERA_HEIGHT_INCH) / Math.tan(angleToGoalRadians);

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                tagID = fiducials.get(0).getFiducialId();
                if (tagID == OBELISK_GPP) detectedMotif = "G - P - P";
                else if (tagID == OBELISK_PGP) detectedMotif = "P - G - P";
                else if (tagID == OBELISK_PPG) detectedMotif = "P - P - G";
            }
        } else {
            tagVisible = false;
            detectedMotif = "None";
        }
    }

    private int getAutoRPM(double distance) {
        Map.Entry<Double, Integer> floor = rpmTable.floorEntry(distance);
        Map.Entry<Double, Integer> ceiling = rpmTable.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        if (floor.equals(ceiling)) return floor.getValue();

        double ratio = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return (int) (floor.getValue() + ratio * (ceiling.getValue() - floor.getValue()));
    }

    private void syncPedroToLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double xInches = (botpose.getPosition().x * 39.3701) + 72.0;
                double yInches = (botpose.getPosition().y * 39.3701) + 72.0;
                double headingRad = Math.toRadians(botpose.getOrientation().getYaw());
                follower.setPose(new Pose(xInches, yInches, headingRad));
            }
        }
    }

    private void handleFeederAndFire() {
        // Macro: Base & Fire (GP2 A)
        // Only fires if button is held AND robot is close to target
        if (gamepad2.a) {
            double currentX = follower.getPose().getX();
            double currentY = follower.getPose().getY();
            double targetX = DRIVE_TARGET.getX();
            double targetY = DRIVE_TARGET.getY();

            // Calculate manual distance
            double distanceToTarget = Math.hypot(targetX - currentX, targetY - currentY);

            // Check if we are close enough to start shooting
            if (distanceToTarget < MACRO_ARRIVAL_TOLERANCE) {
                if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                    autoShotsCount = 3; // Fire 3 shots
                    currentFireState = FireState.FEEDING;
                    autoFireTimer.reset();
                }
            }
        }
        // Shoot One Shot (GP1 X or GP2 X)
        else if (gamepad1.x || gamepad2.x) {
            if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                autoShotsCount = 1; // Fire 1 shot
                currentFireState = FireState.FEEDING;
                autoFireTimer.reset();
            }
        }

        // State Machine
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
                // Use MACRO_SHOT_DELAY for the delay
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
        // Changed to D-Pad (GP2)
        if (gamepad2.dpad_up) {
            liftL.setPosition(LIFT_L_ACTIVE);
            liftR.setPosition(LIFT_R_ACTIVE);
        } else if (gamepad2.dpad_down) {
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

        if (gamepad1.left_bumper && tagVisible) {
            currentTargetRPM = getAutoRPM(calculatedDistance);
        }
        else if (gamepad1.dpad_up)    currentTargetRPM = TARGET_RPM_UP;
        else if (gamepad1.dpad_right) currentTargetRPM = TARGET_RPM_RIGHT;
        else if (gamepad1.dpad_down)  currentTargetRPM = TARGET_RPM_DOWN;
        else if (gamepad1.dpad_left)  currentTargetRPM = TARGET_RPM_LEFT;

        if (currentTargetRPM > 0) {
            launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
            currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
            flywheelAtSpeed = (Math.abs(currentFlywheelRPM - currentTargetRPM) <= RPM_TOLERANCE);
        } else {
            launcherMotor.setPower(0);
            flywheelAtSpeed = false;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Motif", detectedMotif);

        if(tagVisible) {
            telemetry.addData("Vision", "Dist: %.1f in | ty: %.1f", calculatedDistance, targetTy);
            if (gamepad1.left_bumper) {
                telemetry.addData("AUTO-RPM", "Active: %d", currentTargetRPM);
            }
        } else {
            telemetry.addData("Vision", "No Tag Visible");
        }

        telemetry.addData("Flywheel", "Tgt:%d | Act:%.0f", currentTargetRPM, currentFlywheelRPM);
        telemetry.addData("Pedro Pose", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}