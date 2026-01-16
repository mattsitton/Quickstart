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

@TeleOp(name = "MasterTeleOp_Pedro_LL3A_AutoAim_V6", group = "TeleOp")
public class MasterTeleOp_Pedro_LL3A_AutoAim_V6 extends OpMode {

    // =========================================================
    //                 SECTION 1: TUNING & CONFIG
    //           (Edit these values to tune the robot)
    // =========================================================

    // --- DRIVE & AUTO-ALIGN ---
    private final Pose DRIVE_TARGET = new Pose(72, 72, Math.toRadians(135)); // Target for "A" Button
    public static double AIM_kP = 0.03;              // Auto-Align turn power per degree of error
    public static double MAX_AUTO_TURN_SPEED = 0.6;  // Cap for auto-turn speed
    public static double MACRO_ARRIVAL_TOLERANCE = 2.0; // Inches (How close to be before firing)

    // --- LAUNCHER PHYSICS (V6 Style - Coast) ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static double LAUNCHER_kP = 50;
    public static double LAUNCHER_kI = 0;
    public static double LAUNCHER_kD = 0;
    public static double LAUNCHER_kF = 12;

    // --- FIRING TIMING (V3 Style - Time Based) ---
    public static double FEED_PULSE_TIME = 0.25;     // How long the servo pushes (seconds)
    public static double MACRO_SHOT_DELAY = 0.8;     // Time between shots in auto-fire (seconds)

    // --- RPM PRESETS ---
    public static int TARGET_RPM_UP    = 300;
    public static int TARGET_RPM_RIGHT = 280;
    public static int TARGET_RPM_DOWN  = 260;
    public static int TARGET_RPM_LEFT  = 240;

    // --- RPM DISTANCE TABLE ---
    // Maps Distance (Inches) -> Target RPM
    private final TreeMap<Double, Integer> rpmTable = new TreeMap<>();
    {
        rpmTable.put(24.0, 220);
        rpmTable.put(48.0, 260);
        rpmTable.put(72.0, 300);
        rpmTable.put(96.0, 340);
    }

    // --- VISION CALCULATOR ---
    public static double CAMERA_HEIGHT_INCH = 12.0;
    public static double CAMERA_PITCH_DEG   = 20.0;
    public static double TARGET_TAG_HEIGHT  = 44.5;

    // --- AUX SERVOS ---
    public static double LIFT_L_HOME   = 0.0;
    public static double LIFT_L_ACTIVE = 0.5;
    public static double LIFT_R_HOME   = 0.0;
    public static double LIFT_R_ACTIVE = 0.5;

    // --- PINPOINT OFFSETS ---
    public static double PINPOINT_X_OFFSET_MM = 0.0;
    public static double PINPOINT_Y_OFFSET_MM = 0.0;

    // =========================================================
    //                 SECTION 2: HARDWARE & STATE
    // =========================================================

    // Hardware Objects
    private DcMotorEx launcherMotor;
    private CRServo feeder;
    private Servo liftL, liftR;
    private VoltageSensor batteryVoltageSensor;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private Follower follower;

    // Launcher State
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private int autoShotsCount = 0;
    private ElapsedTime autoFireTimer = new ElapsedTime();
    private enum FireState { IDLE, FEEDING, WAITING }
    private FireState currentFireState = FireState.IDLE;

    // Vision State
    private boolean tagVisible = false;
    private double targetTx = 0;
    private double targetTy = 0;
    private double calculatedDistance = 0.0;
    private int tagID = -1;
    private String detectedMotif = "None";

    // Obelisk IDs
    private final int OBELISK_GPP = 21;
    private final int OBELISK_PGP = 22;
    private final int OBELISK_PPG = 23;

    // Drive State
    private boolean fieldCentric = true;
    private boolean rightBumperPressed = false;

    // =========================================================
    //                 SECTION 3: INITIALIZATION
    // =========================================================

    @Override
    public void init() {
        // 1. Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        // 2. Launcher Motor (V6 Physics: FLOAT)
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coast to stop
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD, LAUNCHER_kF));

        // 3. Feeder Servo
        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // 4. Lifts
        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL.setPosition(LIFT_L_HOME);
        liftR.setPosition(LIFT_R_HOME);

        // 5. Sensors
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        } catch (Exception e) {
            telemetry.addLine("WARNING: Pinpoint not found!");
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 6. Limelight (Pipeline 0)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Initialized: V6 (Cleaned & Hybrid)");
        telemetry.update();
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
        // 1. Read Sensors
        double voltComp = 12.0 / batteryVoltageSensor.getVoltage();
        updateVisionLogic();

        // 2. Update Systems
        handleDrive();              // Movement
        handleFlywheel(voltComp);   // RPM Control & Manual Override
        handleFeederAndFire();      // Firing Logic
        handleLifts();              // Aux Servos

        // 3. Pedro Update
        follower.update();

        // 4. Feedback
        updateTelemetry();
    }

    // =========================================================
    //                 SECTION 5: SYSTEM HANDLERS
    // =========================================================

    /**
     * Handles Robot Movement, Field Centric Toggles, and Auto-Alignment
     */
    private void handleDrive() {
        // Toggle Field Centric (GP2 Right Bumper)
        if (gamepad2.right_bumper && !rightBumperPressed) fieldCentric = !fieldCentric;
        rightBumperPressed = gamepad2.right_bumper;

        // --- NEW BUTTON MAPPINGS ---

        // Reset Pose (GP2 Y) - New Mapping
        if (gamepad2.y) {
            follower.setStartingPose(new Pose(0,0,0));
        }

        // Sync Vision Pose (GP2 B) - New Mapping
        // Only works if the Limelight actually sees a tag
        if (gamepad2.b && tagVisible) {
            syncPedroToLimelight();
        }

        double fwd = -gamepad1.left_stick_y;
        double str = -gamepad1.left_stick_x;
        double rot = -gamepad1.right_stick_x;

        // Macro: Auto-Drive to Point (GP2 A)
        if (gamepad2.a) {
            follower.holdPoint(DRIVE_TARGET);
        }
        // Macro: Auto-Align to Tag (GP1 B)
        else if (gamepad1.b && tagVisible) {
            double error = -targetTx;
            rot = Range.clip(error * AIM_kP, -MAX_AUTO_TURN_SPEED, MAX_AUTO_TURN_SPEED);
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
        // Manual Drive
        else {
            follower.setTeleOpDrive(fwd, str, rot, !fieldCentric);
        }
    }

    /**
     * Handles Flywheel PID, RPM selection, and V3 Manual Override
     */
    private void handleFlywheel(double voltComp) {
        double manualTrigger = gamepad1.left_trigger;

        // --- V3 Feature: Manual Analog Override ---
        // If trigger is pulled, ignore PID and send raw power
        if (manualTrigger > 0.1) {
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(Range.clip((manualTrigger / 0.8) * voltComp, 0, 1));
            currentTargetRPM = 0; // Clear target so telemetry shows manual
        }
        // --- Normal PID Control ---
        else {
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Select RPM Target
            if (gamepad1.left_bumper && tagVisible) {
                currentTargetRPM = getAutoRPM(calculatedDistance); // Auto Distance
            } else if (gamepad1.dpad_up)    currentTargetRPM = TARGET_RPM_UP;
            else if (gamepad1.dpad_right)   currentTargetRPM = TARGET_RPM_RIGHT;
            else if (gamepad1.dpad_down)    currentTargetRPM = TARGET_RPM_DOWN;
            else if (gamepad1.dpad_left)    currentTargetRPM = TARGET_RPM_LEFT;

            // Apply Velocity
            if (currentTargetRPM > 0) {
                launcherMotor.setVelocity((currentTargetRPM / 60.0) * FLWHEEL_TICKS_PER_REV);
            } else {
                launcherMotor.setPower(0); // V6 Physics: This will FLOAT (Coast)
            }
        }

        // Calculate current RPM for Telemetry
        currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
    }

    /**
     * Handles the Firing State Machine (V3 Time-Based Logic)
     */
    private void handleFeederAndFire() {
        // Trigger: Auto Drive & Fire Macro (GP2 A)
        if (gamepad2.a) {
            double distToTarget = Math.hypot(DRIVE_TARGET.getX() - follower.getPose().getX(),
                    DRIVE_TARGET.getY() - follower.getPose().getY());

            if (distToTarget < MACRO_ARRIVAL_TOLERANCE) {
                // Fire 3 shots if we are idle
                if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                    autoShotsCount = 3;
                    currentFireState = FireState.FEEDING;
                    autoFireTimer.reset();
                }
            }
        }
        // Trigger: Manual Single Shot (X)
        else if (gamepad1.x || gamepad2.x) {
            if (currentFireState == FireState.IDLE && autoShotsCount == 0) {
                autoShotsCount = 1;
                currentFireState = FireState.FEEDING;
                autoFireTimer.reset();
            }
        }

        // State Machine
        switch (currentFireState) {
            case FEEDING:
                feeder.setPower(1.0); // Push ring
                if (autoFireTimer.seconds() > FEED_PULSE_TIME) {
                    currentFireState = FireState.WAITING;
                    autoFireTimer.reset();
                }
                break;

            case WAITING:
                feeder.setPower(0); // Retract/Stop
                if (autoFireTimer.seconds() > MACRO_SHOT_DELAY) {
                    autoShotsCount--;
                    if (autoShotsCount > 0) {
                        currentFireState = FireState.FEEDING; // Fire next shot
                    } else {
                        currentFireState = FireState.IDLE;    // Done
                    }
                    autoFireTimer.reset();
                }
                break;

            case IDLE:
                feeder.setPower(0);
                break;
        }
    }

    private void handleLifts() {
        if (gamepad2.dpad_up) {
            liftL.setPosition(LIFT_L_ACTIVE);
            liftR.setPosition(LIFT_R_ACTIVE);
        } else if (gamepad2.dpad_down) {
            liftL.setPosition(LIFT_L_HOME);
            liftR.setPosition(LIFT_R_HOME);
        }
    }

    // =========================================================
    //                 SECTION 6: HELPERS & VISION
    // =========================================================

    private void updateVisionLogic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tagVisible = true;
            targetTx = result.getTx();
            targetTy = result.getTy();

            double angle = Math.toRadians(CAMERA_PITCH_DEG + targetTy);
            calculatedDistance = (TARGET_TAG_HEIGHT - CAMERA_HEIGHT_INCH) / Math.tan(angle);

            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (!fids.isEmpty()) {
                tagID = fids.get(0).getFiducialId();
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
                double x = (botpose.getPosition().x * 39.3701) + 72.0;
                double y = (botpose.getPosition().y * 39.3701) + 72.0;
                double h = Math.toRadians(botpose.getOrientation().getYaw());
                follower.setPose(new Pose(x, y, h));
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("--- SYSTEM ---", "");
        telemetry.addData("Drive Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Firing State", currentFireState.toString());

        telemetry.addData("--- VISION ---", "");
        if (tagVisible) {
            telemetry.addData("Target", "ID %d (%s)", tagID, detectedMotif);
            telemetry.addData("Distance", "%.1f in", calculatedDistance);
        } else {
            telemetry.addData("Target", "SEARCHING...");
        }

        telemetry.addData("--- LAUNCHER ---", "");
        if (gamepad1.left_trigger > 0.1) {
            telemetry.addData("Mode", "MANUAL OVERRIDE");
        } else {
            telemetry.addData("Mode", gamepad1.left_bumper ? "AUTO-RPM" : "PRESET");
            telemetry.addData("Target RPM", currentTargetRPM);
        }
        telemetry.addData("Actual RPM", "%.0f", currentFlywheelRPM);

        telemetry.addData("--- POSE ---", "");
        telemetry.addData("Loc", follower.getPose().toString());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}