package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogic {

    // =========================================================
    //                HARDWARE & CONSTANTS
    // =========================================================

    // Hardware matches your LauncherSystemTest.java
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private CRServo intake;

    // --- Tuning Variables from your code ---
    public static double FLWHEEL_TICKS_PER_REV = 534.8;
    public static double LAUNCHER_kP = 50;
    public static double LAUNCHER_kI = 0;
    public static double LAUNCHER_kD = 0;
    public static double LAUNCHER_kF = 12;
    public static double FEED_TIME = 0.20; // Time to run feeder per shot

    // Target RPM (Using your "UP" target as default for auto)
    public static int TARGET_RPM = 300;
    public static int RPM_TOLERANCE = 15;

    // --- State Tracking ---
    private ElapsedTime stateTimer;
    private FlywheelState flywheelState;
    private int shotsRemaining = 0;

    // Delay between shots to let the feeder stop/reset
    private double shotDelayTime = 0.3;

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        FEEDING,      // Replaces "LAUNCH" (Gate Open)
        RESET_FEEDER  // Replaces "RESET_GATE" (Gate Close)
    }

    // =========================================================
    //                      INIT
    // =========================================================
    public void init(HardwareMap hardwareMap) {
        // --- Launcher Setup ---
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Apply your PIDF Coefficients
        PIDFCoefficients currentPIDF = new PIDFCoefficients(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD, LAUNCHER_kF);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentPIDF);

        // --- Loader/Feeder Setup ---
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE); // Preserving your reverse logic

        // --- Intake Setup ---
        intake = hardwareMap.get(CRServo.class, "intake");

        // Initial State
        stateTimer = new ElapsedTime();
        flywheelState = FlywheelState.IDLE;

        stopFeeders();
        launcherMotor.setPower(0);
    }

    // =========================================================
    //                      UPDATE LOOP
    // =========================================================
    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    // Start spinning up
                    setFlywheelRPM(TARGET_RPM);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                } else {
                    launcherMotor.setPower(0);
                    stopFeeders();
                }
                break;

            case SPIN_UP:
                // Check if flywheel is at speed using your tolerance logic
                if (isFlywheelAtSpeed() || stateTimer.seconds() > 2.0) { // 2.0s safety timeout
                    startFeeders(); // Start injecting the ring/pixel
                    stateTimer.reset();
                    flywheelState = FlywheelState.FEEDING;
                }
                break;

            case FEEDING:
                // Run feeders for FEED_TIME (0.20s)
                if (stateTimer.seconds() > FEED_TIME) {
                    shotsRemaining--;
                    stopFeeders();
                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET_FEEDER;
                }
                break;

            case RESET_FEEDER:
                // Wait briefly to ensure spacing between shots
                if (stateTimer.seconds() > shotDelayTime) {
                    if (shotsRemaining > 0) {
                        // Maintain speed and shoot again
                        setFlywheelRPM(TARGET_RPM);
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        // Done
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }

    // =========================================================
    //                    HELPER METHODS
    // =========================================================

    public void fireShots(int shots) {
        if (flywheelState == FlywheelState.IDLE) {
            this.shotsRemaining = shots;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }

    // Logic from handleFlywheel in LauncherSystemTest.java
    private void setFlywheelRPM(int rpm) {
        double velocity = (rpm / 60.0) * FLWHEEL_TICKS_PER_REV;
        launcherMotor.setVelocity(velocity);
    }

    // Logic from handleFlywheel in LauncherSystemTest.java
    private boolean isFlywheelAtSpeed() {
        double currentFlywheelRPM = (launcherMotor.getVelocity() / FLWHEEL_TICKS_PER_REV) * 60.0;
        return Math.abs(currentFlywheelRPM - TARGET_RPM) <= RPM_TOLERANCE;
    }

    // Logic from handleFeeder in LauncherSystemTest.java
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
}