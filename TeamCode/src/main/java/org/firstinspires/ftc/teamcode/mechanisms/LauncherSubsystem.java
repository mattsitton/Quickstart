package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.util.Timer;

public class LauncherSubsystem { // <--- The file name MUST match "LauncherSubsystem.java"

    // Hardware
    private DcMotorEx launcherMotor;
    private CRServo feeder;

    // Tuning
    private final double FLWHEEL_TICKS_PER_REV = 534.8;
    private final double TARGET_RPM = 300;
    private final double RPM_TOLERANCE = 15.0; // +/- 15 RPM
    private final double FEED_DURATION_MS = 1000; // How long to push the ring

    // State Machine
    private Timer mechTimer = new Timer();
    private enum State { IDLE, WAIT_FOR_SPEED, FIRE_PULSE }
    private State state = State.IDLE;

    private int shotsToFire = 0;
    private int shotsFiredCount = 0;
    private double currentRPM = 0;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        // Initialize Hardware
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- FIX APPLIED HERE ---
        // Changed to FLOAT so the flywheel coasts instead of locking up
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(50, 0, 0, 12));

        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Call this to start the firing sequence.
     * @param count Number of shots to fire (e.g., 3)
     */
    public void fireShots(int count) {
        if (state == State.IDLE) {
            shotsToFire = count;
            shotsFiredCount = 0;
            state = State.WAIT_FOR_SPEED;
        }
    }

    /**
     * Call this in your loop() every cycle.
     */
    public void update() {
        // 1. Calculate RPM
        double velocity = launcherMotor.getVelocity();
        currentRPM = (velocity / FLWHEEL_TICKS_PER_REV) * 60.0;

        // 2. Handle Idle State
        if (state == State.IDLE) {
            launcherMotor.setPower(0);
            feeder.setPower(0);
            return;
        }

        // 3. Maintain Target Speed (Run motor during entire sequence)
        double targetVel = (TARGET_RPM / 60.0) * FLWHEEL_TICKS_PER_REV;
        launcherMotor.setVelocity(targetVel);

        // 4. State Machine Logic
        switch (state) {
            case WAIT_FOR_SPEED:
                feeder.setPower(0);
                // Only fire if we are within tolerance
                if (Math.abs(currentRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                    state = State.FIRE_PULSE;
                    mechTimer.resetTimer();
                }
                break;

            case FIRE_PULSE:
                feeder.setPower(1.0);
                // Run feeder for exactly 1.0 second
                if (mechTimer.getElapsedTime() > FEED_DURATION_MS) {
                    shotsFiredCount++;
                    if (shotsFiredCount >= shotsToFire) {
                        state = State.IDLE; // Sequence Complete
                    } else {
                        state = State.WAIT_FOR_SPEED; // Wait for recovery
                    }
                }
                break;
        }
    }

    // --- Info Methods ---
    public boolean isBusy() {
        return state != State.IDLE;
    }

    public String getStatus() {
        return state.toString() + " (" + shotsFiredCount + "/" + shotsToFire + ")";
    }

    public double getCurrentRPM() {
        return currentRPM;
    }
}