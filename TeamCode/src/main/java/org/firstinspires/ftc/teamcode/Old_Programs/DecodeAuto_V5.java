package org.firstinspires.ftc.teamcode.Old_Programs;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DECODE Auto: V5 (Shoot & Park)", group = "Autonomous")
public class DecodeAuto_V5 extends OpMode {

    private Follower follower;
    private LauncherSubsystem launcher;
    private Timer opModeTimer;

    private int pathState;
    private final int START = 0;
    private final int DRIVE_TO_SHOOT = 1;
    private final int SHOOTING = 2;
    private final int PARKING = 3;
    private final int DONE = 99;

    // Start: Back wall
    private final Pose startPose = new Pose(52, 9.2, Math.toRadians(90));
    // Shoot Pos: Aimed at goal
    private final Pose shootPose = new Pose(55, 20, Math.toRadians(113));
    // Park: Face 90 degrees
    private final Pose parkPose = new Pose(57, 32, Math.toRadians(90));

    private PathChain pathStartToShoot, pathPark;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        launcher = new LauncherSubsystem(hardwareMap);
        opModeTimer = new Timer();
        buildPaths();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(START);
    }

    @Override
    public void loop() {
        follower.update();
        launcher.update();
        autonomousLogic();
        telemetry.addData("State", pathState);
        telemetry.addData("Launcher", launcher.getStatus());
        telemetry.update();
    }

    private void autonomousLogic() {
        switch (pathState) {
            case START:
                follower.followPath(pathStartToShoot, true);
                setPathState(DRIVE_TO_SHOOT);
                break;
            case DRIVE_TO_SHOOT:
                if (!follower.isBusy()) {
                    launcher.fireShots();
                    setPathState(SHOOTING);
                }
                break;
            case SHOOTING:
                if (!launcher.isBusy()) {
                    follower.followPath(pathPark, true);
                    setPathState(PARKING);
                }
                break;
            case PARKING:
                if (!follower.isBusy()) setPathState(DONE);
                break;
            case DONE:
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        opModeTimer.resetTimer();
    }

    private void buildPaths() {
        pathStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        pathPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public static class LauncherSubsystem {
        private DcMotorEx launcherMotor;
        private CRServo feeder;
        private Servo liftL, liftR;
        private final double FLWHEEL_TICKS_PER_REV = 534.8;
        private final double TARGET_RPM = 300;
        private final double FEED_DURATION_MS = 3000;
        private Timer mechTimer = new Timer();
        private enum State { IDLE, SPINNING_UP, FEEDING }
        private State state = State.IDLE;

        public LauncherSubsystem(HardwareMap hardwareMap) {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0, 12));

            feeder = hardwareMap.get(CRServo.class, "feeder");
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);

            try {
                liftL = hardwareMap.get(Servo.class, "liftL");
                liftR = hardwareMap.get(Servo.class, "liftR");
                liftL.setPosition(0.0);
                liftR.setPosition(0.0);
            } catch (Exception e) {}
        }

        public void fireShots() {
            state = State.SPINNING_UP;
            mechTimer.resetTimer();
        }

        public boolean isBusy() { return state != State.IDLE; }
        public String getStatus() { return state.toString(); }

        public void update() {
            switch (state) {
                case IDLE:
                    launcherMotor.setPower(0);
                    feeder.setPower(0);
                    break;
                case SPINNING_UP:
                    feeder.setPower(0);
                    double vel = (TARGET_RPM / 60.0) * FLWHEEL_TICKS_PER_REV;
                    launcherMotor.setVelocity(vel);
                    if (mechTimer.getElapsedTime() > 1000) {
                        state = State.FEEDING;
                        mechTimer.resetTimer();
                    }
                    break;
                case FEEDING:
                    feeder.setPower(1.0);
                    if (mechTimer.getElapsedTime() > FEED_DURATION_MS) {
                        state = State.IDLE;
                        mechTimer.resetTimer();
                    }
                    break;
            }
        }
    }
}
