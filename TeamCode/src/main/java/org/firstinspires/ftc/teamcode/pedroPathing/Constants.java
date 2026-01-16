package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // =================================================================
    // 1. FOLLOWER CONSTANTS (Physics & Control)
    // =================================================================
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5) // TODO: TUNE THIS! (Robot Weight in KG)
            .centripetalScaling(0.0005)

            // --- Automatic Tuners ---
            .forwardZeroPowerAcceleration(-30.0) // TODO: TUNE THIS!
            .lateralZeroPowerAcceleration(-30.0)

            // --- PIDF Coefficients ---
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.05, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.0005, 0.6, 0));

    // =================================================================
    // 2. DRIVETRAIN CONSTANTS (Motors & Velocities)
    // =================================================================
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(60.0)
            .yVelocity(45.0);

    // =================================================================
    // 3. LOCALIZER CONSTANTS (Pinpoint Odometry)
    // =================================================================
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)

            // TODO: UPDATE THESE WITH REAL MEASUREMENTS
            .forwardPodY(-5.0)  // Inches from center (Left = +, Right = -)
            .strafePodX(0.5);   // Inches from center (Forward = +, Back = -)

    // =================================================================
    // 4. PATH CONSTRAINTS
    // =================================================================
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100.0, 1.0, 1.0
    );

    // =================================================================
    // 5. BUILDER
    // =================================================================
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}