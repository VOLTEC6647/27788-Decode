package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedropathing.constants;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;
    public Limelight limelight;

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Pathing & Localization
    public static Follower follower;
    public static Pose pose;

    // State Variables
    public static boolean fieldCentric = true;

    // Constants for Tuning
    public static double STRAFE_MULTIPLIER = 1.1;

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        // 1. Initialize Vision
        limelight = new Limelight(bot);
        this.register();
        limelight.register();

        // 2. Initialize Follower
        if (pose == null) pose = new Pose(0, 0, 0);
        follower = constants.createFollower(bot.hMap);
        follower.setPose(pose);

        // 3. Hardware Map
        frontLeft = bot.hMap.get(DcMotorEx.class, "frontLeft");
        frontRight = bot.hMap.get(DcMotorEx.class, "frontRight");
        backLeft = bot.hMap.get(DcMotorEx.class, "backLeft");
        backRight = bot.hMap.get(DcMotorEx.class, "backRight");

        // 4. Motor Directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // 5. Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 6. Run Mode (Directly set)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        follower.update();
        pose = follower.getPose();

        bot.telem.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
        bot.telem.addData("Limelight Turn", limelight.getTurnPower());

        if (bot.driver.gamepad.left_bumper) {
            teleopDrive(limelight.getTurnPower(), 1.0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", pose.getX());
        packet.put("y", pose.getY());
        packet.put("heading", pose.getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * The Main Drive Method - mathematically corrected for zero errors.
     * @param rxInput - Rotation input (Right Stick X)
     * @param speedMultiplier - General speed governor (0.0 to 1.0)
     */
    public void teleopDrive(double rxInput, double speedMultiplier) {
        // 1. Read Inputs
        double y = -bot.driver.getLeftY(); // Forward/Backward
        double x = bot.driver.getLeftX() * STRAFE_MULTIPLIER; // Strafe
        double rx = rxInput; // Rotation

        // 2. Field Centric Math
        if (fieldCentric) {
            double botHeading = pose.getHeading();

            // Rotate the velocity vector
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            x = rotX;
            y = rotY;
        }

        // 3. Mecanum Kinematics (Normalization and Power Distribution)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the raw power, normalized by the denominator
        double flRawPower = (y + x + rx) / denominator;
        double blRawPower = (y - x + rx) / denominator;
        double frRawPower = (y - x - rx) / denominator;
        double brRawPower = (y + x - rx) / denominator;

        // 4. Apply Speed Multiplier and Set Power - FIX APPLIED HERE
        frontLeft.setPower(flRawPower * speedMultiplier);
        backLeft.setPower(blRawPower * speedMultiplier);
        frontRight.setPower(frRawPower * speedMultiplier);
        backRight.setPower(brRawPower * speedMultiplier);
    }

    public void resetEncoders() {
        // Stop and Reset
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set back to Run Without Encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Rotation2d getRobotOrientation() {
        return new Rotation2d(pose.getHeading());
    }
}