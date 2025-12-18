package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Limelight limelight;

    public static Follower follower;
    public static Pose pose;

    public static boolean fieldCentric = true;
    public static double STRAFE_MULTIPLIER = 1.1;

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        limelight = new Limelight(bot);
        this.register();

        frontLeft = bot.hMap.get(DcMotorEx.class, "frontLeft");
        frontRight = bot.hMap.get(DcMotorEx.class, "frontRight");
        backLeft = bot.hMap.get(DcMotorEx.class, "backLeft");
        backRight = bot.hMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE );

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (pose == null) pose = new Pose(0, 0, 0);
        follower = constants.createFollower(bot.hMap);
        follower.setPose(pose);
        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();
        pose = follower.getPose();
        bot.telem.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
    }

    // THIS METHOD WAS MISSING - Adding it fixes the TeleopDriveCommand errors
    public void teleopDrive(double xInput, double yInput, double rxInput, double speedMultiplier) {
        double x = xInput * STRAFE_MULTIPLIER;
        double y = yInput;
        double rx = rxInput;

        if (fieldCentric && pose != null) {
            double botHeading = -pose.getHeading();
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeft.setPower(((y + x + rx) / denominator) * speedMultiplier);
        backLeft.setPower(((y - x + rx) / denominator) * speedMultiplier);
        frontRight.setPower(((y - x - rx) / denominator) * speedMultiplier);
        backRight.setPower(((y + x - rx) / denominator) * speedMultiplier);
    }
}