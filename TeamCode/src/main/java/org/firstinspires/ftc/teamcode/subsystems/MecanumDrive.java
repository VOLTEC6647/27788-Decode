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
import org.firstinspires.ftc.teamcode.Vision.Limelight; // Import your custom class
import org.firstinspires.ftc.teamcode.pedropathing.constants;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;

    // 1. Define the object variable
    public Limelight limelight;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static Follower follower;
    public static boolean fieldCentric = true;
    public static Pose pose;
    private boolean isEncoderMode = false;

    public Rotation2d getRobotOrientation() {
        return new Rotation2d(0);
    }

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        // 2. Initialize the Limelight object
        limelight = new Limelight(bot);
        // Register it so its periodic() loop runs automatically
        this.register();
        limelight.register();

        follower = constants.createFollower(bot.hMap);

        if (pose == null) {
            pose = new Pose(0, 0, 0);
        }

        follower.setPose(new Pose(0, 0, 0));

        frontLeft = bot.hMap.get(DcMotorEx.class, "frontleft");
        frontRight = bot.hMap.get(DcMotorEx.class, "backleft");
        backLeft = bot.hMap.get(DcMotorEx.class, "frontright");
        backRight = bot.hMap.get(DcMotorEx.class, "backright");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD );
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // 3. Use 'limelight' (lowercase), which is the object, not the class
        bot.telem.addData("Limelight Turn", limelight.getTurnPower());
        bot.telem.addData("Tx", limelight.getTx());
        bot.telem.addData("Ty", limelight.getTy());

        if (bot.driver.gamepad.left_bumper){
            // Pass the calculated turn power to the drive method
            teleopDrive(limelight.getTurnPower(), 1);
        }

        follower.update();
        pose = follower.getPose();

        bot.telem.addData("EncoderMode",isEncoderMode);
        bot.telem.addData("FieldCentric",fieldCentric);
        TelemetryPacket posePacket = new TelemetryPacket();
        // Convert Pedro units to inches: 1 Pedro unit = 0.5 inches
        posePacket.put("Pose x", follower.getPose().getX()-72 );
        posePacket.put("Pose y", follower.getPose().getY() -72);
        posePacket.put("Pose heading", follower.getPose().getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(posePacket);
    }

    public void teleopDrive(double rx, double multiplier) {
        double x = bot.driver.getLeftY() * multiplier;   // Drive X (strafe) now uses Joystick Y (up/down) - **POSITIVE** sign for correct direction
        double y = bot.driver.getLeftX() * multiplier;   // Drive Y (forward/back) now uses Joystick X (left/right) - Positive sign for now


        rx *= -1;

        double botHeading = pose.getHeading();


        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(  botHeading);


        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        double[] powers = {frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        double[] normalizedPowers = normalizeWheelSpeeds(powers);

        frontLeft.setPower(normalizedPowers[0]);
        frontRight.setPower(normalizedPowers[1]);
        backLeft.setPower(normalizedPowers[2]);
        backRight.setPower(normalizedPowers[3]);
    }

    public void resetEncoders() {
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double[] normalizeWheelSpeeds(double[] speeds) {
        if (largestAbsolute(speeds) > 1) {
            double max = largestAbsolute(speeds);
            for (int i = 0; i < speeds.length; i++){
                speeds[i] /= max;
            }
        }
        return speeds;
    }

    private double largestAbsolute(double[] arr) {
        double largestAbsolute = 0;
        for (double d : arr) {
            double absoluteValue = Math.abs(d);
            if (absoluteValue > largestAbsolute) {
                largestAbsolute = absoluteValue;
            }
        }
        return largestAbsolute;
    }
}