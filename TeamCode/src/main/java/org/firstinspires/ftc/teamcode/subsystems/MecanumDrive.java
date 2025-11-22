package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Limelight; // Import your custom class
import org.firstinspires.ftc.teamcode.utils.ImuGlobal;

import java.net.PortUnreachableException;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;

    // 1. Define the object variable
    public Limelight limelight;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static boolean fieldCentric = true;
    public static Pose2D pose;
    public static Rotation2d rotation2D;
    private boolean isEncoderMode = false;
    public Rotation2d getRobotOrientation() {
        return new Rotation2d(0);

    }

    public static double kP_Heading = 1;
    public static double HEADING_TOLERANCE = Math.toRadians(2); // 2 degrees

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        // 2. Initialize the Limelight object
        limelight = new Limelight(bot);
        // Register it so its periodic() loop runs automatically
        this.register();
        limelight.register();

        odo = bot.hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-82.66924000028, 110.830759999962, DistanceUnit.MM); // Fixed Unit to MM based on typical offset size
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (pose == null) {
            pose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);
        }

        odo.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0));

        frontLeft = bot.hMap.get(DcMotorEx.class, "frontleft");
        frontRight = bot.hMap.get(DcMotorEx.class, "backleft"); // Double check your config names here
        backLeft = bot.hMap.get(DcMotorEx.class, "frontright");
        backRight = bot.hMap.get(DcMotorEx.class, "backright");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        odo.update();
        pose = odo.getPosition();

        // Telemetry
        bot.telem.addData("Limelight Turn", limelight.getTurnPower());
        bot.telem.addData("Tx", limelight.getTx());
        bot.telem.addData("Ty", limelight.getTy());
        bot.telem.addData("EncoderMode",isEncoderMode);
        bot.telem.addData("FieldCentric",fieldCentric);
        bot.telem.addData("Current Pose", "X: %.2f, Y: %.2f, H: %.2f", pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.DEGREES));


        if (bot.driver.gamepad.left_bumper){
            // limelight auto align
            double x = -bot.driver.getLeftX();
            double y = -bot.driver.getLeftY();
            drive(x, y, limelight.getTurnPower());
        } else if (bot.driver.gamepad.right_bumper) {
            autoRotateToPoint();
        } else {
            teleopDrive(1.0);
        }
    }

    public void drive(double x, double y, double rx) {
        rx *= bot.rotMultiplier;

        double botHeading = pose.getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction by the bot's angle
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
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

    public void teleopDrive(double multiplier) {
        double x = - bot.driver.getLeftX() * multiplier;
        double y =  -bot.driver.getLeftY() * multiplier;
        double rx = -bot.driver.getRightX() * multiplier;
        drive(x, y, rx);
    }

    public void autoRotateToPoint() {
        // Target
        double targetX = 0;
        double targetY = 0;

        // Calculate the error
        double xError = targetX - pose.getX(DistanceUnit.METER);
        double yError = targetY - pose.getY(DistanceUnit.METER);

        // Calculate desired heading to face the target
        double targetHeading = Math.atan2(yError, xError);
        double headingError = AngleUnit.normalizeRadians(targetHeading - pose.getHeading(AngleUnit.RADIANS));

        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            drive(0, 0, 0); // Stop the robot
            return;
        }

        // P-controllers
        double xPower = 0; // Only rotate, no translational movement
        double yPower = 0; // Only rotate, no translational movement
        double headingPower = headingError * kP_Heading;

        drive(xPower, yPower, headingPower);

        // Telemetry
        bot.telem.addData("Target Heading", "H: %.2f", Math.toDegrees(targetHeading));
        bot.telem.addData("Current Heading", "H: %.2f", pose.getHeading(AngleUnit.DEGREES));
        bot.telem.addData("Heading Error", "H: %.2f", Math.toDegrees(headingError));
        bot.telem.addData("Power", "H: %.2f", headingPower);
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