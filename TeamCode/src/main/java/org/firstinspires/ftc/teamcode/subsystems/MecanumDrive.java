
package org.firstinspires.ftc.teamcode.subsystems;
import static com.sun.tools.doclint.Entity.mu;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.utils.ImuGlobal;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;
    private ImuGlobal imu;


    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static boolean fieldCentric = true;

    public static Pose2D pose;

    private boolean isEncoderMode = false;
    //  private Vision vision;




    private double targetHeadingRadians;


    public Rotation2d getRobotOrientation() {
        return new Rotation2d(
               0);
    }

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        //vision = new Vision(bot);
        //vision.register();

        odo = bot.hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-82.66924000028, 110.830759999962, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (pose == null) {
            pose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);
        }

        odo.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0));

        frontLeft = bot.hMap.get(DcMotorEx.class, "frontleft");
        frontRight = bot.hMap.get(DcMotorEx.class, "backleft");
        backLeft = bot.hMap.get(DcMotorEx.class, "frontright");
        backRight = bot.hMap.get(DcMotorEx.class, "backright");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE );
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void periodic() {

        // bot.telem.addData("Turret", vision.getTurnPower());
        // bot.telem.addData("Ty", vision.getTy());
        // bot.telem.addData("Tx", vision.getTx());


        // if (bot.driver.gamepad.left_bumper){
        //        teleopDrive(vision.getTurnPower(), 1);

        // }



        odo.update();
        pose = odo.getPosition();

        bot.telem.addData("EncoderMode",isEncoderMode);
        bot.telem.addData("FieldCentric",fieldCentric);

        //bot.telem.addData("Pose",
        //        "X: " + pose.getX(DistanceUnit.MM) +
        //                ", Y: " + pose.getY(DistanceUnit.MM) +
        //                ", Heading: " + pose.getHeading(AngleUnit.DEGREES));
    }


    public void teleopDrive(double rx, double multiplier) {

        double x = -bot.driver.getLeftX() * multiplier;
        double y = -bot.driver.getLeftY() * multiplier;


        rx *= bot.rotMultiplier;


        double botHeading = pose.getHeading(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);

        rotX *= 1.1; // counteract imperfect strafe

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
