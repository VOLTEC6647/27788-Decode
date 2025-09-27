package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;

    private IMU imu = null;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static boolean fieldCentric = true;

    public static Pose pose;

    private boolean isEncoderMode = false;
    public static Pose pose0 = new Pose(50, 72, Math.toRadians(0));
    public static Pose pose1 = new Pose(20, 32, Math.toRadians(180));
    public static Pose pose2 = new Pose(30, 82, Math.toRadians(90));





    private double targetHeadingRadians;
    public void autoAlign(){
        Pose[] poses = {pose0, pose1, pose2};


            double actualX = odo.getEncoderX();
            double actualY = odo.getEncoderY();
            YawPitchRollAngles orientation = bot.getImu().getRobotYawPitchRollAngles();

// 2. Extract the specific Yaw value from that object
//    You must also specify the angle unit (DEGREES or RADIANS).
//    Your pathing library almost certainly expects RADIANS.
            double actualHeading = orientation.getYaw(AngleUnit.RADIANS);

            double[] poseX = {poses[0].getX(), poses[1].getX(), poses[2].getX()};
            double[] poseY = {poses[0].getY(), poses[1].getY(), poses[2].getY()};

            int bestChoiceIndex = -1; // Declare outside the if/else blocks

            if (actualX > 72) {
                double bestChoiceX = Double.MAX_VALUE;
                // No need to declare bestChoiceIndex again
                for (int i = 0; i < 3; i++) {
                    double difference = actualX - poseX[i];
                    if (difference < bestChoiceX) {
                        bestChoiceX = poseX[i];
                        bestChoiceIndex = i;
                    }
                }
            }
            else {
                double bestChoiceY = Double.MAX_VALUE;
                // No need to declare bestChoiceIndex again
                for (int i = 0; i < 3; i++) {
                    double difference = actualY - poseY[i];
                    if (difference < bestChoiceY) {
                        bestChoiceY = poseY[i];
                        bestChoiceIndex = i;
                    }
                }
            }
            Pose actualPose = new Pose(actualX, actualY, actualHeading);


            Follower f = new Follower(bot.hMap, FConstants.class, LConstants.class);
            f.setPose(actualPose);
            f.setMaxPower(0.75);
            f.setPose(actualPose);
            bot.getImu().resetYaw();

            // Ensure a valid index was found before scheduling
            if (bestChoiceIndex != -1) {
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(actualPose),
                                                new Point(poses[bestChoiceIndex])
                                        )
                                )
                                .setLinearHeadingInterpolation(actualPose.getHeading(), poses[bestChoiceIndex].getHeading())
                                .build()
                        )
                ).schedule(); // Schedule the command to run
            }

    }

    public Rotation2d getRobotOrientation() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        odo = bot.hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-82.66924000028, 110.830759999962);
        odo.setEncoderResolution(8192 / (Math.PI * 35));
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        if (pose == null) {
            pose = new Pose(0,0,0);
        }

        odo.setPosition(pose);

        frontLeft = bot.hMap.get(DcMotorEx.class, "FL");
        frontRight = bot.hMap.get(DcMotorEx.class, "FR");
        backLeft = bot.hMap.get(DcMotorEx.class, "BL");
        backRight = bot.hMap.get(DcMotorEx.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);//
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
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


        odo.update();
        pose = odo.getPosition();

        bot.telem.addData("EncoderMode",isEncoderMode);
        bot.telem.addData("FieldCentric",fieldCentric);

        //bot.telem.addData("Pose",
        //        "X: " + pose.getX(DistanceUnit.MM) +
        //                ", Y: " + pose.getY(DistanceUnit.MM) +
        //                ", Heading: " + pose.getHeading(AngleUnit.DEGREES));

        //FtcDashboard.getInstance().setImageQuality();
    }


    public void teleopDrive(double rx, double multiplier) {

        double x = -bot.driver.getLeftX() * multiplier;
        double y = -bot.driver.getLeftY() * multiplier;


        rx *= -bot.rotMultiplier;



        if (!fieldCentric) {
            y *= 1.1; // counteract imperfect strafe
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double[] powers = {frontLeftPower, frontRightPower, backLeftPower, backRightPower};
            double[] normalizedPowers = normalizeWheelSpeeds(powers);

            frontLeft.setPower(normalizedPowers[0]);
            frontRight.setPower(normalizedPowers[1]);
            backLeft.setPower(normalizedPowers[2]);
            backRight.setPower(normalizedPowers[3]);

            return;
        }

        double botHeading = pose.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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