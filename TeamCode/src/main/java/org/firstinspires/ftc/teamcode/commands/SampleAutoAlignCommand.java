package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.autos.adelanteyatras.startingPose;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class SampleAutoAlignCommand extends CommandBase {
    private final MecanumDrive drive;
    private final Vision vision;
    private final Telemetry telemetry;
    private final Pose pose;
    private final Bot bot;
    Follower f = new Follower(hardwareMap, FConstants.class, LConstants.class);


    private final double tickPerUnit = 422 / (440 / 25.4); // tick per inches
    private boolean isTargetVisibleWhenStart = true;

    @Getter private boolean isInitializing = true;

    public SampleAutoAlignCommand(
            MecanumDrive drive,
            Vision vision,
            Telemetry telemetry, Pose pose, Bot bot
            ) {
        this.drive = drive;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pose = pose;
        this.bot = bot;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        isInitializing = true;

        isTargetVisibleWhenStart = vision.isTargetVisible();


        telemetry.addData("isVisibleWhenStart", isTargetVisibleWhenStart);

        double currentPoseRelativeToFieldY = pose.getY();
        double currentPoseRelativeToFieldX = pose.getX();
        double currentPoseRelativeToFieldHeading = pose.getHeading();

        Pose currentPoseRelativeToField = new Pose(currentPoseRelativeToFieldX, currentPoseRelativeToFieldY, currentPoseRelativeToFieldHeading);


        bot.telem.addData("Current Pose", currentPoseRelativeToField);



        double distanceOffset = vision.getDistance() ; // inches
        double slideExtensionValue = 0;

        if (distanceOffset > 0) {
            slideExtensionValue = distanceOffset * tickPerUnit;
            distanceOffset = 0;
        }

        telemetry.addData("Slide Extension Value Auto", slideExtensionValue);

        Pose targetPoseRelativeToRobot =
                new Pose(distanceOffset, -vision.getStrafeOffset() / 25.4, 0);
        telemetry.addData("Target Robot Pose", targetPoseRelativeToRobot);

        // Transform target pose from robot-relative to field-relative coordinates
        double fieldX =
                currentPoseRelativeToFieldX
                        + (targetPoseRelativeToRobot.getX() * Math.cos(currentPoseRelativeToFieldHeading)
                        - targetPoseRelativeToRobot.getY()
                        * Math.sin(currentPoseRelativeToFieldHeading));
        double fieldY =
                currentPoseRelativeToFieldY
                        + (targetPoseRelativeToRobot.getX() * Math.sin(currentPoseRelativeToFieldHeading)
                        + targetPoseRelativeToRobot.getY()
                        * Math.cos(currentPoseRelativeToFieldHeading));
        double fieldHeading =
                currentPoseRelativeToFieldHeading + 0; // We move our claw instead of robot heading

        Pose targetPoseRelativeToField = new Pose(fieldX, fieldY,fieldHeading);


        telemetry.addData("Target Field Pose", targetPoseRelativeToField);
        if (isTargetVisibleWhenStart) {
            new FollowPathCommand(f, f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(currentPoseRelativeToField),
                                    new Point(targetPoseRelativeToField)))
                    .setLinearHeadingInterpolation(
                            currentPoseRelativeToField.getHeading(),
                            targetPoseRelativeToField.getHeading())
                    .build()
            );


        } else {
            cancel();
        }

    }



    @Override
    public void execute() {
        isInitializing = false;
    }

    @Override
    public void end(boolean interrupted) {
        isTargetVisibleWhenStart = true;
    }


}
