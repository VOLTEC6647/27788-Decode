package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;


public class Limelight implements Subsystem {
    private Bot bot;
    private Limelight3A limelight;

    public Limelight(Bot bot){
        this.bot = bot;

        limelight = bot.hMap.get(Limelight3A.class,"limelight");
        bot.telem.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();
    }

    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                MecanumDrive.odo.setPosition(new Pose(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));

                bot.telem.addData("tx", result.getTx());
                bot.telem.addData("ty", result.getTy());
                bot.telem.addData("Botpose", botpose.toString());
            }
        }
    }
}
