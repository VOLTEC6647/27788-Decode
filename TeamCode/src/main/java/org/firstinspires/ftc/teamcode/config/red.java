package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@Autonomous(group = "config")
public class red extends LinearOpMode {
    @Override
    public void runOpMode() {
        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        ReadWriteFile.writeFile(myFileName, "red");
    }
}