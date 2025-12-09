

package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ReadWriteFile;

import lombok.Getter;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

import java.io.File;

@Config
public class Vision extends SubsystemBase {
    private final PIDFController pid = new PIDFController(0.2, 0, 0.0, 0.0);

    private final Limelight3A camera;
    @Getter private boolean isDataOld = false;
    @Getter private LLResult result;

    public static double CAMERA_HEIGHT = 10;
    public static double CAMERA_ANGLE = 45.0;
    public static double TURN_P = 0.02;
    public static double strafeConversionFactor = 6.6667;
    public static double cameraStrafeToBot = -20;

    private Bot bot;


    public Vision(Bot bot) {
        this.bot = bot;
        camera = bot.hMap.get(Limelight3A.class, "limelight");

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);

        if (team.equals("blue")){
            camera.pipelineSwitch(2);

        }
        if (team.equals("red")){
            camera.pipelineSwitch(1);
        }
    }

    public void initializeCamera() {
        camera.setPollRateHz(50);
        camera.start();
    }
    public double getTurnPower() {
        if (!isTargetVisible()) {
            return 0.0;
        }

        double tx = getTx(0.0);

        //double turnPower = pid.calculate(tx);
        double turnPower = tx * TURN_P;

        return turnPower;
    }

    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtils.isNear(0, result.getTa(), 0.0001);
    }

    public double getDistance() {
        double ty = getTy(0.0);
        if (MathUtils.isNear(0, ty, 0.01)) {
            return 0;
        }
        double distance = (CAMERA_HEIGHT * Math.tan(CAMERA_ANGLE + ty));
        return Math.abs(distance);
    }

    // Get the strafe
    public double getStrafeOffset() {
        double tx = getTx(0);
        if (tx != 0) {
            return tx * strafeConversionFactor - cameraStrafeToBot;
        }
        return 0;
    }


    public Double getTurnServoDegree() {
        if (result == null) {
            return null;
        }
        return result.getPythonOutput()[3];
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();

        if (result != null) {

            long staleness = result.getStaleness();
            isDataOld = staleness >= 100;
            bot.telem.addData("Distance", getDistance());
            bot.telem.addData("Angle", getTurnServoDegree());

        }
    }
}
