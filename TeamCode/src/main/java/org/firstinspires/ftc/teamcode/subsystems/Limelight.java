/*

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;

@Config
public class Limelight extends SubsystemBase {

    private final Limelight3A camera;
    @Getter private LLResult result;
    public static double TURN_P = 0.05;

    // Conversion constants
    private static final double METERS_TO_INCH = 39.37;
    private static final double INCH_TO_PEDRO = 1.0 / 0.5; // 1 pedro unit = 0.5 in

    private Bot bot;


    public Limelight(Bot bot) {
        this.bot = bot;
        camera = bot.hMap.get(Limelight3A.class, "limelight");

        pipeline(0);
        initializeCamera();

    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }
    public void pipeline(int switchPipeline){
        camera.pipelineSwitch(switchPipeline);
    }

    public double getTurnPower() {
        if (!result.isValid()) {
            return 0.0;
        }
        double tx = getTx();

        return -tx * TURN_P;
    }

    public double getTx() {
        if (result == null) {
            return 0;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return 0;
        }
        return result.getTy();
    }



    @Override
    public void periodic() {
        result = camera.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();

                // --- Limelight meters -> inches -> FTC Pedro units ---
                double xFTC = (botpose.getPosition().x * METERS_TO_INCH) * INCH_TO_PEDRO;
                double yFTC = (botpose.getPosition().y * METERS_TO_INCH) * INCH_TO_PEDRO;
                double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                // --- Build Pose in FTC coordinates, then convert to Pedro bottom-left ---
                Pose pedroPose = new Pose(xFTC, yFTC, headingRad, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                MecanumDrive.follower.setPose(pedroPose);

                bot.telem.addData("Turn Power", getTurnPower());
                bot.telem.addData("Ty", getTy());
                bot.telem.addData("Tx", getTx());
                bot.telem.addData("Bot pose", botpose.toString());

            }
        }
    }
}

}

 */
