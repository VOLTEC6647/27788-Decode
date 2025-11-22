package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class Limelight implements Subsystem {
    private Bot bot;
    private Limelight3A limelight;

    // Variables to hold data so MecanumDrive can read them
    private double tx = 0;
    private double ty = 0;
    private boolean hasTarget = false;

    public Limelight(Bot bot){
        this.bot = bot;
        limelight = bot.hMap.get(Limelight3A.class,"limelight");
        bot.telem.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;
            tx = result.getTx();
            ty = result.getTy();

            // Optional: Update Odometry if needed
            Pose3D botpose = result.getBotpose();
            // careful with this update, it might conflict with Pinpoint if not synchronized
            MecanumDrive.odo.setPosition(new Pose2D(DistanceUnit.METER, botpose.getPosition().x, botpose.getPosition().y, AngleUnit.RADIANS, botpose.getOrientation().getYaw(AngleUnit.RADIANS)));

            bot.telem.addData("tx", tx);
            bot.telem.addData("ty", ty);
        } else {
            hasTarget = false;
            tx = 0;
            ty = 0;
        }
    }

    // --- NEW METHODS ---

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTurnPower() {
        if (!hasTarget) return 0;

        // P-Controller: Adjust kP to change how fast it turns
        // If it shakes, lower kP (e.g., 0.01)
        // If it's too slow, raise kP (e.g., 0.05)
        double kP = 0.03;

        // Important: You might need to change the sign (-tx or +tx)
        // depending on your motor directions.
        return -tx * kP;
    }
}