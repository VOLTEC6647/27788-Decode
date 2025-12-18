
package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {

    private DcMotorEx Shooter;
    @Config
    public static class ShooterPIDF{
        public static double kp = 60;
        public static double ki = 0;
        public static double kd = 0;
        public static double kf = 0;
    }
    public static double targetVelocity = 1000;
    private Bot bot;


    public Shooter(Bot bot) {
        this.bot = bot;

        Shooter = bot.hMap.get(DcMotorEx.class, "Shooter");


        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        Shooter.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );


    }

    @Override
    public void periodic(){
        double currentVelocity = Shooter.getVelocity();

        bot.telem.addData("Target Velocity", targetVelocity);
        bot.telem.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(double velocity){
        Shooter.setVelocity(velocity);



    }
}
