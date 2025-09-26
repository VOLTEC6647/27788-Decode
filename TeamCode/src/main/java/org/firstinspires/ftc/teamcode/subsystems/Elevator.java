package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

public class Elevator implements AnyArm {

    private double pidOutput;

    @Config
    public static class ArmpidConstants{
        public static double kp = 0.05;
        static double ki = 0;
        static double kd = 0;
        public static double kf = -0.0005;
    }
    private final PIDFController pid = new PIDFController(0, 0, 0.0, 0.0);
    private Bot bot;
    public int setPoint = 0;
    public double currentSetpoint = 0;
    public int currentPosition = 0;

    public double speed = 15;
    public DcMotor motorRotate;
    public DcMotor motor2;

    public int jitTicks = 0;
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Config
    public static class ArmSetpoints{
        public static int climb = 1;
        public static int save = 0;
    }

    public Elevator(Bot bot) {
        this.bot = bot;

        pid.setTolerance(20);

        motorRotate = bot.hMap.get(DcMotor.class, "motor1");
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = bot.hMap.get(DcMotor.class, "motor2");

        dashboard.sendTelemetryPacket(packet);
    }

    public void resetEncoders(){
        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void disableperiodictwo (){
        bot.telem.addData("ArmRotateRight",motorRotate.getCurrentPosition());

    }
    public void setSetpoint (int setPoint) {
        this.setPoint = setPoint;
    }


    @Override
    public void periodic() {
        jitTicks--;

        currentPosition = motorRotate.getCurrentPosition();


        if(bot.opertator.getButton(GamepadKeys.Button.A)){
            setPoint = ArmSetpoints.save;
        }
        if(bot.opertator.getButton(GamepadKeys.Button.Y)){
            setPoint = ArmSetpoints.climb;
            jitTicks = 0;
        }

        pid.setP(ArmpidConstants.kp);
        pid.setI(ArmpidConstants.ki);

        if(setPoint == ArmSetpoints.climb || setPoint == ArmSetpoints.save){
            pid.setF(ArmpidConstants.kf);
        }
        double manualAdjustmentIncrement = 3.0;

        if (bot.opertator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            currentSetpoint -= manualAdjustmentIncrement;
            setPoint = (int) Math.round(currentSetpoint);
        }

        if (bot.opertator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            currentSetpoint += manualAdjustmentIncrement;
            setPoint = (int) Math.round(currentSetpoint);
        }
        if(Math.abs(currentSetpoint-setPoint)<speed){
            currentSetpoint = (int) setPoint;
        }

        if(currentSetpoint > setPoint){
            currentSetpoint = currentSetpoint - speed;
        }

        if(currentSetpoint < setPoint){
            currentSetpoint = currentSetpoint + speed;
        }



        double pidOutput = pid.calculate(motorRotate.getCurrentPosition(), currentSetpoint);

        if(jitTicks > 0){
            pidOutput += 0.84;
        }

        if (pidOutput > 1){
            pidOutput = 0.84;
        }

        if (pidOutput < -1){
            pidOutput = -0.84;
        }

        motorRotate.setPower(pidOutput);
        motor2.setPower(pidOutput);

        bot.telem.addData("ArmSetpoint", currentSetpoint);
        bot.telem.addData("ArmPosition", motorRotate.getCurrentPosition());
        bot.telem.addData("ArmOutput", pidOutput);

    }

    @Override
    public DcMotor getMotorRotate() {
        return motorRotate;
    }

    public void setPosition(int setPoint){
        this.setPoint = setPoint;
    }
}