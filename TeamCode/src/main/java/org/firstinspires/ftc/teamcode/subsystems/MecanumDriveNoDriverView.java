package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// IMU import is not directly used in this subsystem's methods but Bot might need it.
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Bot;

public class MecanumDriveNoDriverView implements Subsystem {
    private DcMotor fr; // e.g., "Front" motor or "M0"
    private DcMotor fl; // e.g., "LeftRear" motor or "M1"
    private DcMotor br; // e.g., "RightRear" motor or "M2"
    private DcMotor bl;

    public Bot bot;

    // Constants for Kiwi drive kinematics
    private static final double COS_0 = 1.0; // cos(0 deg)
    private static final double SIN_0 = 0.0; // sin(0 deg)
    private static final double COS_120 = -0.5; // cos(120 deg)
    private static final double SIN_120 = Math.sqrt(3.0) / 2.0; // sin(120 deg) approx 0.866
    private static final double COS_240 = -0.5; // cos(240 deg)
    private static final double SIN_240 = -Math.sqrt(3.0) / 2.0; // sin(240 deg) approx -0.866

    public Rotation2d getRobotOrientation() {
        return bot.getYaw();
    }

    public MecanumDriveNoDriverView(Bot bot) {
        // Ensure your hardware configuration matches these names: "M0", "M1", "M2"
        // Or adjust these names to match your configuration (e.g., "frontMotor", "leftMotor", "rightMotor")
        fr = bot.hMap.get(DcMotor.class, "fr");
        fl = bot.hMap.get(DcMotor.class, "fl");
        br = bot.hMap.get(DcMotor.class, "br");
        bl = bot.hMap.get(DcMotor.class, "bl");

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        // Motor directions: This is CRITICAL and depends on your physical build.
        // For the kinematics math used, if positive power to a motor means it spins
        // to contribute positively to its designated angle (0, 120, 240 deg),
        // then all should be FORWARD. If a motor is mounted "backwards" or geared
        // differently, you might need to REVERSE it.
        // Test rotation: if positive rot makes it spin CCW instead of CW (or vice versa),
        // you might need to reverse all motors or flip the sign of rx_joystick.
        // Start with all FORWARD and adjust based on testing.
        fr.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust if needed
        fl.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed
        br.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust if needed
        bl.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        this.bot = bot;
    }

    public void teleopDrive(double x, double y, double rot) {/*
        if(bot.opertator.getButton(GamepadKeys.Button.DPAD_UP)){
            forward();
        }
        if(bot.opertator.getButton(GamepadKeys.Button.DPAD_DOWN)){
            downward();
        }
        if(bot.opertator.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            right();
        }
        if(bot.opertator.getButton(GamepadKeys.Button.DPAD_LEFT)){
            left();
        }*/
        double Y = -bot.opertator.getLeftY();
        double X = bot.opertator.getLeftX();
        double R = -bot.opertator.getRightX();
        fl.setPower(Y+X+R);
        fr.setPower(Y-X-R);
        bl.setPower(Y-X+R);
        br.setPower(Y+X-R);

    }

    public DcMotor getFr() {
        return fr;
    }

    public DcMotor getFl() {
        return fl;
    }

    public DcMotor getBr() {
        return br;
    }
    public DcMotor getBl() {
        return bl;
    }

    @Override
    public void periodic() {
        bot.telem.addData("M0 Enc", fr.getCurrentPosition());
        bot.telem.addData("M1 Enc", fl.getCurrentPosition());
        bot.telem.addData("M2 Enc", br.getCurrentPosition());
        bot.telem.addData("M3 Enc", bl.getCurrentPosition());
        bot.telem.addData("M0 Pwr", fr.getPower());
        bot.telem.addData("M1 Pwr", fl.getPower());
        bot.telem.addData("M2 Pwr", br.getPower());
        bot.telem.addData("M3 Pwr", bl.getPower());
        bot.telem.addData("Orientation", getRobotOrientation().getDegrees());
        if (bot.driver.getButton(GamepadKeys.Button.BACK)) {
            bot.getImu().resetYaw();
        }
    }

    public void resetEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBrakeMode(DcMotor.ZeroPowerBehavior mode) {
        fr.setZeroPowerBehavior(mode);
        fl.setZeroPowerBehavior(mode);
        br.setZeroPowerBehavior(mode);
        bl.setZeroPowerBehavior(mode);
    }

    public void stop() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void forward() {
        fr.setPower(0.5);
        fl.setPower(0.5);
        br.setPower(0.5);
        bl.setPower(0.5);
    }
    public void downward() {
        fr.setPower(-0.5);
        fl.setPower(-0.5);
        br.setPower(-0.5);
        bl.setPower(-0.5);
    }
    public void right() {
        fr.setPower(-0.5);
        fl.setPower(0.5);
        br.setPower(0.5);
        bl.setPower(-0.5);
    }
    public void left() {
        fr.setPower(0.5);
        fl.setPower(-0.5);
        br.setPower(-0.5);
        bl.setPower(0.5);
    }

}