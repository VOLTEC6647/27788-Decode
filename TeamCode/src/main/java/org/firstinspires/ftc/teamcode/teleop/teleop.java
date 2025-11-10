package org.firstinspires.ftc.teamcode.teleop;

import static com.arcrobotics.ftclib.kotlin.extensions.gamepad.GamepadExExtKt.whileActiveOnce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveNoDriverView;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.io.File;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private MecanumDriveNoDriverView drivenoview;
    private ShooterPivot shooterPivot;
    private ClawPivot clawPivot;
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    private Limelight limelight;
    boolean SPECIMENMODE = false;






    public void initialize() {

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();



        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // drive region

         bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);
        bot.getImu().resetYaw();

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);


        if (team.equals("blue")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        };
        if (team.equals("red")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }

        drive = new MecanumDrive(bot);
        drive.register();
       //vision = new Vision(bot);
        // vision.register();
        limelight = new Limelight(bot);
        limelight.register();

        drive = new MecanumDrive(bot);
        drive.register();

        intake = new Intake(bot);
        intake.register();

        shooter = new Shooter(bot);
        shooter.register();
        /*

        dClaw = new DiffClaw(bot);
        dClaw.register();

        shooterPivot = new ShooterPivot(bot);
        shooterPivot.register();

        clawPivot = new ClawPivot(bot);
        clawPivot.register();

        arm = new Arm(bot);
        arm.register();

        diffClawUp = new DiffClawUp(bot);
        diffClawUp.register();

        clawUp = new ClawUp(bot);
        clawUp.register();
*/
        //intake = new Intake(bot);
        //intake.register();


     TeleopDriveCommand driveCommand = new TeleopDriveCommand(
                drive,
                () -> -driverGamepad.getRightX()*.80,
                () -> -driverGamepad.getLeftY()*1,
                () -> driverGamepad.getLeftX()*1,
                () -> bot.speed
        );
        bot.speed = 0.75;

        register(drive);
        drive.setDefaultCommand(driveCommand);

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> shooter.setVelocity()),
                                new InstantCommand(()-> intake.setPower(1))
                        )
                );
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> shooter.setVelocity()),
                                new InstantCommand(()-> intake.setPower(0))

                        )
                );


        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileActiveOnce(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> drive.odo.resetPosAndIMU())

                    )
                );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileActiveOnce(
                        new InstantCommand(() -> {drive.autoAlign();})
                );






        while (opModeInInit()){
            telem.update();
        }
        //endregion


    }
    @Override
    public void run() {
        //periodicBindings();
        CommandScheduler.getInstance().run();
        telem.update();
    }

}