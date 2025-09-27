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
        drive.register();/*

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

/*


        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.setPosition(armSaved)),
                                new InstantCommand(()-> diffClawUp.setPositionD(savedForPickUp)),
                                new InstantCommand(()-> diffClawUp.setPositionI(savedForPickUp))

                        )
                );
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> clawPivot.setPosition(savedPivot)),
                                new InstantCommand(()-> dClaw.setPositionD(savedForSpecimen)),
                                new InstantCommand(()-> dClaw.setPositionI(savedForSpecimen))

                        )
                );


        //CYCLES


        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> clawPivot.setPosition(grabPivot))
                                //new InstantCommand(()-> dClaw.setPositionD(grab-(0*intakeRotatePerDegree))),
                                //new InstantCommand(()-> dClaw.setPositionI(grab+(0*intakeRotatePerDegree)))
                        )
                );



        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> clawPivot.setPosition(cloudPivot))
                        )
                );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> clawPivot.setPosition(0.5))
                        )
                );



        // SPECIMEN


            new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> clawUp.setPosition(outakeClose)),
                                new WaitCommand(500),
                                new InstantCommand(()-> arm.setPosition(armScore)),
                                new WaitCommand(600),
                                new InstantCommand(()-> diffClawUp.setPositionD(placeDiff-(45*outTakeRotatePerDegree))),
                                new InstantCommand(()-> diffClawUp.setPositionI(placeDiff+(45*outTakeRotatePerDegree)))
                )
        );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whileActiveContinuous(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> diffClawUp.setPositionD(
                                        bot.opertator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                                                - bot.opertator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                                                + placeDiff - (65 * outTakeRotatePerDegree))),
                                new InstantCommand(() -> diffClawUp.setPositionI(
                                        bot.opertator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                                                - bot.opertator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                                                + placeDiff + (65 * outTakeRotatePerDegree)))
                        )
                );   // <--- semicolon instead of comma

        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenInactive(   // or .onFalse(...) depending on your FTCLib version
                        new SequentialCommandGroup(
                                new InstantCommand(() -> arm.setPosition(armAfterScore)),
                                new InstantCommand(() -> diffClawUp.setPositionD(afterScore - (65 * outTakeRotatePerDegree))),
                                new InstantCommand(() -> diffClawUp.setPositionI(afterScore + (65 * outTakeRotatePerDegree))),
                                new WaitCommand(800),
                                new InstantCommand(() -> clawUp.setPosition(outakeOpen))
                        )
                );   // <--- semicolon instead of comma



        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> arm.setPosition(armGrabWall)),
                                new InstantCommand(() -> clawUp.setSetpoint(outakeOpen)),
                                new WaitCommand(50),
                                new InstantCommand(() -> diffClawUp.setPositionD(grabWall - (0 * outTakeRotatePerDegree))),
                                new InstantCommand(() -> diffClawUp.setPositionI(grabWall + (0 * outTakeRotatePerDegree)))
                        )
                );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> dClaw.setPositionD(grab - (45 * intakeRotatePerDegree))),
                                new InstantCommand(() -> dClaw.setPositionI(grab + (45 * intakeRotatePerDegree)))
                        )
                );

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> dClaw.setPositionD(grab - (0 * intakeRotatePerDegree))),
                                new InstantCommand(() -> dClaw.setPositionI(grab + (0 * intakeRotatePerDegree)))
                        )
                );*/
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileActiveOnce(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> drive.odo.resetPosAndIMU())

                    )
                );



  /*      new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER);
            double ActualX = startingPose.getX();
            double ActualY = startingPose.getY();
            new SequentialCommandGroup(
                    if (ActualX>72){
                        for (int i = 0; i < 3; i++) {

                        }
                    }
            );
*/



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