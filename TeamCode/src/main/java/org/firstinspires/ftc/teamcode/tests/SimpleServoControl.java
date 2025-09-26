package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Simple Servo Control", group = "Examples")
public class SimpleServoControl extends OpMode {

    // 1. Declare your servo objects
    private Servo servoR;
    private Servo servoL;

    // These constants will hold the target positions
    final double POSITION_OPEN = 1.0;  // Position when 'A' is pressed
    final double POSITION_CLOSED = 0.0; // Position when 'B' is pressed

    @Override
    public void init() {
        // 2. Map the servos to the hardware configuration on your Robot Controller
        // Make sure the names "servoR" and "servoL" match your configuration exactly.
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");

        // Optional: It's good practice to set the servos to a known starting position.
        // We will set them to the closed position on initialization.
        servoR.setPosition(POSITION_CLOSED);
        servoL.setPosition(POSITION_CLOSED);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press A to move to 1.0, B to move to 0.0");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 3. Check for controller input in the main loop

        // If the 'a' button on gamepad1 is pressed, move both servos to the open position.
        if (gamepad1.a) {
            servoR.setPosition(POSITION_OPEN);
            servoL.setPosition(POSITION_OPEN);
        }

        // If the 'b' button on gamepad1 is pressed, move both servos to the closed position.
        if (gamepad1.b) {
            servoR.setPosition(POSITION_CLOSED);
            servoL.setPosition(POSITION_CLOSED);
        }

        // 4. Add telemetry to display the servo positions on the Driver Station
        // This is very useful for debugging!
        telemetry.addData("servoR Position", servoR.getPosition());
        telemetry.addData("servoL Position", servoL.getPosition());
        telemetry.update();
    }
}