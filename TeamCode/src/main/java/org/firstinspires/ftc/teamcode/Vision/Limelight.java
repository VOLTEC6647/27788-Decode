

package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;

public class Limelight implements Subsystem {
    private Bot bot;
    private Limelight3A limelight;

    // Conversion constants
    private static final double METERS_TO_INCH = 39.37;
    private static final double INCH_TO_PEDRO = 1.0 / 0.5; // 1 pedro unit = 0.5 in

    // PID controller for continuous rotation control
    private final PIDFController rotationPID;

    // Continuous input range for angle wrapping
    private double minInput = -Math.PI;
    private double maxInput = Math.PI;
    private boolean continuousInputEnabled = false;

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

        // Initialize PID controller with continuous input enabled for angles
        rotationPID = new PIDFController(-3, 0.0, 0.0, 0.0);
        enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Enables continuous input for angle wrapping.
     * This ensures that angles like -PI and PI are treated as equivalent.
     *
     * @param minimumInput The minimum input value (e.g., -Math.PI)
     * @param maximumInput The maximum input value (e.g., Math.PI)
     */
    private void enableContinuousInput(double minimumInput, double maximumInput) {
        this.minInput = minimumInput;
        this.maxInput = maximumInput;
        this.continuousInputEnabled = true;
    }

    /**
     * Wraps the error to ensure it's within the continuous input range.
     * This handles angle wrapping for smooth rotation control.
     *
     * @param error The error between current and target values
     * @return The wrapped error
     */
    private double wrapError(double error) {
        if (!continuousInputEnabled) {
            return error;
        }

        double inputRange = maxInput - minInput;

        // Normalize error to be within [-inputRange/2, inputRange/2]
        while (error > inputRange / 2) {
            error -= inputRange;
        }
        while (error < -inputRange / 2) {
            error += inputRange;
        }

        return error;
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

            // --- Limelight meters -> inches -> FTC Pedro units ---
            double xFTC = (botpose.getPosition().x * METERS_TO_INCH) * INCH_TO_PEDRO;
            double yFTC = (botpose.getPosition().y * METERS_TO_INCH) * INCH_TO_PEDRO;
            double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

            // --- Build Pose in FTC coordinates, then convert to Pedro bottom-left ---
            Pose pedroPose = new Pose(72.0+botpose.getPosition().x* METERS_TO_INCH, 72.0+botpose.getPosition().y* METERS_TO_INCH, headingRad);

            // careful with this update, it might conflict with Pinpoint if not synchronized
              MecanumDrive.follower.setPose(pedroPose);

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
        // Get current pose from follower
        Pose currentPose = MecanumDrive.follower.getPose();

        // Get current position (in inches)
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        // Calculate angle to target (0,0)
        double angleToTarget = Math.atan2(-currentY, -currentX);

        // Calculate the error and wrap it for continuous input
        double error = angleToTarget - currentHeading;
        error = wrapError(error);

        // Use the wrapped error with the PID controller
        // We use calculate with the wrapped setpoint
        return rotationPID.calculate(currentHeading, currentHeading + error);
    }
}
