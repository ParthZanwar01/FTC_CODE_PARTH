package pedroPathing.actual_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LinearSlideOneRev")
public class LinearSlideControl extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the motor
        DcMotor linearSlideMotor = hardwareMap.get(DcMotor.class, "ScoringSlides");

        // Reset the encoder and set the motor to use encoder mode
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start of the op mode
        waitForStart();

        if (opModeIsActive()) {
            // Set the target position (1 revolution = 537.6 ticks)
            int targetPosition = 537;

            // Set the motor target position and mode
            linearSlideMotor.setTargetPosition(targetPosition);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start the motor moving to the target position
            linearSlideMotor.setPower(0.5); // Adjust power as needed

            // Wait until the motor reaches the target position
            while (opModeIsActive() && linearSlideMotor.isBusy()) {
                telemetry.addData("Motor Position", linearSlideMotor.getCurrentPosition());
                telemetry.addData("Target", targetPosition);
                telemetry.update();
            }

            // Stop the motor after reaching the target
            linearSlideMotor.setPower(0);

            telemetry.addData("Status", "Slide movement complete");
            telemetry.update();
            sleep(4000);

            linearSlideMotor.setTargetPosition(0);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(0.5);

            while (opModeIsActive() && linearSlideMotor.isBusy()) {
                telemetry.addData("Motor Position", linearSlideMotor.getCurrentPosition());
                telemetry.addData("Target", targetPosition);
                telemetry.update();
            }

            linearSlideMotor.setPower(0.0);

        }
    }
}
