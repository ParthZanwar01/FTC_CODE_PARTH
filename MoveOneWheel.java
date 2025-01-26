package pedroPathing.actual_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Change LeftFront to rightRear
//Change rightRear to leftfront
//Change rightFront to leftrear
//Change leftBack to rightFront

@TeleOp(name = "MoveOneWheel")
public class MoveOneWheel extends OpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        //Set Motor Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            leftFront.setPower(1.0);
        } else if (gamepad1.square) {
            leftBack.setPower(1.0);
        } else if (gamepad1.circle) {
            rightFront.setPower(1.0);
        } else if (gamepad1.triangle) {
            rightBack.setPower(1.0);
        } else {
            leftBack.setPower(0.0);
            leftFront.setPower(0.0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }
}
