package org.firstinspires.ftc.teamcode;

//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy4

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "PiRhos", name = "bussin2")
public class teleOpOutreach extends LinearOpMode {
    double cascadeMotorPower;

    public PirhosOutreachHArdware robot = new PirhosOutreachHArdware();

    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            cascadeMotorPower = Range.clip(cascadeMotorPower, -.1, .8);
            robot.frontRight.setPower(.5 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.frontLeft.setPower(.5 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.backRight.setPower(.5 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.backLeft.setPower(.5 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.cascadeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            /*if (gamepad1.right_trigger > 0.5){
                robot.carouselpowerfortime(.5, .1);
            }
            if (gamepad1.left_trigger > 0.5){
                robot.carouselpowerfortime(-.5, .1);
            }*/


            if (gamepad2.y) {

                cascadeMotorPower += .2;
                robot.cascadeMotorRight.setPower(cascadeMotorPower);
                robot.cascadeMotorLeft.setPower(cascadeMotorPower);

            }
            if (gamepad2.a) {

                cascadeMotorPower -= .2;
                robot.cascadeMotorRight.setPower(cascadeMotorPower);
                robot.cascadeMotorLeft.setPower(cascadeMotorPower);

            } else {
                robot.cascadeMotorRight.setPower(0);
                robot.cascadeMotorLeft.setPower(0);
            }
            if (gamepad2.right_trigger > .3){
                robot.intake2.setPower(1);
                robot.intake1.setPower(-1);
            }
            if (gamepad2.left_trigger > .3){
                robot.intake2.setPower(-1);
                robot.intake1.setPower(1);
            }
            if (gamepad2.left_trigger < .3 && gamepad2.right_trigger < .3){
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
        }
    }
}