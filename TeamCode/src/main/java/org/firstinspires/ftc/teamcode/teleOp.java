package org.firstinspires.ftc.teamcode;

//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy4

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.tensorflow.lite.task.core.vision.ImageProcessingOptions;

@TeleOp(group = "PiRhos", name = "TeleOp Pirhos 2022-23")
public class teleOp extends LinearOpMode {
    double cascadeMotorPower;
    BNO055IMU imu;
    Orientation angles;

    public piRhosHardware robot = new piRhosHardware();
    //int number, newNumber;
    public void runOpMode() {

        robot.initTeleOpIMU(hardwareMap);
        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int pos1 = 650;
        int pos2 = 1300;
        int pos1Back = -650;


        waitForStart();
        while (opModeIsActive()) {

//            while (number % 2 != 1){
//                for (int i = 0; i < ("" + number).length(); i++) {
//                    newNumber += number % 10;
//                }
//                if (newNumber == number){
//                    break;
//                }
//                number = newNumber;
//
//            }
//            if (number % 2 == 1){
//                System.out.println("goofy");
//            }
//            else{
//                System.out.println("not goofy");
//            }

            cascadeMotorPower = Range.clip(cascadeMotorPower, -.4, 1);
            robot.frontRight.setPower(.65 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.frontLeft.setPower(.65 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.backRight.setPower(.65 * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.backLeft.setPower(.65 * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad2.left_stick_y > .4 && robot.cascadeMotorRight.getCurrentPosition() < 3050){
                robot.cascadeMotorRight.setPower(-.655 * gamepad2.left_stick_y);
                robot.cascadeMotorLeft.setPower(.65 * gamepad2.left_stick_y);
                telemetry.addData("position", robot.cascadeMotorRight.getCurrentPosition());
                telemetry.update();


            }
            if(gamepad2.left_stick_y < -.4){
                robot.cascadeMotorRight.setPower(-.6 * gamepad2.left_stick_y);
                robot.cascadeMotorLeft.setPower(.6 * gamepad2.left_stick_y);
                telemetry.addData("position", robot.cascadeMotorRight.getCurrentPosition());
                telemetry.update();
            }
            if (gamepad2.left_stick_y > -.4 && gamepad2.left_stick_y < .4){

                robot.cascadeMotorLeft.setPower(0);
                robot.cascadeMotorRight.setPower(0);
                telemetry.addData("position", robot.cascadeMotorRight.getCurrentPosition());
                telemetry.update();
            }


            robot.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);


            /*if (gamepad1.right_trigger > 0.5){
                robot.carouselpowerfortime(.5, .1);
            }
            if (gamepad1.left_trigger > 0.5){
                robot.carouselpowerfortime(-.5, .1);
            }*/

            /*if (gamepad2.y) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(1250);
                robot.cascadeMotorLeft.setTargetPosition(-1250);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.3);
                robot.cascadeMotorRight.setPower(.3);
                while (robot.timer.seconds() < 3) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            if (gamepad2.a) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(-1250);
                robot.cascadeMotorLeft.setTargetPosition(1250);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.3);
                robot.cascadeMotorRight.setPower(.3);
                while (robot.timer.seconds() < 3) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            if (gamepad2.x) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(175);
                robot.cascadeMotorLeft.setTargetPosition(-175);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.3);
                robot.cascadeMotorRight.setPower(.3);
                while (robot.timer.seconds() < 3) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }*/
            /*if (gamepad2.right_bumper) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(300);
                robot.cascadeMotorLeft.setTargetPosition(-300);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.3);
                robot.cascadeMotorRight.setPower(.3);
                while (robot.timer.seconds() < 3) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            if (gamepad2.left_bumper) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(130);
                robot.cascadeMotorLeft.setTargetPosition(-130);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();

                robot.cascadeMotorLeft.setPower(.3);
                robot.cascadeMotorRight.setPower(.3);
                robot.intake2.setPower(-.7);
                robot.intake1.setPower(.7);
                while (robot.timer.seconds() < 1.5) {

                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }





            if (gamepad2.y) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(1900);
                robot.cascadeMotorLeft.setTargetPosition(-1900);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.5);
                robot.cascadeMotorRight.setPower(.5);
                while (robot.timer.seconds() < 4) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            if (gamepad2.b) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(2950);
                robot.cascadeMotorLeft.setTargetPosition(-2950);
                robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.5);
                robot.cascadeMotorRight.setPower(.5);
                while (robot.timer.seconds() < 5) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            if (gamepad2.a) {
                //encoderDrive(.5, 5, 5, 10);
                robot.cascadeMotorRight.setTargetPosition(10);
                robot.cascadeMotorLeft.setTargetPosition(-10);
                robot.timer.reset();
                robot.cascadeMotorLeft.setPower(.5);
                robot.cascadeMotorRight.setPower(.5);
                while (robot.timer.seconds() < 5) {
                    telemetry.addData("right pos", robot.cascadeMotorRight.getCurrentPosition());
                    telemetry.addData("left pos", robot.cascadeMotorLeft.getCurrentPosition());
                    telemetry.update();
                    sleep(250);
                }
                robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }*/
            if (gamepad2.right_trigger > .3){
                robot.intakeServoRight.setPower(1);
                robot.intakeServoLeft.setPower(-1);
            }
            if (gamepad2.left_trigger > .3){
                robot.intakeServoRight.setPower(-1);
                robot.intakeServoLeft.setPower(1);
            }
            if (gamepad2.right_trigger < .3 && gamepad2.left_trigger < .3) {
                robot.intakeServoRight.setPower(0);
                robot.intakeServoLeft.setPower(0);
            }

            if (gamepad2.y && robot.cascadeMotorRight.getCurrentPosition() > 1250){
                if ((robot.turntable.getCurrentPosition() >=-325 && robot.turntable.getCurrentPosition() <= 325)){
                    robot.turntable.setTargetPosition(pos1);
                    robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turntable.setPower(.4);
                }

                else if (robot.turntable.getCurrentPosition() >= -975 && robot.turntable.getCurrentPosition() <= -325){
                    robot.turntable.setTargetPosition(0);
                    robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turntable.setPower(.4);

                }
            }
            if (gamepad2.a && robot.cascadeMotorRight.getCurrentPosition() > 1250){
                if ((robot.turntable.getCurrentPosition() >=-325 && robot.turntable.getCurrentPosition() <= 325)){
                    robot.turntable.setTargetPosition(pos1Back);
                    robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turntable.setPower(.4);
                }
                else if ((robot.turntable.getCurrentPosition() >= 325 && robot.turntable.getCurrentPosition() <= 975)){
                    robot.turntable.setTargetPosition(0);
                    robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turntable.setPower(.4);
                }
                else if ((robot.turntable.getCurrentPosition() >= 975 && robot.turntable.getCurrentPosition() <= 1625)){
                    robot.turntable.setTargetPosition(pos1);
                    robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turntable.setPower(.4);
                }

            }




            /*if (gamepad2.dpad_left) {
                robot.slideForTime(0.5, 0.05);
            }

            if (gamepad2.dpad_right) {
                robot.slideForTime(-0.5, 0.05);
            }*/

            /*if (gamepad2.left_bumper) {
                robot.intake1.setPower(1);
                robot.intake2.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);
            }
            if (!gamepad2.left_bumper) {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
            if (!gamepad2.right_bumper) {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }*/
            /*if (!gamepad2.right_bumper && !gamepad2.left_bumper){
                robot.intake2.setPower(0);
                robot.intake1.setPower(0);
            }
            if(gamepad2.right_bumper){
                robot.intake1.setPower(-.75);
                robot.intake2.setPower(.75);
            }
            if (gamepad2.left_bumper) {
                robot.intake1.setPower(.75);
                robot.intake2.setPower(-.75);
            }*/


            /*if (gamepad2.x){
                robot.bucket.setPosition(.2);
            }
            if (gamepad2.b){
                robot.bucket.setPosition(0);
            }*/
//            if (gamepad2.dpad_up){
//                robot.bucket.setPosition(.38);
//                //HELLLO WORLD(gam
//            }


        }
    }
}