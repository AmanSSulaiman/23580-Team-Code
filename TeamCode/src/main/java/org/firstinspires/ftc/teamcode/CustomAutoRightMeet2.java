/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "CustomAutoRightMeet2")

public class CustomAutoRightMeet2 extends LinearOpMode {
    piRhosHardware robot = new piRhosHardware();


    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "CustomSleeve.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "Arm",
            "Bot",
            "Brain"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQ4u9cz/////AAABmQBvtwU6sE/JoEJ/ZOE9KUyENZ1/xxJJdVYFMFm2HbEWktgfq+lfoTLUzT62Gdxec0BG3E7AG+TkTB88zFDlB0OijodJYzcjxFlc2xLmthPqEkfAkW34rho3CRNzUfAAZsUivjibORs2/YIWZvx1rnKFKLE64y8ElKnhvmaLx7QfXxC8pp77/6eczRqEm5J8stblWKmP4EPVwvHd2o3RcNOwBcvP+hrehni8+fVbkrUomfgHmsah/h91gx/5T1n40qtfXlOxciVx+O98elIQobpXKC5w8Zf/7zQDCvFrbuvb55nyq0DEt4xzY36TKMpP0PZdlVqIausikK3irLHic+sh1vLJhkulCTJXQssesqwQ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    String LABEL = "";
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        robot.initTeleOpIMU(hardwareMap);

        robot.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turntable.setTargetPosition(0);
        robot.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turntable.setPower(.1);
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /*robot.backRight.setDirection(DcMotor.Direction.FORWARD);
        robot.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);*/




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            robot.timer.reset();
            while (robot.timer.seconds() < 1.5){
                robot.intakeServoRight.setPower(1);
                robot.intakeServoLeft.setPower(-1);
            }
            robot.intakeServoLeft.setPower(0);
            robot.intakeServoRight.setPower(0);
            robot.cascadeMotorLeft.setTargetPosition(-50);
            robot.cascadeMotorRight.setTargetPosition(50);
            robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setPower(.55);
            robot.cascadeMotorRight.setPower(.55);
            sleep(250);
            robot.timer.reset();
            while (opModeIsActive() && robot.timer.seconds() < .5) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                            LABEL = recognition.getLabel();
                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            telemetry.addLine("this auto gets bitches");
                        }
                        telemetry.update();

                    }
                }

            }
            if (LABEL.equals("Arm")){
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-2);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-19);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-45);
                sleep(125);
                resetEncodersCascade();
                runUsingEncodersCascade();
                encoderCascade(2);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(5);
                sleep(150);
                robot.timer.reset();
                while (robot.timer.seconds() < 1.5) {//take out
                    robot.intakeServoLeft.setPower(.7);
                    robot.intakeServoRight.setPower(-.7);
                }
                robot.intakeServoLeft.setPower(0);
                robot.intakeServoRight.setPower(0);
                sleep(125);
                robot.cascadeMotorRight.setPower(0);
                robot.cascadeMotorLeft.setPower(0);
                resetEncoders();
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-4);
                sleep(125);

                resetEncoders();
                runUsingEncoders();
                sleep(125);
                encoderStrafe(-15);
                sleep(125);
                encoderCascade(0);


                /*resetEncoders();
                runUsingEncoders();
                encoderDrive(48);
                resetEncoders();
                runUsingEncoders();*/

            }
            else if (LABEL.equals("Brain") || LABEL.equals("")){
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-2);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-19);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-45);
                sleep(125);
                resetEncodersCascade();
                runUsingEncodersCascade();
                encoderCascade(2);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(5);
                sleep(125);
                robot.timer.reset();
                while (robot.timer.seconds() < 2) {//take out
                    robot.intakeServoLeft.setPower(1);
                    robot.intakeServoRight.setPower(-1);
                }
                robot.intakeServoLeft.setPower(0);
                robot.intakeServoRight.setPower(0);
                sleep(125);
                robot.cascadeMotorRight.setPower(0);
                robot.cascadeMotorLeft.setPower(0);
                resetEncoders();
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-4);
                sleep(125);

                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-15);
                sleep(125);

                resetEncoders();
                runUsingEncoders();

                encoderDrive(21);
                encoderCascade(0);
            }
            else if (LABEL.equals("Bot")){
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-2);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-19);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-45);
                sleep(125);
                resetEncodersCascade();
                runUsingEncodersCascade();
                encoderCascade(2);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(5);
                sleep(125);
                robot.timer.reset();
                while (robot.timer.seconds() < 1) {//take out
                    robot.intakeServoLeft.setPower(1);
                    robot.intakeServoRight.setPower(-1);
                }
                robot.intakeServoLeft.setPower(0);
                robot.intakeServoRight.setPower(0);
                sleep(125);
                robot.cascadeMotorRight.setPower(0);
                robot.cascadeMotorLeft.setPower(0);
                resetEncoders();
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(-5);
                sleep(125);

                resetEncoders();
                runUsingEncoders();
                encoderStrafe(-12 );
                sleep(1000);
                encoderCascade(0);
                sleep(125);
                resetEncoders();
                runUsingEncoders();
                encoderDrive(43);



                /*resetEncoders();
                runUsingEncoders();
                encoderDrive(48);
                resetEncoders();
                runUsingEncoders();*/

            }



        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void encoderDrive (double inches){
        robot.backLeft.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.backRight.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.frontLeft.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.frontRight.setTargetPosition(robot.inchesToTicksOldDrive(inches));
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.timer.reset();
        robot.backLeft.setPower(.8);
        robot.backRight.setPower(.8);
        robot.frontLeft.setPower(.8);
        robot.frontRight.setPower(.8);
        while (opModeIsActive() &&
                (robot.timer.seconds() < 5) &&
                (robot.backLeft.isBusy() && robot.backRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to", robot.inchesToTicksOldDrive(15));
            telemetry.addData("Currently at", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }
        sleep(125);
        robot.setPowerOfAllMotorsTo(0);
        sleep(125);
    }
    public void encoderStrafe (int inches){

        robot.backRight.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.frontLeft.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.frontRight.setTargetPosition(-robot.inchesToTicksOldDrive(inches));
        robot.backLeft.setTargetPosition(robot.inchesToTicksOldDrive(inches));


        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.timer.reset();
        robot.backRight.setPower(.65);
        robot.frontLeft.setPower(.65);
        robot.frontRight.setPower(.65);
        robot.backLeft.setPower(.65);

        while (opModeIsActive() &&
                (robot.timer.seconds() < 30) &&
                (robot.backLeft.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to", robot.inchesToTicksOldDrive(22));
            telemetry.addData("BL Target Position", robot.backLeft.getTargetPosition());
            telemetry.addData("Currently at BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("Currently at FR", robot.frontRight.getCurrentPosition());
            telemetry.addData("Currently at FL", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Currently at BR", robot.backRight.getCurrentPosition());
            telemetry.update();
            sleep(125);
        }
        sleep(125);
        robot.setPowerOfAllMotorsTo(0);
        sleep(125);
    }
    public void encoderCascade (int num){
        if (num == 0){
            robot.cascadeMotorLeft.setTargetPosition(-10);
            robot.cascadeMotorRight.setTargetPosition(10);
            robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            robot.cascadeMotorLeft.setPower(.7);
            robot.cascadeMotorRight.setPower(.7);
            while (opModeIsActive() &&
                    (robot.timer.seconds() < 5) &&
                    (robot.cascadeMotorLeft.isBusy() && robot.cascadeMotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", 0);
                telemetry.addData("Currently at", -robot.backLeft.getCurrentPosition());
                telemetry.update();
            }
            sleep(125);
            //cascadeBrake();
            robot.cascadeMotorRight.setPower(0);
            robot.cascadeMotorLeft.setPower(0);
            sleep(125);
        }
        else if (num == 1){
            robot.cascadeMotorLeft.setTargetPosition(-1300);
            robot.cascadeMotorRight.setTargetPosition(1300);
            robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            robot.cascadeMotorLeft.setPower(.7);
            robot.cascadeMotorRight.setPower(.7);
            while (opModeIsActive() &&
                    (robot.timer.seconds() < 5) &&
                    (robot.cascadeMotorLeft.isBusy() && robot.cascadeMotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", 0);
                telemetry.addData("Currently at", -robot.backLeft.getCurrentPosition());
                telemetry.update();
            }
            sleep(125);
            //cascadeBrake();
            robot.cascadeMotorRight.setPower(0);
            robot.cascadeMotorLeft.setPower(0);
            sleep(125);
        }
        else if (num == 2){
            robot.cascadeMotorLeft.setTargetPosition(-2250);
            robot.cascadeMotorRight.setTargetPosition(2250);
            robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            robot.cascadeMotorLeft.setPower(.7);
            robot.cascadeMotorRight.setPower(.7);
            while (opModeIsActive() &&
                    (robot.timer.seconds() < 5) &&
                    (robot.cascadeMotorLeft.isBusy() && robot.cascadeMotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", 0);
                telemetry.addData("Currently at", -robot.backLeft.getCurrentPosition());
                telemetry.update();
            }
            sleep(125);
            //cascadeBrake();
            robot.cascadeMotorRight.setPower(0);
            robot.cascadeMotorLeft.setPower(0);
            sleep(125);
        }
        else if (num == 3){
            robot.cascadeMotorLeft.setTargetPosition(-2950);
            robot.cascadeMotorRight.setTargetPosition(2950);
            robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            robot.cascadeMotorLeft.setPower(.7);
            robot.cascadeMotorRight.setPower(.7);
            while (opModeIsActive() &&
                    (robot.timer.seconds() < 5) &&
                    (robot.cascadeMotorLeft.isBusy() && robot.cascadeMotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", 0);
                telemetry.addData("Currently at", -robot.backLeft.getCurrentPosition());
                telemetry.update();
            }
            sleep(125);
            //cascadeBrake();
            robot.cascadeMotorRight.setPower(0);
            robot.cascadeMotorLeft.setPower(0);
            sleep(125);
        }



    }
    public void resetEncoders(){
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncodersCascade(){
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncodersCascade(){
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void cascadeBrake(){
        robot.cascadeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cascadeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



}
