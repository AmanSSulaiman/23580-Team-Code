package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MedInnoServos")
@Disabled
public class MedInnoServos extends LinearOpMode{
    public Servo leftServo, rightServo;

    public void runOpMode(){
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        leftServo.setDirection(Servo.Direction.REVERSE);

        //rightServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            leftServo.setPosition(0);
            rightServo.setPosition(.4);
            sleep(1500);
            leftServo.setPosition(.4);
            rightServo.setPosition(0);
            sleep(1500);


        }
            //rightServo.setPosition(0);

            //rightServo.setPosition(.1);


    }
}
