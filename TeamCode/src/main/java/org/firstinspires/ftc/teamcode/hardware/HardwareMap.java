package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware
        {

        //import drive train motors
        public DcMotor LFM = null;
        public DcMotor RFM = null;
        public DcMotor LBM = null;
        public DcMotor RBM = null;


        //import linear slide motors

        public DcMotor SlideRight = null;
        public DcMotor SlideLeft = null;

        //import intake mechanisms
        public Servo iServoLeft = null;
        public Servo iServoRight = null;
        public CRServo iCRServoLeft = null;
        public CRServo iCRServoRight = null;
        public DcMotor iMotor = null;

        //import outtake servos

        public CRServo oServoRight = null;
        public CRServo oServoLeft = null;


        //additional variables
        HardwareMap hardwareMap = null;
        public ElapsedTime runtime = new ElapsedTime();

        public Hardware(HardwareMap hwMap ) {
                initialize(hwMap);

        }

        private void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //connect drivebase motors
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");

        //connect slides
        SlideRight = hardwareMap.get(DcMotor.class, "SlideRight");
        SlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");

        //connect intake motor + servos
        iMotor = hardwareMap.get(DcMotor.class, "iMotor");

        iServoLeft = hardwareMap.get(Servo.class, "iServoLeft");
        iServoRight = hardwareMap.get(Servo.class, "iServoRight");

        iCRServoLeft = hardwareMap.get(CRServo.class, "iCRServoLeft");
        iCRServoRight = hardwareMap.get(CRServo.class, "iCRServoRight");

        //connect outtake servos
        oServoRight = hardwareMap.get(CRServo.class, "oServoRight");
        oServoLeft = hardwareMap.get(CRServo.class, "oServoLeft");

        //set up motor directions

                //drivebase
                LFM.setDirection(DcMotor.Direction.REVERSE);
                LBM.setDirection(DcMotor.Direction.REVERSE);
                RFM.setDirection(DcMotor.Direction.REVERSE);
                RBM.setDirection(DcMotor.Direction.FORWARD);

                //intake
                iMotor.setDirection(DcMotor.Direction.REVERSE);



        }




}


