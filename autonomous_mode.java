/* Copyright (c) 2017 FIRST. All rights reserved.
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Left: Autonomous Mode", group="Linear Opmode")

public class AutonomousLeft extends LinearOpMode {

    // Declare OpMode members.
    BNO055IMU imu;
    //VoltageSensor battery;
    double storeHeading = Double.NaN;
    double globalAngle = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor w0 = null;
    private DcMotor w1 = null;
    private DcMotor w2 = null;
    private DcMotor liftMotor = null;
    Orientation lastAngles = new Orientation();
    private double max_speed;
    boolean started = false;

private void printData(){
        // add new variables as needed
        telemetry.addData("Ramp Modifier: ", rampModifier);
        telemetry.addData("Motor 0 Power: ", w0.getPower());
        telemetry.addData("Motor 1 Power: ", w1.getPower());
        telemetry.addData("Motor 2 Power: ", w2.getPower());
        telemetry.addData("AvgTicks1: ", avgTPS1);
        telemetry.addData("AvgTicks2: ", avgTPS2);
        telemetry.addData("Motor1Queue: ", motor1Queue.toString());
        telemetry.addData("Motor2Queue: ", motor2Queue.toString());
        telemetry.addData("Current - Start: ", currentTime - startTime);        
        telemetry.update();
    }  
    
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public void resetMotor(DcMotor m){
    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m.setDirection(DcMotor.Direction.FORWARD);
    m.setPower(0);
}

public DcMotor init_motor(String id) {
        DcMotor m = null;
        m = hardwareMap.get(DcMotor.class, id);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setDirection(DcMotor.Direction.FORWARD);
        return m;
    }
    
    float conversionFactor = 8000; // Conversion factor for adjusting TPS
    float timeInterval = 50; // time step for adding TPS values to queues
    double initialMotorSpeed1 = 0.644;
    double initialMotorSpeed2 = 0.69;
    double initialBackwardsMotorSpeed1 = 0.6;
    double initialBackwardsMotorSpeed2 = 0.85;
    double rampModifier = 0.5; // Starting ram modifier
    public List<Float> motor1Queue = new ArrayList<>(); // List of TPS for w1
    public List<Float> motor2Queue = new ArrayList<>(); // List of TPS for w2
    float avgTPS1 = 0; // Ticks per second for W1 motor
    float avgTPS2 = 0; // Ticks per second for W2 motor
    long startTime = System.currentTimeMillis();
    long currentTime = System.currentTimeMillis();
    float TPStarget = 1350; // Target TPS
    float TPSUpperBoundary = 5; // Max error boundary for correction
    float TPSLowerBoundary = 1; // Min error boundary for correction
    float TPSpercent = 1;

    // Backwards correction and ramping
    private void DIP() { //-- NOT WORKING 1/26 and 1/27
        // ramping first
        while(rampModifier < 1) {
            rampModifier += 0.1;
            w1.setPower(initialBackwardsMotorSpeed1*rampModifier);
            w2.setPower(-initialBackwardsMotorSpeed2*rampModifier);
            sleep(10);
        }
    // avg TPS calculations
        currentTime = System.currentTimeMillis();
        if(!(currentTime - startTime >= timeInterval)){ // run function only every tenth of a second.
                return;
            }
        // addds current ticks to queues
        if(currentTime - startTime >= timeInterval){
            motor1Queue.add((float) w1.getCurrentPosition());
            motor2Queue.add((float) w2.getCurrentPosition());
            startTime = System.currentTimeMillis();
             // Clears queue if it has more than 5 elemets
            if(motor1Queue.size() > 5){
                motor1Queue.remove(0);
                motor2Queue.remove(0);
            }
        } 
        // Calculates average TPS
        if(motor1Queue.size() > 1){
            avgTPS1 = (motor1Queue.get(motor1Queue.size() - 1) - motor1Queue.get(0))/motor1Queue.size() * (1000/timeInterval);
            avgTPS2 = (motor2Queue.get(motor2Queue.size() - 1) - motor2Queue.get(0))/motor2Queue.size() * (1000/timeInterval);
        }   
        // Adjusts W1 motor
        if (Math.abs(TPStarget - Math.abs(avgTPS1))/TPStarget * 100 > TPSLowerBoundary && Math.abs(TPStarget - Math.abs(avgTPS1))/TPStarget * 100 < TPSUpperBoundary){
            w1.setPower((double)(w1.getPower() + ((TPStarget-Math.abs(avgTPS1))/conversionFactor))); /// substracts power if avgTPS is bigger than TPStarget; adds if avgTPS < TPStarget
        }
        else{
            w1.setPower(initialBackwardsMotorSpeed1);
        }
        // Adjusts W2 motor
        if (Math.abs(TPStarget - Math.abs(avgTPS2))/TPStarget * 100 > TPSLowerBoundary && Math.abs(TPStarget - Math.abs(avgTPS2))/TPStarget * 100 < TPSUpperBoundary){
            w2.setPower((double)(w2.getPower() - ((TPStarget-Math.abs(avgTPS2))/conversionFactor))); /// substracts power if avgTPS is bigger than TPStarget; adds if avgTPS < TPStarget
        }
        else {
            w2.setPower(-initialBackwardsMotorSpeed2);
        }
        
        
        

    }

    // Forward correction and ramping
    private void PID() {
    // ramping first
        while(rampModifier < 1) {
            rampModifier += 0.1;
            w1.setPower(-initialMotorSpeed1*rampModifier);
            w2.setPower(initialMotorSpeed2*rampModifier);
            sleep(10);
        }
    // avg TPS calculations
        currentTime = System.currentTimeMillis();
        if(!(currentTime - startTime >= timeInterval)){ // run function only every tenth of a second.
                return;
        }
        // addds current ticks to queues
        if(currentTime - startTime >= timeInterval){
            motor1Queue.add((float) w1.getCurrentPosition());
            motor2Queue.add((float) w2.getCurrentPosition());
            startTime = System.currentTimeMillis();
            // Clears queue if it has more than 5 elemets
            if(motor1Queue.size() > 5){
                motor1Queue.remove(0);
                motor2Queue.remove(0);
            }
        } 
        // Calculates average TPS
        if(motor1Queue.size() > 1){
            avgTPS1 = (motor1Queue.get(motor1Queue.size() - 1) - motor1Queue.get(0))/motor1Queue.size() * (1000/timeInterval);
            avgTPS2 = (motor2Queue.get(motor2Queue.size() - 1) - motor2Queue.get(0))/ motor1Queue.size() * (1000/timeInterval);
        }   
        
        // Adjusts W1 motor
        if (Math.abs(TPStarget - Math.abs(avgTPS1))/TPStarget * 100 > TPSLowerBoundary && Math.abs(TPStarget - Math.abs(avgTPS1))/TPStarget * 100 < TPSUpperBoundary){
            w1.setPower((double)(w1.getPower() - ((TPStarget-Math.abs(avgTPS1))/conversionFactor))); /// substracts power if avgTPS is bigger than TPStarget; adds if avgTPS < TPStarget
        }
        else{
            w1.setPower(-initialMotorSpeed1);
        }
        // Adjusts W2 motor
        if (Math.abs(TPStarget - Math.abs(avgTPS2))/TPStarget * 100 > TPSLowerBoundary && Math.abs(TPStarget - Math.abs(avgTPS2))/TPStarget * 100 < TPSUpperBoundary){
            w2.setPower((double)(w2.getPower() + ((TPStarget-Math.abs(avgTPS2))/conversionFactor))); /// substracts power if avgTPS is bigger than TPStarget; adds if avgTPS < TPStarget
        }
        else {
            w2.setPower(initialMotorSpeed2);
        }
        
    }
    
    public void setupGyroscope() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
          // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }
    
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    
    private void turnDegrees(int degrees){
      
      // reset encoders
      resetMotor(w0);
      resetMotor(w1);
      resetMotor(w2);
      
      int direction = -1;
      // +1 is right
      // -1 is left

      int rampTicks = (int)(degrees/2);
        if(Math.sin(degrees) < 0){
            direction = -1;
        }
    
    // turning right makes getAngle() more negative
        int startingGlobalAngle = (int)getAngle();

        if ( Math.abs((degrees % 360)) <= 180){ // +
        // Turn using gyroscope untill angle is reached.
            while(Math.abs(Math.abs(startingGlobalAngle) - Math.abs((int)getAngle())) < Math.abs((degrees % 360))){
                double progress = Math.abs((Math.abs((int)getAngle()-Math.abs(startingGlobalAngle))/Math.abs((degrees % 360))))*100; //tracks percentage of current turn  NOT WORKING  
                telemetry.addData("Progress: ", progress);
                telemetry.update();
                double[] i_speeds_list = {0.7, 0.5, 0.2};
                double i_speed;
                if (progress < 25){
                    i_speed = i_speeds_list[2];
                }
                else{
                    i_speed = i_speeds_list[2];
                } 
                
                w0.setPower(i_speed);
                w1.setPower(i_speed);
                w2.setPower(i_speed);
                telemetry.addData(("Turning to " + degrees + "˚") , "...");
                telemetry.addData("Global Angle: " , getAngle());
                telemetry.update();
            }
    } else{
        // Turn using gyroscope untill angle is reached.
        while(Math.abs(Math.abs(startingGlobalAngle) - Math.abs((int)getAngle())) < 360 - Math.abs((degrees % 360))){
          double progress = Math.abs((Math.abs((int)getAngle()-Math.abs(startingGlobalAngle))/Math.abs((degrees % 360))))*100; //tracks percentage of current turn  NOT WORKING  
          double[] i_speeds_list = {0.7, 0.5, 0.2};
            double i_speed;
            if (progress < 25){
                i_speed = i_speeds_list[2];
            }
            else{
                i_speed = i_speeds_list[2];
            } 
            w0.setPower(-i_speed);
            w1.setPower(-i_speed);
            w2.setPower(-i_speed);
            telemetry.addData(("Turning to " + degrees + "˚") , "...");
            telemetry.addData("Global Angle: " , getAngle());
            telemetry.update();
        }
    }     
      w0.setPower(0);
      w1.setPower(0);
      w2.setPower(0);
      
      telemetry.addData("Starting Angle: ", startingGlobalAngle);
      telemetry.addData("Ending Angle: ", globalAngle);
      telemetry.update();
      //sleep(5000); //displays values on screen for 5 seconds

    }
    
    private void turnUntillGyroscope(int angle, String direction){ // "l" or "r"
    
        double power = 0.2;
        if(direction.equals("l")) {
            power = -0.2;
        }
        
        while((int)getAngle() != angle){
            telemetry.addData("global angle: ", getAngle());
            telemetry.update();
         w0.setPower(power);
         w1.setPower(power);
         w2.setPower(power);
        }
         w0.setPower(0);
         w1.setPower(0);
         w2.setPower(0);
        
        
        
    }
    // Drives for the given distance
    private void liftOperation(int ticks){
       resetMotor(liftMotor);
       
       if(ticks < 0){
            while(Math.abs(liftMotor.getCurrentPosition()) < Math.abs(ticks)){
                           telemetry.addData("ticks: ", liftMotor.getCurrentPosition());
                telemetry.update();
                liftMotor.setPower(1.0);
                
            }
       }
       else{
            while(Math.abs(liftMotor.getCurrentPosition()) < ticks){
                telemetry.addData("ticks: ", liftMotor.getCurrentPosition());
                 telemetry.update();
                liftMotor.setPower(-1.0);
            }
       }
       liftMotor.setPower(0);
   }
   
       private void driveForTicks(int ticks){
        
       resetMotor(w0);
       resetMotor(w1);
       resetMotor(w2);
       
       while(w1.getCurrentPosition() > -ticks){
           telemetry.addData("ticks: ", w1.getCurrentPosition());
       telemetry.update();
           PID();
       }

       w1.setPower(0);
       w2.setPower(0);
       w0.setPower(0);
       globalAngle = getAngle();
       
   }
   
          private void driveForTicksBack(int ticks){
        
       resetMotor(w0);
       resetMotor(w1);
       resetMotor(w2);
       
       while(Math.abs(w1.getCurrentPosition()) < ticks){
           telemetry.addData("ticks: ", w1.getCurrentPosition());
       telemetry.update();
           DIP();
       }

       w1.setPower(0);
       w2.setPower(0);
       w0.setPower(0);
       globalAngle = getAngle();
       
   }
   
    
    // --------Object Detect
    //--------
    
private static final String TFOD_MODEL_ASSET = "customSleeveV4.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "YP",
            "zone1",
            "zone2",
            "zone3"
    };
    
    private static final String VUFORIA_KEY =
            "AR5qD2X/////AAABmesV12w+HUDPrdcPirnTIWYxLJgFC1t4gwgN/vpkuO3EceMlYjyR7aG9XH+ZGDaTgjUM9LHe6TlFoST3rhlVoVVW0qpzzTzRUyeBpcTfj2gsxS2nArVUBlNSG5IR10PKiaLmuXrmADgv8416LHJbsOhxvaIJ/heTIXEkxLo+iqNYXs4N+yPpGijrddGbzUOxMNE5QCdja2GDEHXsCtAema6VviApo9RQ6f3z0duhaZiaQ3X5GSClWEgnGNJFZm6dtGklhPpXahFFodFZHZnTEkUdjoTA4c+MLJ3wXFJfcKqeF/g8NvLt06OH4EZUary3AOwiQS2e49BZjs4GYgsH0UdUly3SuZD5FuGnjbir9o5M";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    
    
    int targetZone = 2;

    @Override
    public void runOpMode() {
        
         // OBJECT DETECT
        initVuforia();
        initTfod();
        
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
        
        
        
        
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
       // battery = hardwareMap.voltageSensor.get("Expansion Hub 2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        max_speed = 1;

        w0 = init_motor("w0");
        w1 = init_motor("w1");
        w2 = init_motor("w2");
        liftMotor = init_motor("liftMotor");
        
        // wait for start button.
        setupGyroscope();
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
        
        while (opModeIsActive()) {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                
                int zone1Counter = 0;
                int zone2Counter = 0;
                int zone3Counter = 0;
                
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                    if(recognition.getLabel().equals("zone1")){
                        zone1Counter++;
                    }
                    else if(recognition.getLabel().equals("zone2")){
                        zone2Counter++;
                    }
                    else if(recognition.getLabel().equals("zone3")){
                        zone3Counter++;
                    }
                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                }
                
                if(zone1Counter > zone2Counter){
                    targetZone = 1;
                }
                else if (zone3Counter > 0){
                        targetZone = 3;
                }
                
                telemetry.addData("Zone: ", targetZone); // 2/3/23 IT WORKS
                telemetry.update();
            }
            
                    
                    
                //// Driving?
                

                // driveForTicks(300); // COMPETITION TUNE!
                // sleep(5000);
                // turnUntillGyroscope(-41, "r");// 2/9 adjust for initial gloabl angle

                // driveForTicks(200); 
                // liftOperation(8700);
                // sleep(1000);
                // liftOperation(-8500);
                // sleep(2000);
                // driveForTicksBack(200);
                // sleep(1000);
                // // after delivering cone
                
                //  turnUntillGyroscope(0, "l");
                //     driveForTicks(700);
                // if (targetZone == 1){
                //   sleep(1000); // delete later
                //   turnUntillGyroscope(-90, "l");
                //   driveForTicks(1000);
                   
                   
                // }
                // if (targetZone == 2){
                //     sleep(1000);
                // }
                
                // // IT WORKS> DON'T TOUCH UNLESS YOU CAN MAKE IT BETTER (dont worry i will not touch it :))
                // if (targetZone == 3){
                //     sleep(1000); // delete later
                //   turnUntillGyroscope(90, "r");
                //   driveForTicks(1000);
                // }

                //// Recognizing the pole
                
                
                
                /// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                
                
                // PROPOSED NEW PARKING + DELIEVERING
                
                //  driveForTicks(1600); // COMPETITION TUNE!
                liftOperation(3000);
                 driveForTicks(100);
                if (targetZone == 1){
                    turnUntillGyroscope(90, "l");    
                    driveForTicks(900);
                    turnUntillGyroscope(0, "r");
                    driveForTicks(1150);
                    // turnUntillGyroscope(-90, "r");
                    //     liftOperation(8700);
                    //     driveForTicks(150);
                    //     sleep(1000);
                    //     liftOperation(-8500);
                    // driveForTicksBack(200);
                }
                if(targetZone == 2){
                    driveForTicks(1150);
                    // turnUntillGyroscope(-90, "r");
                    //     liftOperation(8700);
                    //     sleep(1000);
                    //     driveForTicks(150);
                    //     liftOperation(-8500);
                    // driveForTicksBack(200);
                    //turnUntillGyroscope(0, "l");
                    //driveForTicks(400);
                    
                }
                if(targetZone == 3){
                    turnUntillGyroscope(-90, "r");    
                    driveForTicks(1000);
                    turnUntillGyroscope(0, "l");
                    driveForTicks(1150);
                    // turnUntillGyroscope(-90, "r");
                    //     liftOperation(12000);
                    //     sleep(1000);
                    //      driveForTicks(150);
                    //     liftOperation(-12000);
                    // driveForTicksBack(200);
                }
                                
                
                
                
                }
                
            

        // Moving forward
        while(gamepad1.dpad_up){
            if(!started){
                startTime = System.currentTimeMillis();
                started = true;
                 w1.setPower(-initialMotorSpeed1);
                 w2.setPower(initialMotorSpeed2);
            }
            PID();            
            printData();
        }

        // Moving backward
        while(gamepad1.dpad_down){
            if(!started){
                startTime = System.currentTimeMillis();
                started = true;
                 w1.setPower(initialMotorSpeed1);
                 w2.setPower(-initialMotorSpeed2);
            }
            DIP();
        }
        
        rampModifier = 0.5;
        avgTPS1 = 0;
        avgTPS2 = 0;
        motor1Queue.clear();
        motor2Queue.clear();
        started = false;

        // Turning right
        while(gamepad1.dpad_right){
            w0.setPower(0.5);
            w1.setPower(0.5);
            w2.setPower(0.5);
            telemetry.addData("getAngle(): ", getAngle());
            telemetry.update();
        }
        // Turing left
        while(gamepad1.dpad_left){
            w0.setPower(-0.5);
            w1.setPower(-0.5);
            w2.setPower(-0.5);
            telemetry.addData("getAngle(): ", getAngle());
            telemetry.update();
        }
        
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        liftMotor.setPower(0);
        
       break;
        }
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.3f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    
    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;
        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField globalAngle          = new Datalogger.GenericField("globalAngle");
        public Datalogger.GenericField motor0        = new Datalogger.GenericField("motor0");
        public Datalogger.GenericField motor1         = new Datalogger.GenericField("motor1");
        public Datalogger.GenericField motor2      = new Datalogger.GenericField("motor2");
        public Datalogger.GenericField ticks1      = new Datalogger.GenericField("ticks1");
        public Datalogger.GenericField ticks2      = new Datalogger.GenericField("ticks2");
        public Datalogger.GenericField avgtps1      = new Datalogger.GenericField("AvgTPS1");
        public Datalogger.GenericField avgtps2      = new Datalogger.GenericField("AvgTPS2");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            globalAngle,
                            motor0,
                            motor1,
                            motor2,
                            ticks1,
                            ticks2,
                            avgtps1,
                            avgtps2
                    )
                    .build();
        }
        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
    
}
