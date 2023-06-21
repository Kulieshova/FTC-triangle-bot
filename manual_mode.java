/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/

package org.firstinspires.ftc.teamcode;

// w0 - Front
// w1 - Right
// w2 - Left

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Queue;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Concept Datalogger v01", group = "Datalogging")
public class ConceptDatalogger extends LinearOpMode
{
    Datalog datalog;
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
    // Converts angle from [0, 360] range to [-180, 180]
    double angleInRange(Double angle){
        if(angle > 180){
            return angle -=360;
        }
        else {
            return angle += 360;
        }
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
        
        
        
                // Logs stuff :)
        datalog.globalAngle.set(globalAngle);
        datalog.motor1.set(w1.getPower());
        datalog.motor2.set(w2.getPower());
        datalog.ticks1.set(w1.getCurrentPosition());
        datalog.ticks2.set(w2.getCurrentPosition());
        datalog.avgtps1.set(avgTPS1);
        datalog.avgtps2.set(avgTPS2);
        datalog.writeLine();
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
        
        // Logs stuff :)
        datalog.globalAngle.set(globalAngle);
        datalog.motor1.set(w1.getPower());
        datalog.motor2.set(w2.getPower());
        datalog.ticks1.set(w1.getCurrentPosition());
        datalog.ticks2.set(w2.getCurrentPosition());
        datalog.avgtps1.set(avgTPS1);
        datalog.avgtps2.set(avgTPS2);
        datalog.writeLine();
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

    private void joystickTurn(){
        double x_left_joy = gamepad1.left_stick_x;
        double y_left_joy = gamepad1.left_stick_y;
        
        double angle = Math.atan2(y_left_joy, x_left_joy);
        double increment = Math.PI / 6;
        double incrementedAngle = increment * (Math.round(angle/increment));

        telemetry.addData("Angle: ", Math.toDegrees(incrementedAngle));
        telemetry.update();
    }
    // ---------------------------------------------------------------------------------
    
    
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
    
    
   // 4 TICKS PER DEGREE!!!
   
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
    
        if ( Math.abs((degrees % 360)) <= 180){ // +
        int startingGlobalAngle = (int)getAngle();

        // Turn using gyroscope untill angle is reached.
            while(Math.abs(Math.abs(startingGlobalAngle) - Math.abs((int)getAngle())) < Math.abs((degrees % 360))){
                double progress = Math.abs((Math.abs((int)getAngle()-Math.abs(startingGlobalAngle))/Math.abs((degrees % 360))))*100; //tracks percentage of current turn  NOT WORKING  
                telemetry.addData("Progress: ", progress);
                telemetry.update();
                double[] i_speeds_list = {0.7, 0.5, 0.4};
                double i_speed;
                if (progress < 25){
                    i_speed = i_speeds_list[0];
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
        int startingGlobalAngle = (int)getAngle();
        // Turn using gyroscope untill angle is reached.
        
        // ADD RAMPING HERE \/ *********************  1/25   /// ????????? 1/27
        while(Math.abs(Math.abs(startingGlobalAngle) - Math.abs((int)getAngle())) < 360 - Math.abs((degrees % 360))){
          double progress = Math.abs((Math.abs((int)getAngle()-Math.abs(startingGlobalAngle))/Math.abs((degrees % 360))))*100; //tracks percentage of current turn  NOT WORKING  
          double[] i_speeds_list = {0.7, 0.5, 0.4};
            double i_speed;
            if (progress < 25){
                i_speed = i_speeds_list[0];
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
    }
   /// ????????? 1/27
 /*  private void driveForTicks(int ticks){
       resetMotor(w0);
       resetMotor(w1);
       resetMotor(w2);
       
       while(Math.abs(w1.getCurrentPosition()) < ticks){
           PID();
       }

       w1.setPower(0);
       w2.setPower(0);
       w0.setPower(0);
   } */


    // ---------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
       // battery = hardwareMap.voltageSensor.get("Expansion Hub 2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize the datalog
        datalog = new Datalog("new_datalog");

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        //   datalog.opModeStatus.set("INIT");
        //datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

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
    
       // datalog.opModeStatus.set("RUNNING");
        for (int i = 0; opModeIsActive(); i++)
        {
            // Note that the order in which we set datalog fields
            // does *not* matter! The order is configured inside
            // the Datalog class constructor.

            //     datalog.loopCounter.set(i);

            Orientation orientation = imu.getAngularOrientation();

            datalog.globalAngle.set(globalAngle);
            datalog.motor0.set(w0.getPower());
            datalog.motor1.set(w1.getPower());
            datalog.motor2.set(w2.getPower());

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // Datalog fields are stored as text only; do not format here.
            telemetry.addData("globalAngle", datalog.globalAngle);
            telemetry.addData("motor0", datalog.motor0);
            telemetry.addData("motor1", datalog.motor1);
            telemetry.addData("motor2", datalog.motor2);
            telemetry.addLine();
            telemetry.update();
            sleep(20);
            
        while(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0){
            joystickTurn();
        }
        resetMotor(w0);
        resetMotor(w1);
        resetMotor(w2);
        
        // Rasining lift
        while(gamepad1.y){
            liftMotor.setPower(-1);
        }
        
        // Lowering lift
        while(gamepad1.a){
            liftMotor.setPower(1);
        }
        // Reseting lift's motor
        liftMotor.setPower(0);
        
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

        // ??????????????? 1/27
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        
        if(gamepad1.left_bumper){
liftOperation(8700);
liftOperation(-8500);
}
        
        }
    
        /*
         * The datalog is automatically closed and flushed to disk after 
         * the OpMode ends - no need to do that manually :')
         */
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
