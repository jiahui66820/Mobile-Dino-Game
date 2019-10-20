package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.AnimatorSet;
import android.animation.ValueAnimator;
import android.content.Context;
import android.graphics.drawable.AnimationDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Precision;
import org.junit.Assert;

// 1020 game #4 Initial class
//
import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends AppCompatActivity {

    //
    private final static String TAG = "MainActivity";


    CustomView view;
    AnimationDrawable dinoAnimation;
    AnimationDrawable dinoDuckingAnimation;
    AnimationDrawable groundAnimation;
    AnimationDrawable cactusSprite1Animation;


    // 1020
    AnimationDrawable birdSpriteAnimation;

    AnimationDrawable dinoJumpAnimation;

    Animation groundSlide;
    Animation obstacleSlide;
    Animation dinoJump;
    Animation dinoDuck;

    ImageView groundSprite;
    ImageView cactusSprite1;

    // 1020
    ImageView birdSprite;

    ImageView dinoSprite;
    ImageView dinoDuckingSprite;
    ImageView dinoJumpSprite;

    // 1020 game#3 calculate score & front page start
    private TextView scoreLabel;
    private TextView startLabel;
    // 自創
    private int scoreTime = 0;


    // 1020 game #4 Initial class
    //private Timer timer = new Timer();
    // pick up android studio (another option: java)
    // 沒用到
    //private Handler handler = new Handler();

    //新招
    // https://blog.csdn.net/zuolongsnail/article/details/8168689
    private Timer mTimer;






    private double absolutePressure;
    private boolean initRun = true;

    KalmanFilter filter;

    float gravX = 0;
    float gravY = 0;
    float gravZ = 0;

    float totAcc =0;

    double total=0;
    double alt=0;

    boolean isBaroReady = false;
    boolean isAccReady = false;

    private float[] gravityValues = null;
    private float[] magneticValues = null;

    boolean isJumping;
    boolean isPrepareJump = false;
    boolean isDucking;
    @Override

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 在AndroidManifest 中加上android:screenOrientation="portrait"來鎖螢幕

        // 1020 score
        scoreLabel = (TextView) findViewById(R.id.scoreLabel);
        // 這行還不能用，因為 game#2後半的FrameLayout還沒做
        //startLabel = (TextView) findViewById(R.id.startLabel);


        // 1020 新招
        // init timer
        mTimer = new Timer();
        // start timer task
        // 改放在 onStart 裡
        //setTimerTask();



        //Walid: Filter initialization
        kalmanInitial();

        Button clickButton = (Button) findViewById(R.id.button);
        clickButton.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                dinoJump();

            }
        });

        Button clickButton2 = (Button) findViewById(R.id.duckbutton);
        clickButton2.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                dinoDuck();

            }
        });

        Button clickButton3 = (Button) findViewById(R.id.unduckbutton);
        clickButton3.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                dinoUnduck();

            }
        });





        dinoSprite = (ImageView) findViewById(R.id.dino_animation);
        dinoSprite.setBackgroundResource(R.drawable.dino_animation);
        dinoAnimation = (AnimationDrawable) dinoSprite.getBackground();

        dinoDuckingSprite = (ImageView) findViewById(R.id.dino_ducking_animation);
        dinoDuckingSprite.setBackgroundResource(R.drawable.dino_ducking_animation);
        dinoDuckingAnimation = (AnimationDrawable) dinoDuckingSprite.getBackground();
        dinoDuckingSprite.setVisibility(View.GONE);


        // Refer the ImageView like this
        groundSprite = (ImageView) findViewById(R.id.ground_animation);
        groundSprite.setBackgroundResource(R.drawable.ground_animation);
        groundAnimation = (AnimationDrawable) groundSprite.getBackground();
        groundSlide = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.ground_slide);
//
        cactusSprite1 = (ImageView) findViewById(R.id.obstacle_animation);
        cactusSprite1.setBackgroundResource(R.drawable.obstacle_animation);
        cactusSprite1Animation = (AnimationDrawable) cactusSprite1.getBackground();
        obstacleSlide = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.obstacle_slide);

        // 1020 bird
        birdSprite = (ImageView) findViewById(R.id.obstacle_animation2);
        birdSprite.setBackgroundResource(R.drawable.obstacle_animation2);
        birdSpriteAnimation = (AnimationDrawable) birdSprite.getBackground();
        obstacleSlide = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.obstacle_slide);


        showAnim();


        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //Sensor accelrometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor barometer = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        Sensor gravity = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        Sensor mag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        SensorEventListener sensorListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                Sensor sensor = event.sensor;


                if (sensor.getType() == Sensor.TYPE_PRESSURE) {
                    float pressure = event.values[0];
                    if (initRun) {
                        absolutePressure = pressure;
                        initRun = false;
                    }
                    alt = 44300 * (1 - Math.pow(pressure / absolutePressure, 0.19));
                    //System.out.println("pressure: "+ pressure);
                    // System.out.println("ALTITUDE:"+ (alt));
                    // System.out.println(System.currentTimeMillis());
                    isBaroReady = true;
                }
//
//                }else if (sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
////                    float accX = event.values[0];// - gravX;
////                    float accY = event.values[1];// - gravY;
////                    float accZ = event.values[2];// - gravZ;
////                    // System.out.println("ACCELERATION:"+accZ);
////                    System.out.println("ACCELERATIONX:"+accX);
////                    System.out.println("ACCELERATIONY:"+accY);
////                    System.out.println("ACCELERATIONZ:"+accZ);
////                    total = accZ;
//                    //isAccReady = true;



                        if ((gravityValues != null) && (magneticValues != null)
                                && (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION)) {

                            float[] deviceRelativeAcceleration = new float[4];
                            deviceRelativeAcceleration[0] = event.values[0];
                            deviceRelativeAcceleration[1] = event.values[1];
                            deviceRelativeAcceleration[2] = event.values[2];
                            deviceRelativeAcceleration[3] = 0;

                            // Change the device relative acceleration values to earth relative values
                            // X axis -> East
                            // Y axis -> North Pole
                            // Z axis -> Sky

                            float[] R = new float[16], I = new float[16], earthAcc = new float[16];

                            SensorManager.getRotationMatrix(R, I, gravityValues, magneticValues);

                            float[] inv = new float[16];

                            android.opengl.Matrix.invertM(inv, 0, R, 0);
                            android.opengl.Matrix.multiplyMV(earthAcc, 0, inv, 0, deviceRelativeAcceleration, 0);
                          // Log.d("Acceleration", "Values: ( " + earthAcc[0] + ", " + earthAcc[1] + ", " + earthAcc[2] + " )");
                            if (earthAcc[2] < -1 && !isJumping && !isDucking ) isPrepareJump = true;
                            if (earthAcc[2] > 3.9 && isPrepareJump && !isJumping && !isDucking){
                                dinoJump();
                                isPrepareJump = false;
                                isDucking = false;
                            }

                            total = earthAcc[2];
                            isAccReady = true;

                        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
                            gravityValues = event.values;
                        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                            magneticValues = event.values;
                        }


              if (isAccReady && isBaroReady) {
                   kalmanFilterAltitude((float) total, (float) alt);
                   //System.out.print("Kalman H: "+ filter.getStateEstimation()[0]);
                  //TODO
                  //System.out.println(": Kalman V: "+ filter.getStateEstimation()[1]);
                 // System.out.println("Kalman= " + altitude);
                 // System.out.println("alt= " + absolutePressure);
                //   System.out.println(System.currentTimeMillis());

                  if (filter.getStateEstimation()[1] < -0.8 && !isDucking && !isJumping) {
                      dinoDuck();
                  }else if (filter.getStateEstimation()[1] > 0.5 && isDucking){
                      dinoUnduck();
                  }

                  isBaroReady = false;
                  isAccReady = false;
                  }

              }

            public void onAccuracyChanged(Sensor sensor, int accuracy) {
            }



        };


        sensorManager.registerListener(sensorListener, sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION), SensorManager.SENSOR_DELAY_NORMAL );
        sensorManager.registerListener(sensorListener, sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY), SensorManager.SENSOR_DELAY_NORMAL );
        sensorManager.registerListener(sensorListener, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(sensorListener, sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE), SensorManager.SENSOR_DELAY_NORMAL);
    }

    // 1020 20:00
    @Override
    protected void onStart() {
        super.onStart();
        Log.i(TAG, "On Start .....");

        // start timer task
        setTimerTask();
    }


    // 新招 https://blog.csdn.net/zuolongsnail/article/details/8168689
    // scoring function
    private void setTimerTask() {
        mTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                hitCheck();
                //Log.i(TAG, "Timer Run!");
                scoreTime += 1;
                scoreLabel.setText("Score : " + scoreTime);
            }
        }, 0, 543/* 表示0毫秒之後，每隔20毫秒執行一次 */);
    }

    // game over 結束score累加用的
    @Override
    protected void onDestroy() {
        super.onDestroy();
        // cancel timer
        mTimer.cancel();
    }


    // game#6
    public void hitCheck(){

        //ImageView groundSprite;
        //ImageView cactusSprite1;
        //ImageView dinoSprite;
        //ImageView dinoDuckingSprite;
        //ImageView dinoJumpSprite;

        // dino position
        float dinoTop = 0;
        float dinoBottom = 0;
        float dinoRight = 0;
        float dinoLeft = 0;
        Log.i(TAG, "hitCheck!");

        /*
        // ducking
        if (isDucking){
            dinoTop= dinoDuckingSprite.getTop();
            dinoBottom = dinoDuckingSprite.getBottom();
            dinoRight = dinoDuckingSprite.getRight();
            dinoLeft = dinoDuckingSprite.getLeft();

        }
        // jumping
        else if (isJumping){
            dinoTop= dinoJumpSprite.getTop();
            dinoBottom = dinoJumpSprite.getBottom();
            dinoRight = dinoJumpSprite.getRight();
            dinoLeft = dinoJumpSprite.getLeft();

        }
        // walking
        else {
            // y range
            dinoTop= dinoSprite.getTop();
            dinoBottom = dinoSprite.getBottom();
            // x position
            dinoRight = dinoSprite.getRight();
            dinoLeft = dinoSprite.getLeft();

        }*/


        // ducking
        if (isDucking){
            dinoTop= dinoDuckingSprite.getY();
            dinoLeft = dinoDuckingSprite.getX();
            dinoBottom = dinoTop + (dinoDuckingSprite.getBottom()-dinoDuckingSprite.getTop());
            dinoRight = dinoLeft + (dinoDuckingSprite.getRight()-dinoDuckingSprite.getLeft());
        }
        // jumping
        else if (isJumping){
            dinoTop= dinoJumpSprite.getY();
            dinoLeft = dinoJumpSprite.getX();
            dinoBottom = dinoTop + (dinoJumpSprite.getBottom()-dinoJumpSprite.getTop());
            dinoRight = dinoLeft + (dinoJumpSprite.getRight()-dinoJumpSprite.getLeft());
        }
        // walking
        else {
            dinoTop= dinoSprite.getY();
            dinoLeft = dinoSprite.getX();
            dinoBottom = dinoTop + (dinoSprite.getBottom()-dinoSprite.getTop());
            dinoRight = dinoLeft + (dinoSprite.getRight()-dinoSprite.getLeft());
        }

        // different obstacle

        //cactusSprite1
        //float cactusSprite1Top= cactusSprite1.getTop();
        //float cactusSprite1Bottom= cactusSprite1.getBottom();
        //float cactusSprite1Right= cactusSprite1.getRight();
        //float cactusSprite1Left= cactusSprite1.getLeft();

        float cactusSprite1Top = cactusSprite1.getY();
        float cactusSprite1Left = cactusSprite1.getX();
        float cactusSprite1Bottom = cactusSprite1Top + cactusSprite1.getBottom()-cactusSprite1.getTop();
        float cactusSprite1Right = cactusSprite1Left + cactusSprite1.getRight()-cactusSprite1.getLeft();

        System.out.println(dinoTop);
        System.out.println(dinoBottom);
        System.out.println(dinoLeft);
        System.out.println(dinoRight);
        System.out.println(cactusSprite1Top);
        System.out.println(cactusSprite1Bottom);
        System.out.println(cactusSprite1Left);
        System.out.println(cactusSprite1Right);

        // validate collision
        if (! (dinoRight<cactusSprite1Left||dinoLeft>cactusSprite1Right)) {
            if (! (dinoBottom<cactusSprite1Top||dinoTop>cactusSprite1Bottom)) {
                // collide !!
                // game over
                Log.i(TAG, "Game Over!");
                scoreTime -= 1000;
            }

        }



    }





    private void dinoDuck(){
        isDucking = true;
        //
        isJumping = false;

        dinoDuckingAnimation.start();
        dinoDuckingSprite.setVisibility(View.VISIBLE);
        dinoAnimation.stop();
        dinoSprite.setVisibility(View.GONE);
    }

    private void dinoUnduck(){
        isDucking = false;
        //
        isJumping = false;
        dinoDuckingAnimation.stop();
        dinoDuckingSprite.setVisibility(View.GONE);
        dinoAnimation.start();
        dinoSprite.setVisibility(View.VISIBLE);
    }

    /*
    private void dinoJump() {
        isJumping = true;
        //
        isDucking = false;
        dinoJumpSprite = (ImageView) findViewById(R.id.dino_jump_animation);
        dinoJumpSprite.setBackgroundResource(R.drawable.dino_jump_animation);
        dinoJumpAnimation = (AnimationDrawable) dinoJumpSprite.getBackground();
        dinoJump = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.dino_jump);

        dinoSprite.setVisibility(View.GONE);
        dinoDuckingSprite.setVisibility(View.GONE);

        dinoJumpSprite.startAnimation(dinoJump);

        dinoJump.setAnimationListener(new Animation.AnimationListener(){
            @Override
            public void onAnimationStart(Animation arg0) {
                dinoSprite.setVisibility(View.GONE);
            }
            @Override
            public void onAnimationRepeat(Animation arg0) {
            }
            @Override
            public void onAnimationEnd(Animation arg0) {
                dinoSprite.setVisibility(View.VISIBLE);
                dinoJumpSprite.setVisibility(View.GONE);
                isJumping = false;
            }
        });

    }*/

    private void dinoJump() {
        isJumping = true;
        dinoJumpSprite = (ImageView) findViewById(R.id.dino_jump_animation);
        dinoJumpSprite.setBackgroundResource(R.drawable.dino_jump_animation);
        dinoJumpAnimation = (AnimationDrawable) dinoJumpSprite.getBackground();
        dinoJump = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.dino_jump);

        dinoSprite.setVisibility(View.GONE);
        dinoDuckingSprite.setVisibility(View.GONE);

        //dinoJumpSprite.startAnimation(dinoJump);


        float y = dinoSprite.getY();

        ValueAnimator animator = ValueAnimator.ofFloat(y, y-300);
        animator.setDuration(350);
        animator.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator animation) {
                dinoJumpSprite.setY((Float) animation.getAnimatedValue());
                //System.out.println(dinoJumpSprite.getY());
            }
        });

        ValueAnimator animator2 = ValueAnimator.ofFloat(y-300, y);
        animator2.setDuration(350);
        animator2.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator animation) {
                dinoJumpSprite.setY((Float) animation.getAnimatedValue());
                //System.out.println(dinoJumpSprite.getY());
            }
        });

        animator.addListener(new AnimatorListenerAdapter() {
            @Override
            public void onAnimationStart(Animator animation) {

                dinoSprite.setVisibility(View.GONE);
                dinoJumpSprite.setVisibility(View.VISIBLE);
                isJumping = true;
            }
        });

        animator2.addListener(new AnimatorListenerAdapter() {
            @Override
            public void onAnimationEnd(Animator animation) {

                dinoSprite.setVisibility(View.VISIBLE);
                dinoJumpSprite.setVisibility(View.GONE);
                isJumping = false;
            }
        });
        AnimatorSet set = new AnimatorSet();
        set.playSequentially(animator, animator2);


        set.start();

    }







    public void showAnim() {
        dinoAnimation.start();
        groundSprite.startAnimation(groundSlide);
        cactusSprite1.startAnimation(obstacleSlide);
        // 1020
        birdSprite.startAnimation(obstacleSlide);
    }

    public void kalmanInitial(){
        // discrete time interval
        double dt = 0.178d;
        // position measurement noise (meter)
        double measurementNoise = 10d;
        // acceleration noise (meter/sec^2)
        double accelNoise = 0.2d;

        // A = [ 1 dt ]
        //     [ 0  1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1, dt }, { 0, 1 } });
        // B = [ dt^2/2 ]
        //     [ dt     ]
        RealMatrix B = new Array2DRowRealMatrix(
                new double[][] { { FastMath.pow(dt, 2d) / 2d }, { dt } });
        // H = [ 1 0 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[][] { { 1d, 0d } });
        //System.out.println(H.toString());
        // x = [ 0 0 ]
        RealVector x = new ArrayRealVector(new double[] { 0, 0 });

        // Q = [ dt^4/4 dt^3/2 ]
        //     [ dt^3/2 dt^2   ]
        RealMatrix tmp = new Array2DRowRealMatrix(
                new double[][] { { FastMath.pow(dt, 4d) / 4d, FastMath.pow(dt, 3d) / 2d },
                        { FastMath.pow(dt, 3d) / 2d, FastMath.pow(dt, 2d) } });
        RealMatrix Q = tmp.scalarMultiply(FastMath.pow(accelNoise, 2));

        // P0 = [ 1 1 ]
        //      [ 1 1 ]
        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1 }, { 1, 1 } });

        // R = [ measurementNoise^2 ]
        RealMatrix R = new Array2DRowRealMatrix(
                new double[] { FastMath.pow(measurementNoise, 2) });

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        filter = new KalmanFilter(pm, mm);
    }

    public void kalmanFilterAltitude(float accelerometer, float barometer) {
        // simulates a vehicle, accelerating at a constant rate (0.1 m/s)

        // constant control input, increase velocity by 0.1 m/s per cycle
        // Walid: Acceleration Input from Accelerometer
        RealVector u = new ArrayRealVector(new double[] { accelerometer });
        RealVector z = new ArrayRealVector(new double[] { barometer});

        filter.predict(u);

            // Simulate the process
           // RealVector pNoise = tmpPNoise.mapMultiply(accelNoise * rand.nextGaussian());

            // x = A * x + B * u + pNoise
            //x = A.operate(x).add(B.operate(u)).add(pNoise);

            // Simulate the measurement
            //double mNoise = measurementNoise * rand.nextGaussian();

            // z = H * x + m_noise
            //RealVector z = H.operate(x).mapAdd(mNoise);
            //RealVector z = new ArrayRealVector(new double[] { barometer});

        filter.correct(z);

            // state estimate shouldn't be larger than the measurement noise
            //double diff = FastMath.abs(x.getEntry(0) - filter.getStateEstimation()[0]);
            //Assert.assertTrue(Precision.compareTo(diff, measurementNoise, 1e-6) < 0);
        }

        // error covariance of the velocity should be already very low (< 0.1)
       // Assert.assertTrue(Precision.compareTo(filter.getErrorCovariance()[1][1],
              //  0.1d, 1e-6) < 0);

    }









