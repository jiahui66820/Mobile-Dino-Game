package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.AnimatorSet;
import android.animation.ValueAnimator;
import android.content.Context;
import android.graphics.Rect;
import android.graphics.drawable.AnimationDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.Button;
import android.widget.ImageView;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.Random;

public class MainActivity extends AppCompatActivity {

    Rect rectObstacle = new Rect();
    Rect rectJumpDino = new Rect();
    Rect rectDuckDino = new Rect();
    Rect rectDino = new Rect();

    AnimationDrawable dinoAnimation;
    AnimationDrawable dinoDuckingAnimation;
    AnimationDrawable groundAnimation;


    Animation groundSlide;
    ImageView groundSprite;
    ImageView obstacle;
    ImageView dinoSprite;
    ImageView dinoDuckingSprite;
    ImageView dinoJumpSprite;

    private double absolutePressure;
    private boolean initRun = true;
    private float[] gravityValues = null;
    private float[] magneticValues = null;
    KalmanFilter filter;

    double absoluteAccZ =0;
    double alt=0;

    boolean isBaroReady = false;
    boolean isAccReady = false;

    boolean isJumping;
    boolean isPrepareJump = false;
    boolean isDucking;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //Debug buttons (Jump, Duck, Unduck)
        buttons();
        //Kalman Filter Initialization
        filter = KalmanFilter.kalmanInitial();
        //Animation Image Views Initialization
        animationInitial();
        // Random Obstacle Generator
        randomObstacle();
        //Show Animation
        showAnim();
        //Start Sensors
        sensorActivity();
    }

    private void sensorActivity() {
        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
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
                    //Convert Pressure to Altitude
                    alt = 44300 * (1 - Math.pow(pressure / absolutePressure, 0.19));
                    isBaroReady = true;
                }
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

                    absoluteAccZ = earthAcc[2];
                    isAccReady = true;

                } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
                    gravityValues = event.values;
                } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    magneticValues = event.values;
                }

                if (isAccReady && isBaroReady) {
                    kalmanFilterAltitude((float) absoluteAccZ, (float) alt);
                    //System.out.print("Kalman H: "+ filter.getStateEstimation()[0]);
                    //System.out.println(": Kalman V: "+ filter.getStateEstimation()[1]);

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

    public void showAnim() {
        dinoAnimation.start();
        groundSprite.startAnimation(groundSlide);
        ValueAnimator obstacleAnimation = ValueAnimator.ofFloat(1200, -300);
        obstacleAnimation.setDuration(2600);
        obstacleAnimation.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator animation) {
                obstacle.setX((Float) animation.getAnimatedValue());
                sprite2rect(rectObstacle, obstacle);
                sprite2rect(rectDino, dinoSprite);

                if (isCollisionDetected()){
                    gameOver();
                }

            }
        });
        obstacleAnimation.start();
        obstacleAnimation.addListener(new AnimatorListenerAdapter() {
            @Override
            public void onAnimationEnd(Animator animation) {
                randomObstacle();
                animation.start();
            }
        });
    }

    private void gameOver() {
        //Game Over method
        //Stop all animation
        // Add big Eye Dino at exact same last lacation
        //Display game over
        //Display restart
        //Save high score
    }

    private boolean isCollisionDetected() {
        if (!isJumping && !isDucking){
            if (Rect.intersects(rectObstacle,rectDino)){
                System.out.println("Collision! Standing");
                return true;
            }
        } else if (isJumping) {
            if (Rect.intersects(rectObstacle, rectJumpDino)) {
                System.out.println("Collision! Jumping");
                return true;
            }
        }
        else if (isDucking){
            if (Rect.intersects(rectObstacle,rectDuckDino)){
                System.out.println("Collision! Ducking");
                return true;
            }
        }
        return false;
    }


    private void animationInitial() {
        //Animate Dino
        dinoSprite = (ImageView) findViewById(R.id.dino_animation);
        dinoSprite.setBackgroundResource(R.drawable.dino_animation);
        dinoAnimation = (AnimationDrawable) dinoSprite.getBackground();
        //Animate Dino Ducking
        dinoDuckingSprite = (ImageView) findViewById(R.id.dino_ducking_animation);
        dinoDuckingSprite.setBackgroundResource(R.drawable.dino_ducking_animation);
        dinoDuckingAnimation = (AnimationDrawable) dinoDuckingSprite.getBackground();
        dinoDuckingSprite.setVisibility(View.GONE);
        // Animate Ground
        groundSprite = (ImageView) findViewById(R.id.ground_animation);
        groundSprite.setBackgroundResource(R.drawable.ground_animation);
        groundAnimation = (AnimationDrawable) groundSprite.getBackground();
        groundSlide = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.ground_slide);
    }

    private void buttons() {
        Button jumpButton = (Button) findViewById(R.id.button);
        jumpButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dinoJump();
            }
        });
        Button duckButton = (Button) findViewById(R.id.duckbutton);
        duckButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dinoDuck();
            }
        });
        Button unduckButton = (Button) findViewById(R.id.unduckbutton);
        unduckButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dinoUnduck();
            }
        });
    }

    private void sprite2rect(Rect rectangle, ImageView animation) {
        rectangle.left = (int) animation.getX();
        rectangle.top = (int) animation.getY();
        rectangle.bottom = (int) (animation.getY() + animation.getWidth());
        rectangle.right = (int) (animation.getX() + animation.getWidth());
    }

    private void randomObstacle() {
        Random random = new Random();
        //Generate random number between 1 and 5
        int randomNumber = random.nextInt(5 - 1 + 1) + 1;
        //System.out.println(randomNumber);
        switch (randomNumber) {
            case 1:
                obstacle = (ImageView) findViewById(R.id.obstacle1_animation);
                obstacle.setBackgroundResource(R.drawable.obstacle1_animation);
                break;
            case 2:
                obstacle = (ImageView) findViewById(R.id.obstacle2_animation);
                obstacle.setBackgroundResource(R.drawable.obstacle2_animation);
                break;
            case 3:
                obstacle = (ImageView) findViewById(R.id.obstacle3_animation);
                obstacle.setBackgroundResource(R.drawable.obstacle3_animation);
                break;
            case 4:
                obstacle = (ImageView) findViewById(R.id.obstacle4_animation);
                obstacle.setBackgroundResource(R.drawable.obstacle4_animation);
                break;
            case 5:
                obstacle = (ImageView) findViewById(R.id.obstacle5_animation);
                obstacle.setBackgroundResource(R.drawable.obstacle5_animation);
                break;
        }
    }

    private void dinoDuck(){
        isDucking = true;
        dinoDuckingAnimation.start();
        dinoDuckingSprite.setVisibility(View.VISIBLE);
        dinoAnimation.stop();
        dinoSprite.setVisibility(View.GONE);
        //Create Dino Ducking Rectangle
        rectDuckDino.left = (int) dinoDuckingSprite.getX();
        rectDuckDino.top = (int) dinoDuckingSprite.getY();
        rectDuckDino.bottom = (int) (dinoDuckingSprite.getY() + dinoDuckingSprite.getWidth());
        rectDuckDino.right = (int) (dinoDuckingSprite.getX() + dinoDuckingSprite.getWidth());
    }

    private void dinoUnduck(){
        isDucking = false;
        dinoDuckingAnimation.stop();
        dinoDuckingSprite.setVisibility(View.GONE);
        dinoAnimation.start();
        dinoSprite.setVisibility(View.VISIBLE);
    }

    private void dinoJump() {
            dinoJumpSprite = (ImageView) findViewById(R.id.dino_jump_animation);
            dinoJumpSprite.setBackgroundResource(R.drawable.dino_jump_animation);
            float y = dinoSprite.getY();
            ValueAnimator jumpUpAnimation = ValueAnimator.ofFloat(y, y-350);
            jumpUpAnimation.setDuration(350);
            jumpUpAnimation.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
                @Override
                public void onAnimationUpdate(ValueAnimator animation) {
                    dinoJumpSprite.setY((Float) animation.getAnimatedValue());
                    sprite2rect(rectJumpDino, dinoJumpSprite);
                }
            });
            ValueAnimator jumpDownAnimation = ValueAnimator.ofFloat(y-350, y);
            jumpDownAnimation.setDuration(350);
            jumpDownAnimation.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
                @Override
                public void onAnimationUpdate(ValueAnimator animation) {
                    dinoJumpSprite.setY((Float) animation.getAnimatedValue());
                    sprite2rect(rectJumpDino, dinoJumpSprite);
                }
            });
            jumpUpAnimation.addListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationStart(Animator animation) {
                    isJumping = true;
                    isDucking = false;
                    dinoSprite.setVisibility(View.GONE);
                    dinoDuckingSprite.setVisibility(View.GONE);
                    dinoJumpSprite.setVisibility(View.VISIBLE);
                }
            });
            jumpDownAnimation.addListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    isJumping = false;
                    dinoSprite.setVisibility(View.VISIBLE);
                    dinoJumpSprite.setVisibility(View.GONE);
                }
            });
            AnimatorSet set = new AnimatorSet();
            set.playSequentially(jumpUpAnimation, jumpDownAnimation);
            set.start();
    }

    public void kalmanFilterAltitude(float accelerometer, float barometer) {
        // Walid: Acceleration Input from Accelerometer
        RealVector u = new ArrayRealVector(new double[] { accelerometer });
        RealVector z = new ArrayRealVector(new double[] { barometer});

        filter.predict(u);
        filter.correct(z);
        }
    }








