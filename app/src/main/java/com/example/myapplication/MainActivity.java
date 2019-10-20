package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.animation.PropertyValuesHolder;
import android.animation.ValueAnimator;
import android.content.Context;
import android.graphics.Rect;
import android.graphics.drawable.AnimationDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.view.animation.Interpolator;
import android.view.animation.LinearInterpolator;
import android.widget.Button;
import android.widget.ImageView;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Precision;
import org.junit.Assert;

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

    KalmanFilter filter;

    double total=0;
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
        //kalmanInitial();
        //Animation Image Views Initialization
        animationInitial();
        // Random Obstacle Generator
        randomObstacle();
        //Show Animation
        showAnim();
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
//            dinoJumpAnimation = (AnimationDrawable) dinoJumpSprite.getBackground();
//            dinoJump = AnimationUtils.loadAnimation(getApplicationContext(), R.anim.dino_jump);


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
        System.out.println(H.toString());
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
        // Walid: Acceleration Input from Accelerometer
        RealVector u = new ArrayRealVector(new double[] { accelerometer });
        RealVector z = new ArrayRealVector(new double[] { barometer});

        filter.predict(u);
        filter.correct(z);

        }
    }








