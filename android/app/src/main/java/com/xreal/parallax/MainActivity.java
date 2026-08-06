package com.xreal.parallax;

import android.app.Activity;
import android.app.Presentation;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.display.DisplayManager;
import android.os.Bundle;
import android.view.Display;
import android.view.Window;
import android.view.WindowManager;

public final class MainActivity extends Activity implements SensorEventListener {
    private DisplayManager displayManager;
    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private ParallaxView mainView;
    private ParallaxPresentation presentation;

    private final DisplayManager.DisplayListener displayListener =
            new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                    updatePresentation();
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                    updatePresentation();
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    updatePresentation();
                }
            };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mainView = new ParallaxView(this);
        setContentView(mainView);

        displayManager = (DisplayManager) getSystemService(Context.DISPLAY_SERVICE);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (rotationSensor != null) {
            sensorManager.registerListener(
                    this,
                    rotationSensor,
                    SensorManager.SENSOR_DELAY_GAME);
        }
        displayManager.registerDisplayListener(displayListener, null);
        updatePresentation();
    }

    @Override
    protected void onPause() {
        displayManager.unregisterDisplayListener(displayListener);
        sensorManager.unregisterListener(this);
        dismissPresentation();
        super.onPause();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != Sensor.TYPE_ROTATION_VECTOR) {
            return;
        }

        float[] matrix = new float[9];
        float[] orientation = new float[3];
        SensorManager.getRotationMatrixFromVector(matrix, event.values);
        SensorManager.getOrientation(matrix, orientation);

        float yaw = orientation[0];
        float pitch = orientation[1];
        mainView.setHeadPose(yaw, pitch);
        if (presentation != null) {
            presentation.setHeadPose(yaw, pitch);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    private void updatePresentation() {
        Display[] displays = displayManager.getDisplays(DisplayManager.DISPLAY_CATEGORY_PRESENTATION);
        Display target = displays.length > 0 ? displays[0] : null;

        if (target == null) {
            dismissPresentation();
            mainView.setStatus("Phone or DeX display");
            return;
        }

        if (presentation != null && presentation.getDisplay().getDisplayId() == target.getDisplayId()) {
            return;
        }

        dismissPresentation();
        presentation = new ParallaxPresentation(this, target);
        presentation.show();
        mainView.setStatus("External display active");
    }

    private void dismissPresentation() {
        if (presentation != null) {
            presentation.dismiss();
            presentation = null;
        }
    }

    private static final class ParallaxPresentation extends Presentation {
        private final ParallaxView view;

        ParallaxPresentation(Context context, Display display) {
            super(context, display);
            view = new ParallaxView(context);
            view.setStatus("External XREAL/DeX display");
        }

        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            setContentView(view);
        }

        void setHeadPose(float yaw, float pitch) {
            view.setHeadPose(yaw, pitch);
        }
    }
}
