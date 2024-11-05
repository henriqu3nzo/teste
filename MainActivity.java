package com.example.sensors;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import java.util.List;

public class MainActivity extends AppCompatActivity {
    private SensorManager sensorManager;
    private TextView lightLevel;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        lightLevel = (TextView) findViewById(R.id.light_level);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> sensorList = sensorManager.getSensorList(Sensor.TYPE_DEVICE_PRIVATE_BASE);
        Sensor mq131 = sensorManager.getSensorList(Sensor.TYPE_DEVICE_PRIVATE_BASE).get(0);

        Sensor mq7 = sensorManager.getSensorList(Sensor.TYPE_DEVICE_PRIVATE_BASE).get(1);
        Sensor pm25 = sensorManager.getSensorList(Sensor.TYPE_DEVICE_PRIVATE_BASE).get(2);
        sensorManager.registerListener(listener, mq7, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(listener, mq131, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(listener, pm25, SensorManager.SENSOR_DELAY_NORMAL);

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (sensorManager != null) {
            sensorManager.unregisterListener(listener);

        }
    }

    private SensorEventListener listener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {

            Sensor source = event.sensor;


            if (source.toString().contains("SensorPm25")) {
                // do your stuff
                Log.println(Log.INFO, "LIGHT_TEST", source.getName());
                Log.println(Log.INFO, "LIGHT_TEST", String.valueOf(event.values[0]));
                lightLevel.setText("PM25 " + String.valueOf(event.values[0]));
            } else if (source.toString().contains("Mq7")) {
                Log.println(Log.INFO, "LIGHT_TEST", source.getName());
                Log.println(Log.INFO, "LIGHT_TEST", String.valueOf(event.values[0]));
//                lightLevel.setText("mq7 "+String.valueOf(event.values[0]));

            } else if (source.toString().contains("SensorMq131")) {
                Log.println(Log.INFO, "LIGHT_TEST", source.getName());
                Log.println(Log.INFO, "LIGHT_TEST", String.valueOf(event.values[0]));
//               lightLevel.setText("mq131 "+String.valueOf(event.values[0]));

            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }
    };

}