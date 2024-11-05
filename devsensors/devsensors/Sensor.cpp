/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Sensor.h"

#include <hardware/sensors.h>
#include <utils/SystemClock.h>

#include <cmath>


#include <fcntl.h>
#include <linux/input.h>
#include <log/log.h>
#include <fstream>

#include <iostream>

SensorData data;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

using ::android::hardware::sensors::V1_0::MetaDataEventType;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;



Sensor::Sensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : mIsEnabled(false),
      mSamplingPeriodNs(0),
      mLastSampleTimeNs(0),
      mCallback(callback),
      mMode(OperationMode::NORMAL) {
    mSensorInfo.sensorHandle = sensorHandle;
    mSensorInfo.vendor = "Vendor String";
    mSensorInfo.version = 1;
    constexpr float kDefaultMaxDelayUs = 1000 * 1000;
    mSensorInfo.maxDelay = kDefaultMaxDelayUs;
    mSensorInfo.fifoReservedEventCount = 0;
    mSensorInfo.fifoMaxEventCount = 0;
    mSensorInfo.requiredPermission = "";
    mSensorInfo.flags = 0;
    mRunThread = std::thread(startThread, this);
}

Sensor::~Sensor() {
    // Ensure that lock is unlocked before calling mRunThread.join() or a
    // deadlock will occur.
    {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        mWaitCV.notify_all();
    }
    mRunThread.join();
}

const SensorInfo& Sensor::getSensorInfo() const {
    return mSensorInfo;
}

void Sensor::batch(int64_t samplingPeriodNs) {
    samplingPeriodNs = std::clamp(samplingPeriodNs,
                                  static_cast<int64_t>(mSensorInfo.minDelay) * 1000,
                                  static_cast<int64_t>(mSensorInfo.maxDelay) * 1000);

    if (mSamplingPeriodNs != samplingPeriodNs) {
        mSamplingPeriodNs = samplingPeriodNs;

        // Wake up the 'run' thread to check if a new event should be generated now
        mWaitCV.notify_all();
    }
}

void Sensor::activate(bool enable) {
    if (mIsEnabled != enable) {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mIsEnabled = enable;
        mWaitCV.notify_all();
    }
}

Result Sensor::flush() {
    // Only generate a flush complete event if the sensor is enabled and if the sensor is not a
    // one-shot sensor.
    if (!mIsEnabled || (mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::ONE_SHOT_MODE))) {
        return Result::BAD_VALUE;
    }

    // Note: If a sensor supports batching, write all of the currently batched events for the sensor
    // to the Event FMQ prior to writing the flush complete event.
    Event ev;
    ev.sensorHandle = mSensorInfo.sensorHandle;
    ev.sensorType = SensorType::META_DATA;
    ev.u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
    std::vector<Event> evs{ev};
    mCallback->postEvents(evs, isWakeUpSensor());

    return Result::OK;
}

void Sensor::startThread(Sensor* sensor) {
    sensor->run();
}

void Sensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);
    constexpr int64_t kNanosecondsInSeconds = 1000 * 1000 * 1000;

    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL) || mStopThread);
            });
        } else {
            timespec curTime;
            clock_gettime(CLOCK_REALTIME, &curTime);
            int64_t now = (curTime.tv_sec * kNanosecondsInSeconds) + curTime.tv_nsec;
            int64_t nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;

            if (now >= nextSampleTime) {
                mLastSampleTimeNs = now;
                nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;
                mCallback->postEvents(readEvents(), isWakeUpSensor());
            }

            mWaitCV.wait_for(runLock, std::chrono::nanoseconds(nextSampleTime - now));
        }
    }
}

bool Sensor::isWakeUpSensor() {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP);
}

std::vector<Event> Sensor::readEvents() {
    std::vector<Event> events;
    Event event;
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    event.u.vec3.x = 0;
    event.u.vec3.y = 0;
    event.u.vec3.z = 0;
    event.u.vec3.status = SensorStatus::ACCURACY_HIGH;
    events.push_back(event);
    return events;
}

void Sensor::setOperationMode(OperationMode mode) {
    if (mMode != mode) {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mMode = mode;
        mWaitCV.notify_all();
    }
}

bool Sensor::supportsDataInjection() const {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::DATA_INJECTION);
}

Result Sensor::injectEvent(const Event& event) {
    Result result = Result::OK;
    if (event.sensorType == SensorType::ADDITIONAL_INFO) {
        // When in OperationMode::NORMAL, SensorType::ADDITIONAL_INFO is used to push operation
        // environment data into the device.
    } else if (!supportsDataInjection()) {
        result = Result::INVALID_OPERATION;
    } else if (mMode == OperationMode::DATA_INJECTION) {
        mCallback->postEvents(std::vector<Event>{event}, isWakeUpSensor());
    } else {
        result = Result::BAD_VALUE;
    }
    return result;
}

OnChangeSensor::OnChangeSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback), mPreviousEventSet(false) {
    mSensorInfo.flags |= SensorFlagBits::ON_CHANGE_MODE;
}

void OnChangeSensor::activate(bool enable) {
    Sensor::activate(enable);
    if (!enable) {
        mPreviousEventSet = false;
    }
}

std::vector<Event> OnChangeSensor::readEvents() {
    //ALOGE("READ EVENT");
    std::vector<Event> events = Sensor::readEvents();
    std::vector<Event> outputEvents;

    for (auto iter = events.begin(); iter != events.end(); ++iter) {
        Event ev = *iter;
        if (ev.u.vec3 != mPreviousEvent.u.vec3 || !mPreviousEventSet) {
            outputEvents.push_back(ev);
            mPreviousEvent = ev;
            mPreviousEventSet = true;
        }
    }
    return outputEvents;
}

ContinuousSensor::ContinuousSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback) {
    mSensorInfo.flags |= SensorFlagBits::CONTINUOUS_MODE;
}



SensorMq7::SensorMq7(int32_t sensorHandle, ISensorsEventCallback* callback)
    : OnChangeSensor(sensorHandle, callback) {
    mSensorInfo.name = "Air Quality Sensor Mq7";
    mSensorInfo.vendor = "devtitans";
    mSensorInfo.type = SensorType::DEVICE_PRIVATE_BASE;  // Ajustado para múltiplos sensores
    mSensorInfo.typeAsString = "SENSOR_STRING_TYPE_AIR_QUALITY";
    mSensorInfo.maxRange = 1000.0f;  // Defina um valor adequado para a faixa máxima de leituras
    mSensorInfo.resolution = 0.01f;  // Defina uma resolução adequada
    mSensorInfo.power = 0.001f;  // mA
    mSensorInfo.minDelay = 8000000 ;  // 200 * 1000 microsegundos
}



SensorPm25::SensorPm25(int32_t sensorHandle, ISensorsEventCallback* callback)
    : OnChangeSensor(sensorHandle, callback) {
    mSensorInfo.name = "Air Quality Sensor SensorPm25";
    mSensorInfo.vendor = "devtitans";
    mSensorInfo.type = SensorType::DEVICE_PRIVATE_BASE;  // Ajustado para múltiplos sensores
    mSensorInfo.typeAsString = "SENSOR_STRING_TYPE_AIR_QUALITY";
    mSensorInfo.maxRange = 1000.0f;  // Defina um valor adequado para a faixa máxima de leituras
    mSensorInfo.resolution = 0.01f;  // Defina uma resolução adequada
    mSensorInfo.power = 0.001f;  // mA
    mSensorInfo.minDelay = 9000000;  // microsegundos
}



SensorMq131::SensorMq131(int32_t sensorHandle, ISensorsEventCallback* callback)
    : OnChangeSensor(sensorHandle, callback) {
    mSensorInfo.name = "Air Quality Sensor SensorMq131";
    mSensorInfo.vendor = "devtitans";
    mSensorInfo.type = SensorType::DEVICE_PRIVATE_BASE;  // Ajustado para múltiplos sensores
    mSensorInfo.typeAsString = "SENSOR_STRING_TYPE_AIR_QUALITY";
    mSensorInfo.maxRange = 1000.0f;  // Defina um valor adequado para a faixa máxima de leituras
    mSensorInfo.resolution = 0.01f;  // Defina uma resolução adequada
    mSensorInfo.power = 0.001f;  // mA
    mSensorInfo.minDelay = 10000000;  // microsegundos
}


float extractSensorValue(const std::string& mensure,const std::string& sensorData) {
    float value = 0.0f;

    if (!sensorData.empty()) {          // Se obteve os dados dos sensores
        // Exemplo de linha lida: "RES GET_SENSORS mq131: 25.00 | pm25: 30.00 | mq7: 40.00"
        std::stringstream ss(sensorData);
        std::string  temp;

        // Variáveis para armazenar os valores dos sensores
        float mq131_value = 0.0f, pm25_value = 0.0f, mq7_value = 0.0f;

        // Parsing da string
        getline(ss, temp, ':');         // Ignora a parte "RES GET_SENSORS mq131"
        ss >> mq131_value;              // Lê o valor de mq131
        getline(ss, temp, '|');         // Ignora até "pm25"
        ss >> temp;
        ss >> pm25_value;               // Lê o valor de pm25
        getline(ss, temp, '|');         // Ignora até "mq7"
        ss >> temp;
        ss >> mq7_value;                // Lê o valor de mq7

        if(mensure == "mq7")
            value = mq7_value;
        else if (mensure == "mq131")
            value = mq131_value;
        else if (mensure == "pm25")
            value =  pm25_value;

    }

    return value;
}



std::vector<Event> SensorMq7::readEvents() {
    std::ifstream in("/sys/kernel/iqar/sensors");
    std::vector<Event> events;
    if (in) {
        std::string sensorData;
        std::getline(in, sensorData);
        mq7Read = extractSensorValue("mq7",sensorData);
        data.pm25 = extractSensorValue("pm25",sensorData);
        data.mq131 = extractSensorValue("mq131",sensorData);
        in.close();


        Event mq7Event;
        mq7Event.timestamp = ::android::elapsedRealtimeNano();
        mq7Event.sensorHandle = mSensorInfo.sensorHandle;
        mq7Event.sensorType = mSensorInfo.type;
        mq7Event.u.scalar = mq7Read;
        events.push_back(mq7Event);

    }
    return events;
}


std::vector<Event> SensorPm25::readEvents() {
    std::vector<Event> events;

    pm25Read = data.pm25;
    Event pm25Event;
    pm25Event.timestamp = ::android::elapsedRealtimeNano();
    pm25Event.sensorHandle = mSensorInfo.sensorHandle;
    pm25Event.sensorType = mSensorInfo.type;
    pm25Event.u.scalar = pm25Read;
    events.push_back(pm25Event);

    return events;
}




std::vector<Event> SensorMq131::readEvents() {
    std::vector<Event> events;
    mq131Read = data.mq131;
    Event mq131Event;
    mq131Event.timestamp = ::android::elapsedRealtimeNano();
    mq131Event.sensorHandle = mSensorInfo.sensorHandle;
    mq131Event.sensorType = mSensorInfo.type;
    mq131Event.u.scalar = mq131Read;
    events.push_back(mq131Event);
    return events;
}


}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
