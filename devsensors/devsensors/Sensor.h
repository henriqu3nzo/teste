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

#pragma once

#include <android/hardware/sensors/2.1/types.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;


struct SensorData {
    float pm25;
    float mq131;

    // Construtor para inicializar os valores com 0
    SensorData() : pm25(0.0), mq131(0.0) {}
};



namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

class ISensorsEventCallback {
  public:
    virtual ~ISensorsEventCallback(){};
    virtual void postEvents(const std::vector<Event>& events, bool wakeup) = 0;
};

class Sensor {
  public:
    Sensor(int32_t sensorHandle, ISensorsEventCallback* callback);
    virtual ~Sensor();

    const SensorInfo& getSensorInfo() const;
    void batch(int64_t samplingPeriodNs);
    virtual void activate(bool enable);
    Result flush();

    void setOperationMode(OperationMode mode);
    bool supportsDataInjection() const;
    Result injectEvent(const Event& event);

  protected:
    void run();
    virtual std::vector<Event> readEvents();
    static void startThread(Sensor* sensor);

    bool isWakeUpSensor();

    bool mIsEnabled;
    int64_t mSamplingPeriodNs;
    int64_t mLastSampleTimeNs;
    SensorInfo mSensorInfo;

    std::atomic_bool mStopThread;
    std::condition_variable mWaitCV;
    std::mutex mRunMutex;
    std::thread mRunThread;

    ISensorsEventCallback* mCallback;

    OperationMode mMode;
};

class OnChangeSensor : public Sensor {
  public:
    OnChangeSensor(int32_t sensorHandle, ISensorsEventCallback* callback);

    virtual void activate(bool enable) override;

  protected:
    virtual std::vector<Event> readEvents() override;

  protected:
    Event mPreviousEvent;
    bool mPreviousEventSet;
};

class ContinuousSensor : public Sensor {
  public:
    ContinuousSensor(int32_t sensorHandle, ISensorsEventCallback* callback);
};

class SensorMq7 : public OnChangeSensor {
public:
    SensorMq7(int32_t sensorHandle, ISensorsEventCallback* callback);

protected:
    std::vector<Event> readEvents() override;

protected:
    float mq7Read = 0;
    static int64_t timevalToNano(timeval const& t) {
        return t.tv_sec * 1000000000LL + t.tv_usec * 1000;
    }
};

class SensorPm25 : public OnChangeSensor {
public:
    SensorPm25(int32_t sensorHandle, ISensorsEventCallback* callback);

protected:
    std::vector<Event> readEvents() override;

protected:
    float pm25Read = 0;

    static int64_t timevalToNano(timeval const& t) {
        return t.tv_sec * 1000000000LL + t.tv_usec * 1000;
    }
};

class SensorMq131 : public OnChangeSensor {
public:
    SensorMq131(int32_t sensorHandle, ISensorsEventCallback* callback);

protected:
    std::vector<Event> readEvents() override;

protected:
    float mq131Read = 0;
    static int64_t timevalToNano(timeval const& t) {
        return t.tv_sec * 1000000000LL + t.tv_usec * 1000;
    }
};


}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
