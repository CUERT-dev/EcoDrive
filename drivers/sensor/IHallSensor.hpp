#pragma once



template<uint8_t instance>
class HallSensor{
    public:

    void init(const Hall_Config cfg)
    {

    }
    uint8_t getRawValue();
    void update();
    float getElectricalAngle_rad();
    float getElectricalAnglularVelocity_radPs();

    ~HallSensor() = default;
};

