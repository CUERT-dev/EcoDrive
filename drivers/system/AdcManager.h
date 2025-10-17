#pragma once

#include <cstdint>
#include <functional>

namespace EcoDrive {
namespace Drivers {

/**
 * @brief Interface for ADC management in motor control applications
 * 
 * Provides a comprehensive interface for managing multiple ADC channels
 * with support for DMA, priority-based sampling, and filtering.
 */
class IAdcManager {
public:
    /// ADC channel configuration
    struct ChannelConfig {
        uint8_t channelId;           ///< Physical ADC channel number
        uint32_t sampleRate;         ///< Desired sampling rate in Hz
        uint8_t priority;            ///< Sampling priority (0 highest)
        bool useDma;                 ///< Enable DMA for this channel
        uint8_t oversampleRatio;     ///< Oversampling ratio for noise reduction
    };

    /// ADC sample with metadata
    struct Sample {
        uint16_t value;             ///< Raw ADC valu
        uint8_t channelId;          ///< Channel that produced the sample
        bool valid;                 ///< Indicates if the sample is valid
    };

    using SampleCallback = std::function<void(const Sample&)>;
    
    virtual ~IAdcManager() = default;

    /**
     * @brief Initialize the ADC manager
     * @return true if initialization successful
     */
    virtual bool initialize() = 0;

    /**
     * @brief Configure an ADC channel
     * @param config Channel configuration parameters
     * @return true if configuration successful
     */
    virtual bool configureChannel(const ChannelConfig& config) = 0;

    /**
     * @brief Start sampling on all configured channels
     * @return true if sampling started successfully
     */
    virtual bool startSampling() = 0;

    /**
     * @brief Stop sampling on all channels
     */
    virtual void stopSampling() = 0;

    /**
     * @brief Get the latest sample from a specific channel
     * @param channelId The channel to read from
     * @return Latest sample from the channel
     */
    virtual Sample getLastSample(uint8_t channelId) = 0;

    /**
     * @brief Register a callback for new samples on a channel
     * @param channelId Channel to monitor
     * @param callback Function to call when new sample is available
     */
    virtual void registerSampleCallback(uint8_t channelId, SampleCallback callback) = 0;

    /**
     * @brief Get the current sampling rate for a channel
     * @param channelId Channel to query
     * @return Current sampling rate in Hz
     */
    virtual uint32_t getCurrentSampleRate(uint8_t channelId) = 0;

    /**
     * @brief Check if a channel is healthy and producing valid samples
     * @param channelId Channel to check
     * @return true if channel is working correctly
     */
    virtual bool isChannelHealthy(uint8_t channelId) = 0;
};


