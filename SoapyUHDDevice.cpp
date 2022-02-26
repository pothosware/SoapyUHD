// Copyright (c) 2014-2020 Josh Blum
//               2019-2022 Nicholas Corgan
// SPDX-License-Identifier: GPL-3.0

/***********************************************************************
 * A Soapy module that supports UHD devices within the Soapy API.
 **********************************************************************/

#include "TypeHelpers.hpp"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <uhd/version.hpp>
#include <uhd/device.hpp>
#ifdef UHD_HAS_MSG_HPP
#include <uhd/utils/msg.hpp>
#else
#include <uhd/utils/log_add.hpp>
#endif
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/version.hpp>
#include <algorithm>
#include <cctype>
#include <functional>
#include <iostream>
#include <iterator>
#include <unordered_map>

/***********************************************************************
 * Stream wrapper
 **********************************************************************/
struct SoapyUHDStream
{
    uhd::rx_streamer::sptr rx;
    uhd::tx_streamer::sptr tx;
};

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapyUHDDevice : public SoapySDR::Device
{
public:
    SoapyUHDDevice(uhd::usrp::multi_usrp::sptr dev, const SoapySDR::Kwargs &args):
        _dev(dev),
        _type(args.at("type")),
        _isNetworkDevice(args.count("addr") != 0)
    {
        if (args.count("rx_subdev") != 0) _dev->set_rx_subdev_spec(args.at("rx_subdev"));
        if (args.count("tx_subdev") != 0) _dev->set_tx_subdev_spec(args.at("tx_subdev"));

        this->__createSettingsStructs();
    }

    /*******************************************************************
     * Identification API
     ******************************************************************/
    std::string getDriverKey(void) const
    {
        return _type;
    }

    std::string getHardwareKey(void) const
    {
        return _dev->get_mboard_name();
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        SoapySDR::Kwargs out;
        for (size_t i = 0; i < this->getNumChannels(SOAPY_SDR_TX); i++)
        {
            const uhd::dict<std::string, std::string> info = _dev->get_usrp_tx_info(i);
            for (const std::string &key : info.keys())
            {
                if (key.size() > 3 and key.substr(0, 3) == "tx_")
                    out[str(boost::format("tx%d_%s") % i % key.substr(3))] = info[key];
                else out[key] = info[key];
            }
        }
        for (size_t i = 0; i < this->getNumChannels(SOAPY_SDR_RX); i++)
        {
            const uhd::dict<std::string, std::string> info = _dev->get_usrp_rx_info(i);
            for (const std::string &key : info.keys())
            {
                if (key.size() > 3 and key.substr(0, 3) == "rx_")
                    out[str(boost::format("rx%d_%s") % i % key.substr(3))] = info[key];
                else out[key] = info[key];
            }
        }

        uhd::property_tree::sptr tree = _get_tree();
        if (tree->exists("/mboards/0/fw_version")) out["fw_version"] = tree->access<std::string>("/mboards/0/fw_version").get();
        if (tree->exists("/mboards/0/fpga_version")) out["fpga_version"] = tree->access<std::string>("/mboards/0/fpga_version").get();

        return out;
    }

    /*******************************************************************
     * Channels support
     ******************************************************************/
    void setFrontendMapping(const int dir, const std::string &mapping)
    {
        if (dir == SOAPY_SDR_TX) return _dev->set_tx_subdev_spec(mapping);
        if (dir == SOAPY_SDR_RX) return _dev->set_rx_subdev_spec(mapping);
    }

    std::string getFrontendMapping(const int dir) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_subdev_spec().to_string();
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_subdev_spec().to_string();
        return SoapySDR::Device::getFrontendMapping(dir);
    }

    size_t getNumChannels(const int dir) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_num_channels();
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_num_channels();
        return SoapySDR::Device::getNumChannels(dir);
    }

    /*******************************************************************
     * Stream support
     ******************************************************************/
    std::vector<std::string> getStreamFormats(const int, const size_t) const
    {
        std::vector<std::string> formats;
        formats.push_back("CS8");
        formats.push_back("CS12");
        formats.push_back("CS16");
        formats.push_back("CF32");
        formats.push_back("CF64");
        return formats;
    }

    std::string getNativeStreamFormat(const int, const size_t, double &fullScale) const
    {
        fullScale = (1 << 15);
        return "CS16";
    }

    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t) const
    {
        SoapySDR::ArgInfoList streamArgs;

        SoapySDR::ArgInfo sppArg;
        sppArg.key = "spp";
        sppArg.value = "0";
        sppArg.name = "Samples per packet";
        sppArg.description = "The number of samples per packet.";
        sppArg.units = "samples";
        sppArg.type = SoapySDR::ArgInfo::INT;
        streamArgs.push_back(sppArg);

        SoapySDR::ArgInfo wireFormatArg;
        wireFormatArg.key = "WIRE";
        wireFormatArg.value = "";
        wireFormatArg.name = "Bus format";
        wireFormatArg.description = "The format of samples over the bus.";
        wireFormatArg.type = SoapySDR::ArgInfo::STRING;
        wireFormatArg.options.push_back("sc8");
        wireFormatArg.options.push_back("sc16");
        wireFormatArg.optionNames.push_back("Complex bytes");
        wireFormatArg.optionNames.push_back("Complex shorts");
        streamArgs.push_back(wireFormatArg);

        SoapySDR::ArgInfo peakArgs;
        peakArgs.key = "peak";
        peakArgs.value = "1.0";
        peakArgs.name = "Peak value";
        peakArgs.description = "The peak value for scaling in complex byte mode.";
        peakArgs.type = SoapySDR::ArgInfo::FLOAT;
        streamArgs.push_back(peakArgs);

        const std::string key = (direction == SOAPY_SDR_RX)?"recv":"send";
        const std::string name = (direction == SOAPY_SDR_RX)?"Receive":"Send";

        SoapySDR::ArgInfo bufSizeArgs;
        bufSizeArgs.key = key+"_buff_size";
        bufSizeArgs.value = "0";
        bufSizeArgs.name = name + " socket buffer size";
        bufSizeArgs.description = "The size of the kernel socket buffer in bytes. Use 0 for automatic.";
        bufSizeArgs.units = "bytes";
        bufSizeArgs.type = SoapySDR::ArgInfo::INT;
        if (_isNetworkDevice) streamArgs.push_back(bufSizeArgs);

        SoapySDR::ArgInfo frameSizeArgs;
        frameSizeArgs.key = key+"_frame_size";
        frameSizeArgs.value = "";
        frameSizeArgs.name = name + " frame buffer size";
        frameSizeArgs.description = "The size an individual datagram or frame in bytes.";
        frameSizeArgs.units = "bytes";
        frameSizeArgs.type = SoapySDR::ArgInfo::INT;
        streamArgs.push_back(frameSizeArgs);

        SoapySDR::ArgInfo numFrameArgs;
        numFrameArgs.key = "num_"+key+"_frames";
        numFrameArgs.value = "";
        numFrameArgs.name = name + " number of buffers";
        numFrameArgs.description = "The number of available buffers.";
        numFrameArgs.units = "buffers";
        numFrameArgs.type = SoapySDR::ArgInfo::INT;
        streamArgs.push_back(numFrameArgs);

        SoapySDR::ArgInfo fullscaleArgs;
        fullscaleArgs.key = "fullscale";
        fullscaleArgs.value = "1.0";
        fullscaleArgs.name = "Full-scale amplitude";
        fullscaleArgs.description = "Specifies the full-scale amplitude when using floats (not supported for all devices).";
        fullscaleArgs.type = SoapySDR::ArgInfo::FLOAT;
        streamArgs.push_back(fullscaleArgs);

        if(direction == SOAPY_SDR_TX)
        {
            SoapySDR::ArgInfo underflowPolicyArg;
            underflowPolicyArg.key = "underflow_policy";
            underflowPolicyArg.name = "Underflow policy";
            underflowPolicyArg.description = "How the TX DSP should recover from underflow (not supported for all devices).";
            underflowPolicyArg.type = SoapySDR::ArgInfo::STRING;
            underflowPolicyArg.options.push_back("next_burst");
            underflowPolicyArg.options.push_back("next_packet");
            underflowPolicyArg.optionNames.push_back("Next burst");
            underflowPolicyArg.optionNames.push_back("Next packet");
            streamArgs.push_back(underflowPolicyArg);
        }

        return streamArgs;
    }

    SoapySDR::Stream *setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
    {
        std::string hostFormat;
        for(const char ch : format)
        {
            if (ch == 'C') hostFormat += "c";
            else if (ch == 'F') hostFormat = "f" + hostFormat;
            else if (ch == 'S') hostFormat = "s" + hostFormat;
            else if (std::isdigit(ch)) hostFormat += ch;
            else throw std::runtime_error("SoapyUHDDevice::setupStream("+format+") unknown format");
        }

        //convert input to stream args
        uhd::stream_args_t stream_args(hostFormat);
        stream_args.channels = channels;
        stream_args.args = kwargsToDict(args);
        if (args.count("WIRE") != 0) stream_args.otw_format = args.at("WIRE");

        //create streamers
        SoapyUHDStream *stream = new SoapyUHDStream();
        if (dir == SOAPY_SDR_TX) stream->tx = _dev->get_tx_stream(stream_args);
        if (dir == SOAPY_SDR_RX) stream->rx = _dev->get_rx_stream(stream_args);
        return reinterpret_cast<SoapySDR::Stream *>(stream);
    }

    void closeStream(SoapySDR::Stream *handle)
    {
        SoapyUHDStream *stream = reinterpret_cast<SoapyUHDStream *>(handle);
        delete stream;
    }

    size_t getStreamMTU(SoapySDR::Stream *handle) const
    {
        SoapyUHDStream *stream = reinterpret_cast<SoapyUHDStream *>(handle);
        if (stream->rx) return stream->rx->get_max_num_samps();
        if (stream->tx) return stream->tx->get_max_num_samps();
        return SoapySDR::Device::getStreamMTU(handle);
    }

    int activateStream(SoapySDR::Stream *handle, const int flags, const long long timeNs, const size_t numElems)
    {
        SoapyUHDStream *stream = reinterpret_cast<SoapyUHDStream *>(handle);
        if (not stream->rx) return 0; //NOP, does nothing, but not an error

        //determine stream mode
        uhd::stream_cmd_t::stream_mode_t mode;
        if (numElems == 0) mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
        else if ((flags & SOAPY_SDR_END_BURST) != 0) mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
        else mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE;

        //fill in the command
        uhd::stream_cmd_t cmd(mode);
        cmd.stream_now = (flags & SOAPY_SDR_HAS_TIME) == 0;
        cmd.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);
        cmd.num_samps = numElems;

        //issue command
        stream->rx->issue_stream_cmd(cmd);
        return 0;
    }

    int deactivateStream(SoapySDR::Stream *handle, const int flags, const long long timeNs)
    {
        SoapyUHDStream *stream = reinterpret_cast<SoapyUHDStream *>(handle);
        if (not stream->rx) return 0; //NOP, does nothing, but not an error

        //stop the stream (stop mode might support a timestamp)
        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        cmd.stream_now = (flags & SOAPY_SDR_HAS_TIME) == 0;
        cmd.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);

        //issue command
        stream->rx->issue_stream_cmd(cmd);
        return 0;
    }

    int readStream(SoapySDR::Stream *handle, void * const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs)
    {
        uhd::rx_streamer::sptr &stream = reinterpret_cast<SoapyUHDStream *>(handle)->rx;

        //receive into buffers and metadata
        uhd::rx_metadata_t md;
        uhd::rx_streamer::buffs_type stream_buffs(buffs, stream->get_num_channels());
        int ret = stream->recv(stream_buffs, numElems, md, timeoutUs/1e6, (flags & SOAPY_SDR_ONE_PACKET) != 0);

        //parse the metadata
        flags = 0;
        if (md.has_time_spec) flags |= SOAPY_SDR_HAS_TIME;
        if (md.end_of_burst) flags |= SOAPY_SDR_END_BURST;
        if (md.more_fragments) flags |= SOAPY_SDR_MORE_FRAGMENTS;
        timeNs = md.time_spec.to_ticks(1e9);
        switch (md.error_code)
        {
        case uhd::rx_metadata_t::ERROR_CODE_NONE: return ret;
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW: return SOAPY_SDR_OVERFLOW;
        case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT: return SOAPY_SDR_TIMEOUT;
        case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET: return SOAPY_SDR_CORRUPTION;
        case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT: return SOAPY_SDR_CORRUPTION;
        case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND: return SOAPY_SDR_STREAM_ERROR;
        case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN: return SOAPY_SDR_STREAM_ERROR;
        }
        return ret;
    }

    int writeStream(SoapySDR::Stream *handle, const void * const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs)
    {
        uhd::tx_streamer::sptr &stream = reinterpret_cast<SoapyUHDStream *>(handle)->tx;

        //load metadata
        uhd::tx_metadata_t md;
        md.has_time_spec = (flags & SOAPY_SDR_HAS_TIME) != 0;
        md.end_of_burst = (flags & SOAPY_SDR_END_BURST) != 0;
        md.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);

        //send buffers and metadata
        uhd::tx_streamer::buffs_type stream_buffs(buffs, stream->get_num_channels());
        int ret = stream->send(stream_buffs, numElems, md, timeoutUs/1e6);

        flags = 0;
        //consider a return of 0 to be a complete timeout
        if (ret == 0) return SOAPY_SDR_TIMEOUT;
        return ret;
    }

    int readStreamStatus(SoapySDR::Stream *handle, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs)
    {
        if (reinterpret_cast<SoapyUHDStream *>(handle)->rx) return SOAPY_SDR_NOT_SUPPORTED;
        uhd::tx_streamer::sptr &stream = reinterpret_cast<SoapyUHDStream *>(handle)->tx;

        uhd::async_metadata_t md;
        if (not stream->recv_async_msg(md, timeoutUs/1e6)) return SOAPY_SDR_TIMEOUT;

        chanMask = (1 << md.channel);
        flags = 0;
        if (md.has_time_spec) flags |= SOAPY_SDR_HAS_TIME;
        timeNs = md.time_spec.to_ticks(1e9);

        switch (md.event_code)
        {
        case uhd::async_metadata_t::EVENT_CODE_BURST_ACK: flags |= SOAPY_SDR_END_BURST; break;
        case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW: return SOAPY_SDR_UNDERFLOW;
        case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR: return SOAPY_SDR_CORRUPTION;
        case uhd::async_metadata_t::EVENT_CODE_TIME_ERROR: return SOAPY_SDR_TIME_ERROR;
        case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET: return SOAPY_SDR_UNDERFLOW;
        case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST: return SOAPY_SDR_CORRUPTION;
        case uhd::async_metadata_t::EVENT_CODE_USER_PAYLOAD: break;
        }
        return 0;
    }

    /*******************************************************************
     * Antenna support
     ******************************************************************/

    std::vector<std::string> listAntennas(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_antennas(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_antennas(channel);
        return SoapySDR::Device::listAntennas(dir, channel);
    }

    void setAntenna(const int dir, const size_t channel, const std::string &name)
    {
        if (dir == SOAPY_SDR_TX) _dev->set_tx_antenna(name, channel);
        if (dir == SOAPY_SDR_RX) _dev->set_rx_antenna(name, channel);
    }

    std::string getAntenna(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_antenna(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_antenna(channel);
        return SoapySDR::Device::getAntenna(dir, channel);
    }

    /*******************************************************************
     * Frontend corrections support
     ******************************************************************/

    bool hasDCOffsetMode(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return false;
        if (dir == SOAPY_SDR_RX)
        {
            // This is usually on the motherboard's layer, but for some devices
            // (ex. the B2XX), it is done on the RF frontend.
            return __doesMBoardFEPropTreeEntryExist(dir, channel, "dc_offset/enable") ||
                   __doesDBoardFEPropTreeEntryExist(dir, channel, "dc_offset/enable");
        }

        return SoapySDR::Device::hasDCOffsetMode(dir, channel);
    }

    void setDCOffsetMode(const int dir, const size_t channel, const bool automatic)
    {
        if (dir == SOAPY_SDR_RX) _dev->set_rx_dc_offset(automatic, channel);
    }

    bool getDCOffsetMode(const int dir, const size_t channel) const
    {
        // multi_usrp has no getter for this, so we need to query the
        // property tree itself.
        if (dir == SOAPY_SDR_TX) return false;
        if((dir == SOAPY_SDR_RX) && this->hasDCOffsetMode(dir, channel))
        {
            auto tree = _get_tree();
            const std::string subpath = "/dc_offset/enable";

            auto mboardPath = __getMBoardFEPropTreePath(dir, channel) + subpath;
            if(tree->exists(mboardPath))
            {
                return tree->access<bool>(mboardPath).get();
            }

            auto dboardPath = __getDBoardFEPropTreePath(dir, channel) + subpath;
            if(tree->exists(dboardPath))
            {
                return tree->access<bool>(dboardPath).get();
            }
        }

        return SoapySDR::Device::getDCOffsetMode(dir, channel);
    }

    bool hasDCOffset(const int dir, const size_t channel) const
    {
        return __doesMBoardFEPropTreeEntryExist(dir, channel, "dc_offset/value");
    }

    void setDCOffset(const int dir, const size_t channel, const std::complex<double> &offset)
    {
        if (dir == SOAPY_SDR_TX) _dev->set_tx_dc_offset(offset, channel);
        if (dir == SOAPY_SDR_RX) _dev->set_rx_dc_offset(offset, channel);
    }

    std::complex<double> getDCOffset(const int dir, const size_t channel) const
    {
        // multi_usrp has no getter for this, so we need to query the
        // property tree itself.
        if(this->hasDCOffset(dir, channel))
        {
            auto tree = _get_tree();
            const std::string subpath = "/dc_offset/value";

            auto path = __getMBoardFEPropTreePath(dir, channel) + subpath;
            return tree->access<std::complex<double>>(path).get();
        }

        return SoapySDR::Device::getDCOffset(dir, channel);
    }

    bool hasIQBalance(const int dir, const size_t channel) const
    {
        return __doesMBoardFEPropTreeEntryExist(dir, channel, "iq_balance/value");
    }

    void setIQBalance(const int dir, const size_t channel, const std::complex<double> &balance)
    {
        if (dir == SOAPY_SDR_TX) _dev->set_tx_iq_balance(balance, channel);
        if (dir == SOAPY_SDR_RX) _dev->set_rx_iq_balance(balance, channel);
    }

    std::complex<double> getIQBalance(const int dir, const size_t channel) const
    {
        // multi_usrp has no getter for this, so we need to query the
        // property tree itself.
        if(this->hasIQBalance(dir, channel))
        {
            auto tree = _get_tree();
            const std::string subpath = "/iq_balance/value";

            auto path = __getMBoardFEPropTreePath(dir, channel) + subpath;
            return tree->access<std::complex<double>>(path).get();
        }

        return SoapySDR::Device::getIQBalance(dir, channel);
    }

    bool hasIQBalanceMode(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return false;
        if (dir == SOAPY_SDR_RX)
        {
            return __doesMBoardFEPropTreeEntryExist(dir, channel, "iq_balance/enable");
        }

        return SoapySDR::Device::hasDCOffsetMode(dir, channel);
    }

    void setIQBalanceMode(const int dir, const size_t channel, const bool automatic)
    {
        if (dir == SOAPY_SDR_RX) _dev->set_rx_iq_balance(automatic, channel);
    }

    bool getIQBalanceMode(const int dir, const size_t channel) const
    {
        // multi_usrp has no getter for this, so we need to query the
        // property tree itself.
        if (dir == SOAPY_SDR_TX) return false;
        if((dir == SOAPY_SDR_RX) && this->hasIQBalanceMode(dir, channel))
        {
            auto tree = _get_tree();
            const std::string subpath = "/iq_balance/enable";

            auto path = __getMBoardFEPropTreePath(dir, channel) + subpath;
            return tree->access<bool>(path).get();
        }

        return false;
    }

    /*******************************************************************
     * Gain support
     ******************************************************************/

    std::vector<std::string> listGains(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_gain_names(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_gain_names(channel);
        return SoapySDR::Device::listGains(dir, channel);
    }

    bool hasGainMode(const int dir, const size_t channel) const
    {
        #ifdef UHD_HAS_SET_RX_AGC
        if (dir == SOAPY_SDR_TX) return false;
        if (dir == SOAPY_SDR_RX)
        {
            return __doesDBoardFEPropTreeEntryExist(dir, channel, "gain/agc/enable");
        }
        #endif
        return SoapySDR::Device::hasGainMode(dir, channel);
    }

    void setGainMode(const int dir, const size_t channel, const bool automatic)
    {
        #ifdef UHD_HAS_SET_RX_AGC
        if (dir == SOAPY_SDR_RX) return _dev->set_rx_agc(automatic, channel);
        #endif
        return SoapySDR::Device::setGainMode(dir, channel, automatic);
    }

    void setGain(const int dir, const size_t channel, const double value)
    {
        if (dir == SOAPY_SDR_TX) _dev->set_tx_gain(value, channel);
        if (dir == SOAPY_SDR_RX) _dev->set_rx_gain(value, channel);
    }

    void setGain(const int dir, const size_t channel, const std::string &name, const double value)
    {
        if (dir == SOAPY_SDR_TX) _dev->set_tx_gain(value, name, channel);
        if (dir == SOAPY_SDR_RX) _dev->set_rx_gain(value, name, channel);
    }

    double getGain(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_gain(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_gain(channel);
        return SoapySDR::Device::getGain(dir, channel);
    }

    double getGain(const int dir, const size_t channel, const std::string &name) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_gain(name, channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_gain(name, channel);
        return SoapySDR::Device::getGain(dir, channel, name);
    }

    SoapySDR::Range getGainRange(const int dir, const size_t channel, const std::string &name) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToRange(_dev->get_tx_gain_range(name, channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToRange(_dev->get_rx_gain_range(name, channel));
        return SoapySDR::Device::getGainRange(dir, channel, name);
    }

    SoapySDR::Range getGainRange(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToRange(_dev->get_tx_gain_range(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToRange(_dev->get_rx_gain_range(channel));
        return SoapySDR::Device::getGainRange(dir, channel);
    }

    /*******************************************************************
     * Frequency support
     ******************************************************************/

    void setFrequency(const int dir, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
    {
        uhd::tune_request_t tr(frequency);

        if (args.count("OFFSET") != 0)
        {
            tr = uhd::tune_request_t(frequency, boost::lexical_cast<double>(args.at("OFFSET")));
        }
        if (args.count("RF") != 0)
        {
            try
            {
                tr.rf_freq = boost::lexical_cast<double>(args.at("RF"));
                tr.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
            }
            catch (...)
            {
                tr.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
            }
        }
        if (args.count("BB") != 0)
        {
            try
            {
                tr.dsp_freq = boost::lexical_cast<double>(args.at("BB"));
                tr.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
            }
            catch (...)
            {
                tr.dsp_freq_policy = uhd::tune_request_t::POLICY_NONE;
            }
        }
        tr.args = kwargsToDict(args);

        if (dir == SOAPY_SDR_TX) _trCache[dir][channel] = _dev->set_tx_freq(tr, channel);
        if (dir == SOAPY_SDR_RX) _trCache[dir][channel] = _dev->set_rx_freq(tr, channel);
    }

    void setFrequency(const int dir, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
    {
        //use tune request to get individual elements -- could use property tree here
        uhd::tune_request_t tr(frequency);
        tr.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
        tr.dsp_freq_policy = uhd::tune_request_t::POLICY_NONE;
        tr.args = kwargsToDict(args);
        if (name == "RF")
        {
            tr.rf_freq = frequency;
            tr.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        }
        if (name == "BB")
        {
            tr.dsp_freq = frequency;
            tr.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        }

        if (dir == SOAPY_SDR_TX) _trCache[dir][channel] = _dev->set_tx_freq(tr, channel);
        if (dir == SOAPY_SDR_RX) _trCache[dir][channel] = _dev->set_rx_freq(tr, channel);
    }

    std::map<int, std::map<size_t, uhd::tune_result_t> > _trCache;

    double getFrequency(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_freq(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_freq(channel);
        return SoapySDR::Device::getFrequency(dir, channel);
    }

    double getFrequency(const int dir, const size_t channel, const std::string &name) const
    {
        //we have never tuned before, return the overall freq for RF, assume 0.0 for all else
        if (_trCache.count(dir) == 0 or _trCache.at(dir).count(channel) == 0)
        {
            if (name == "RF") return this->getFrequency(dir, channel);
            else return 0.0;
        }

        const uhd::tune_result_t tr = _trCache.at(dir).at(channel);
        if (name == "RF") return tr.actual_rf_freq;
        if (name == "BB") return tr.actual_dsp_freq;
        return SoapySDR::Device::getFrequency(dir, channel, name);
    }

    std::vector<std::string> listFrequencies(const int, const size_t) const
    {
        std::vector<std::string> comps;
        comps.push_back("RF");
        comps.push_back("BB");
        return comps;
    }

    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToRangeList(_dev->get_tx_freq_range(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToRangeList(_dev->get_rx_freq_range(channel));
        return SoapySDR::Device::getFrequencyRange(dir, channel);
    }

    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t channel, const std::string &name) const
    {
        if (name == "RF")
        {
            //use overall range - could use property tree, but close enough
            if (dir == SOAPY_SDR_TX) return metaRangeToRangeList(_dev->get_tx_freq_range(channel));
            if (dir == SOAPY_SDR_RX) return metaRangeToRangeList(_dev->get_rx_freq_range(channel));
        }
        if (name == "BB")
        {
            //read the range from the property tree
            uhd::property_tree::sptr tree = _get_tree();
            const std::string path = str(boost::format("/mboards/0/%s_dsps/%u/freq/range") % ((dir == SOAPY_SDR_TX)?"tx":"rx") % channel);
            if (tree->exists(path)) return metaRangeToRangeList(tree->access<uhd::meta_range_t>(path).get());
            else return SoapySDR::RangeList(1, SoapySDR::Range(-getSampleRate(dir, channel)/2, getSampleRate(dir, channel)/2));
        }
        return SoapySDR::Device::getFrequencyRange(dir, channel, name);
    }

    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int, const size_t) const
    {
        SoapySDR::ArgInfoList frequencyArgs;

        SoapySDR::ArgInfo modeNArg;
        modeNArg.key = "mode_n";
        modeNArg.name = "N divider";
        modeNArg.description = "Whether the daughterboard tune code should use an integer N divider or fractional N divider (not supported for all devices).";
        modeNArg.type = SoapySDR::ArgInfo::STRING;
        modeNArg.options.push_back("integer");
        modeNArg.options.push_back("fractional");
        modeNArg.optionNames.push_back("Integer");
        modeNArg.optionNames.push_back("Fractional");
        frequencyArgs.emplace_back(std::move(modeNArg));

        SoapySDR::ArgInfo intNStepArg;
        intNStepArg.key = "int_n_step";
        intNStepArg.name = "Integer-N tuning step";
        intNStepArg.description = "The step between valid tunable frequencies when using integer-N tuning (not supported for all devices).";
        intNStepArg.type = SoapySDR::ArgInfo::FLOAT;
        frequencyArgs.emplace_back(std::move(intNStepArg));

        return frequencyArgs;
    }

    /*******************************************************************
     * Sample Rate support
     ******************************************************************/

    void setSampleRate(const int dir, const size_t channel, const double rate)
    {
        if (dir == SOAPY_SDR_TX) return _dev->set_tx_rate(rate, channel);
        if (dir == SOAPY_SDR_RX) return _dev->set_rx_rate(rate, channel);
    }

    double getSampleRate(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_rate(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_rate(channel);
        return SoapySDR::Device::getSampleRate(dir, channel);
    }

    std::vector<double> listSampleRates(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToNumericList(_dev->get_tx_rates(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToNumericList(_dev->get_rx_rates(channel));
        return SoapySDR::Device::listSampleRates(dir, channel);
    }

    SoapySDR::RangeList getSampleRateRange(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToRangeList(_dev->get_tx_rates(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToRangeList(_dev->get_rx_rates(channel));
        return SoapySDR::Device::getSampleRateRange(dir, channel);
    }

    void setBandwidth(const int dir, const size_t channel, const double bw)
    {
        if (dir == SOAPY_SDR_TX) return _dev->set_tx_bandwidth(bw, channel);
        if (dir == SOAPY_SDR_RX) return _dev->set_rx_bandwidth(bw, channel);
    }

    double getBandwidth(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_bandwidth(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_bandwidth(channel);
        return SoapySDR::Device::getBandwidth(dir, channel);
    }

    std::vector<double> listBandwidths(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToNumericList(_dev->get_tx_bandwidth_range(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToNumericList(_dev->get_rx_bandwidth_range(channel));
        return SoapySDR::Device::listBandwidths(dir, channel);
    }

    SoapySDR::RangeList getBandwidthRange(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return metaRangeToRangeList(_dev->get_tx_bandwidth_range(channel));
        if (dir == SOAPY_SDR_RX) return metaRangeToRangeList(_dev->get_rx_bandwidth_range(channel));
        return SoapySDR::Device::getBandwidthRange(dir, channel);
    }

    /*******************************************************************
     * Clocking support
     ******************************************************************/

    void setMasterClockRate(const double rate)
    {
        _dev->set_master_clock_rate(rate);
    }

    double getMasterClockRate(void) const
    {
        return _dev->get_master_clock_rate();
    }

    std::vector<std::string> listClockSources(void) const
    {
        return _dev->get_clock_sources(0);
    }

    void setClockSource(const std::string &source)
    {
        _dev->set_clock_source(source, 0);
    }

    std::string getClockSource(void) const
    {
        return _dev->get_clock_source(0);
    }

    std::vector<std::string> listTimeSources(void) const
    {
        return _dev->get_time_sources(0);
    }

    void setTimeSource(const std::string &source)
    {
        _dev->set_time_source(source, 0);
    }

    std::string getTimeSource(void) const
    {
        return _dev->get_time_source(0);
    }

    /*******************************************************************
     * Time support
     ******************************************************************/

    bool hasHardwareTime(const std::string &what) const
    {
        return (what == "PPS" or what.empty());
    }

    long long getHardwareTime(const std::string &what) const
    {
        if (what == "PPS") return _dev->get_time_last_pps().to_ticks(1e9);
        return _dev->get_time_now().to_ticks(1e9);
    }

    void setHardwareTime(const long long timeNs, const std::string &what)
    {
        uhd::time_spec_t time = uhd::time_spec_t::from_ticks(timeNs, 1e9);
        if (what == "PPS") return _dev->set_time_next_pps(time);
        if (what == "UNKNOWN_PPS") return _dev->set_time_unknown_pps(time);
        if (what == "CMD" and timeNs == 0) return _dev->clear_command_time();
        if (what == "CMD") return _dev->set_command_time(time);
        return _dev->set_time_now(time);
    }

    //deprecated call, just forwards to setHardwareTime with CMD arg
    void setCommandTime(const long long timeNs, const std::string &)
    {
        this->setHardwareTime(timeNs, "CMD");
    }

    /*******************************************************************
     * Sensor support
     ******************************************************************/

    std::vector<std::string> listSensors(void) const
    {
        return _dev->get_mboard_sensor_names();
    }

    SoapySDR::ArgInfo getSensorInfo(const std::string &name) const
    {
        return sensorToArgInfo(_dev->get_mboard_sensor(name), name);
    }

    std::string readSensor(const std::string &name) const
    {
        return _dev->get_mboard_sensor(name).value;
    }

    std::vector<std::string> listSensors(const int dir, const size_t channel) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_sensor_names(channel);
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_sensor_names(channel);
        return SoapySDR::Device::listSensors(dir, channel);
    }

    SoapySDR::ArgInfo getSensorInfo(const int dir, const size_t channel, const std::string &name) const
    {
        if (dir == SOAPY_SDR_TX) return sensorToArgInfo(_dev->get_tx_sensor(name, channel), name);
        if (dir == SOAPY_SDR_RX) return sensorToArgInfo(_dev->get_rx_sensor(name, channel), name);
        return SoapySDR::Device::getSensorInfo(dir, channel, name);
    }

    std::string readSensor(const int dir, const size_t channel, const std::string &name) const
    {
        if (dir == SOAPY_SDR_TX) return _dev->get_tx_sensor(name, channel).value;
        if (dir == SOAPY_SDR_RX) return _dev->get_rx_sensor(name, channel).value;
        return SoapySDR::Device::readSensor(dir, channel, name);
    }

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const
    {
        SoapySDR::ArgInfoList argInfoList;
        std::transform(
            _settings.begin(),
            _settings.end(),
            std::back_inserter(argInfoList),
            [](const std::pair<std::string, Setting> &mapPair)
            {
                auto argInfo = mapPair.second.argInfo;
                if(mapPair.second.queryRangeAtRuntime and mapPair.second.rangeGetter)
                    argInfo.range = mapPair.second.rangeGetter();

                return argInfo;
            });

        return argInfoList;
    }

    SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const
    {
        SoapySDR::ArgInfoList argInfoList;
        const std::vector<std::unordered_map<std::string, ChannelSetting>>* channelSettings = nullptr;

        switch(direction)
        {
        case SOAPY_SDR_TX:
            channelSettings = &_txChannelSettings;
            break;

        case SOAPY_SDR_RX:
            channelSettings = &_rxChannelSettings;
            break;

        default:
            break;
        }

        if(channelSettings and (channel < channelSettings->size()))
        {
            std::transform(
                channelSettings->at(channel).begin(),
                channelSettings->at(channel).end(),
                std::back_inserter(argInfoList),
                [channel](const std::pair<std::string, ChannelSetting> &mapPair)
                {
                    auto argInfo = mapPair.second.argInfo;
                    if(mapPair.second.queryRangeAtRuntime and mapPair.second.rangeGetter)
                        argInfo.range = mapPair.second.rangeGetter(channel);

                    return argInfo;
                });
        }

        return argInfoList;
    }

    /*******************************************************************
     * GPIO API
     ******************************************************************/

    /*!
     * Helpful markup to set the ATRs and CTRL from the writeGPIO API.
     * dev->writeGPIO("BANKFOO:ATR_TX", 0xffff);
     */
    void __splitBankName(const std::string &name, std::string &bank, std::string &attr)
    {
        const size_t sepPos = name.find(':');
        if (sepPos == std::string::npos)
        {
            bank = name;
            attr = "OUT";
            return;
        }
        bank = name.substr(0, sepPos);
        attr = name.substr(sepPos+1);
    }

    std::vector<std::string> listGPIOBanks(void) const
    {
        return _dev->get_gpio_banks(0);
    }

    void writeGPIO(const std::string &name, const unsigned value)
    {
        std::string bank, attr; __splitBankName(name, bank, attr);
        _dev->set_gpio_attr(bank, attr, value);
    }

    void writeGPIO(const std::string &name, const unsigned value, const unsigned mask)
    {
        std::string bank, attr; __splitBankName(name, bank, attr);
        _dev->set_gpio_attr(bank, attr, value, mask);
    }

    unsigned readGPIO(const std::string &bank) const
    {
        return _dev->get_gpio_attr(bank, "READBACK");
    }

    void writeGPIODir(const std::string &bank, const unsigned dir)
    {
        _dev->set_gpio_attr(bank, "DDR", dir);
    }

    void writeGPIODir(const std::string &bank, const unsigned dir, const unsigned mask)
    {
        _dev->set_gpio_attr(bank, "DDR", dir, mask);
    }

    unsigned readGPIODir(const std::string &bank) const
    {
        return _dev->get_gpio_attr(bank, "DDR");
    }

    /*******************************************************************
     * Get handle to underlying device
     ******************************************************************/

    void* getNativeDeviceHandle(void) const
    {
        return _dev.get();
    }

    /*******************************************************************
     * Helpers for searching property tree
     ******************************************************************/

    std::string __getMBoardFEPropTreePath(const int dir, const size_t channel) const
    {
        auto tree = _get_tree();
        const std::string directionName = (dir == SOAPY_SDR_TX) ? "tx" : "rx";
        auto subdevSpec = (dir == SOAPY_SDR_TX) ? _dev->get_tx_subdev_spec(0).at(channel)
                                                : _dev->get_rx_subdev_spec(0).at(channel);

        const std::string path =
            str(boost::format("/mboards/0/%s_frontends/%s")
                % directionName
                % subdevSpec.db_name);

        return path;
    }

    std::string __getDBoardFEPropTreePath(const int dir, const size_t channel) const
    {
        auto tree = _get_tree();
        const std::string directionName = (dir == SOAPY_SDR_TX) ? "tx" : "rx";
        auto subdevSpec = (dir == SOAPY_SDR_TX) ? _dev->get_tx_subdev_spec(0).at(channel)
                                                : _dev->get_rx_subdev_spec(0).at(channel);

        const std::string path =
            str(boost::format("/mboards/0/dboards/%s/%s_frontends/%s")
                % subdevSpec.db_name
                % directionName
                % subdevSpec.sd_name);

        return path;
    }

    bool __doesMBoardFEPropTreeEntryExist(const int dir, const size_t channel, const std::string &subpath) const
    {
        auto path = __getMBoardFEPropTreePath(dir, channel) + "/" + subpath;

        return _get_tree()->exists(path);
    }

    bool __doesDBoardFEPropTreeEntryExist(const int dir, const size_t channel, const std::string &subpath) const
    {
        auto path = __getDBoardFEPropTreePath(dir, channel) + "/" + subpath;

        return _get_tree()->exists(path);
    }

    /*******************************************************************
     * Deriving non-trivial settings lists on construction
     ******************************************************************/

    void __createSettingsStructs(void)
    {
        //
        // Global
        //

#ifdef UHD_HAS_SYNC_SOURCE_OUT
        // multi_usrp has no getter or way to query if this capability is
        // supported, so we need to dive into the property tree directly.
        {
            static const auto clock_source_out_path = "/mboards/0/clock_source/output";
            static const auto time_source_out_path = "/mboards/0/time_source/output";

            auto tree = this->_get_tree();
            if(tree->exists(clock_source_out_path))
            {
                std::string key = "clock_source_out";

                Setting setting
                {
                    {},
                    [this]() -> std::string
                    {
                        return SoapySDR::SettingToString(this->_get_tree()->access<bool>(clock_source_out_path).get());
                    },
                    [this](const std::string &value) -> void
                    {
                        _dev->set_clock_source_out(SoapySDR::StringToSetting<bool>(value));
                    }
                };
                setting.argInfo.key = key;
                setting.argInfo.value = "true"; // Enabled by default
                setting.argInfo.name = "Output clock source?";
                setting.argInfo.description = "Whether or not to send the clock signal to the device's output connector";
                setting.argInfo.type = SoapySDR::ArgInfo::BOOL;

                _settings.emplace(
                    std::move(key),
                    std::move(setting));
            }
            if(tree->exists(time_source_out_path))
            {
                std::string key = "time_source_out";

                Setting setting
                {
                    {},
                    [this]() -> std::string
                    {
                        return SoapySDR::SettingToString(this->_get_tree()->access<bool>(time_source_out_path).get());
                    },
                    [this](const std::string &value) -> void
                    {
                        _dev->set_time_source_out(SoapySDR::StringToSetting<bool>(value));
                    }
                };
                setting.argInfo.key = key;
                setting.argInfo.value = "true"; // Enabled by default
                setting.argInfo.name = "Output time source?";
                setting.argInfo.description = "Whether or not to send the time signal (PPS) to the device's output connector";
                setting.argInfo.type = SoapySDR::ArgInfo::BOOL;

                _settings.emplace(
                    std::move(key),
                    std::move(setting));
            }
        }
#endif

        //
        // Per channel
        //

        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_TX); ++chan)
            _txChannelSettings.emplace_back();
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_RX); ++chan)
            _rxChannelSettings.emplace_back();

#ifdef UHD_HAS_TX_LO_CONFIG
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_TX); ++chan)
        {
            auto &channelSettings = _txChannelSettings[chan];

            auto chanTxLoNames = _dev->get_tx_lo_names(chan);
            chanTxLoNames.emplace_back(uhd::usrp::multi_usrp::ALL_LOS);

            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            for(const auto &txLoName: chanTxLoNames)
            {
                auto sourceSettingName = txLoName + "_lo_source";
                auto exportEnabledSettingName = txLoName + "_lo_export_enabled";
                auto loFreqSettingName = txLoName + "_lo_freq";

                ChannelSetting sourceSetting
                {
                    {},
                    [txLoName, this](const size_t chan) -> std::string
                    {
                        return _dev->get_tx_lo_source(txLoName, chan);
                    },
                    [txLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_tx_lo_source(value, txLoName, chan);
                    }
                };
                sourceSetting.argInfo.key = sourceSetting.argInfo.name = sourceSettingName;
                sourceSetting.argInfo.description = "The TX LO source for the "+txLoName+" LO stage";
                sourceSetting.argInfo.type = SoapySDR::ArgInfo::STRING;

                ChannelSetting exportEnabledSetting
                {
                    {},
                    [txLoName, this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_tx_lo_export_enabled(txLoName, chan));
                    },
                    [txLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_tx_lo_export_enabled(
                            SoapySDR::StringToSetting<bool>(value),
                            txLoName,
                            chan);
                    }
                };
                exportEnabledSetting.argInfo.key = exportEnabledSetting.argInfo.name = exportEnabledSettingName;
                exportEnabledSetting.argInfo.description = "Whether or not the "+txLoName+" LO is exported";
                exportEnabledSetting.argInfo.type = SoapySDR::ArgInfo::BOOL;

                ChannelSetting loFreqSetting
                {
                    {},
                    [txLoName, this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_tx_lo_freq(txLoName, chan));
                    },
                    [txLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        (void)_dev->set_tx_lo_freq(
                            SoapySDR::StringToSetting<double>(value),
                            txLoName,
                            chan);
                    }
                };
                loFreqSetting.argInfo.key = loFreqSetting.argInfo.name = loFreqSettingName;
                loFreqSetting.argInfo.description = "LO frequency";
                loFreqSetting.argInfo.type = SoapySDR::ArgInfo::FLOAT;
                exportEnabledSetting.argInfo.range = metaRangeToRange(_dev->get_tx_lo_freq_range(txLoName, chan));

                channelSettings.emplace(
                    std::move(sourceSettingName),
                    std::move(sourceSetting));
                channelSettings.emplace(
                    std::move(exportEnabledSettingName),
                    std::move(exportEnabledSetting));
                channelSettings.emplace(
                    std::move(loFreqSettingName),
                    std::move(loFreqSetting));
            }
        }
#endif
#ifdef UHD_USRP_MULTI_USRP_LO_CONFIG_API
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_RX); ++chan)
        {
            auto &channelSettings = _rxChannelSettings[chan];

            auto chanRxLoNames = _dev->get_rx_lo_names(chan);
            chanRxLoNames.emplace_back(uhd::usrp::multi_usrp::ALL_LOS);

            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            for(const auto &rxLoName: chanRxLoNames)
            {
                auto sourceSettingName = rxLoName + "_lo_source";
                auto exportEnabledSettingName = rxLoName + "_lo_export_enabled";
                auto loFreqSettingName = rxLoName + "_lo_freq";

                ChannelSetting sourceSetting
                {
                    {},
                    [rxLoName, this](const size_t chan) -> std::string
                    {
                        return _dev->get_rx_lo_source(rxLoName, chan);
                    },
                    [rxLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_rx_lo_source(value, rxLoName, chan);
                    }
                };
                sourceSetting.argInfo.key = sourceSetting.argInfo.name = sourceSettingName;
                sourceSetting.argInfo.description = "The RX LO source for the "+rxLoName+" LO stage";
                sourceSetting.argInfo.type = SoapySDR::ArgInfo::STRING;

                ChannelSetting exportEnabledSetting
                {
                    {},
                    [rxLoName, this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_rx_lo_export_enabled(rxLoName, chan));
                    },
                    [rxLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_rx_lo_export_enabled(
                            SoapySDR::StringToSetting<bool>(value),
                            rxLoName,
                            chan);
                    }
                };
                exportEnabledSetting.argInfo.key = exportEnabledSetting.argInfo.name = exportEnabledSettingName;
                exportEnabledSetting.argInfo.description = "Whether or not the "+rxLoName+" LO is exported";
                exportEnabledSetting.argInfo.type = SoapySDR::ArgInfo::BOOL;

                ChannelSetting loFreqSetting
                {
                    {},
                    [rxLoName, this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_rx_lo_freq(rxLoName, chan));
                    },
                    [rxLoName, this](const size_t chan, const std::string &value) -> void
                    {
                        (void)_dev->set_rx_lo_freq(
                            SoapySDR::StringToSetting<double>(value),
                            rxLoName,
                            chan);
                    }
                };
                loFreqSetting.argInfo.key = loFreqSetting.argInfo.name = loFreqSettingName;
                loFreqSetting.argInfo.description = "LO frequency";
                loFreqSetting.argInfo.type = SoapySDR::ArgInfo::FLOAT;
                exportEnabledSetting.argInfo.range = metaRangeToRange(_dev->get_rx_lo_freq_range(rxLoName, chan));

                channelSettings.emplace(
                    std::move(sourceSettingName),
                    std::move(sourceSetting));
                channelSettings.emplace(
                    std::move(exportEnabledSettingName),
                    std::move(exportEnabledSetting));
                channelSettings.emplace(
                    std::move(loFreqSettingName),
                    std::move(loFreqSetting));
            }
        }
#endif
#ifdef UHD_HAS_POWER_LEVEL
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_TX); ++chan)
        {
            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            if(_dev->has_tx_power_reference(chan))
            {
                std::string key = "reference_power";

                ChannelSetting setting
                {
                    {},
                    [this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_tx_power_reference(chan));
                    },
                    [this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_tx_power_reference(
                            SoapySDR::StringToSetting<double>(value),
                            chan);
                    },
                    [this](const size_t chan) -> SoapySDR::Range
                    {
                        return metaRangeToRange(_dev->get_tx_power_range(chan));
                    },
                    true // queryRangeAtRuntime
                };
                setting.argInfo.key = key;
                setting.argInfo.value = "0.0";
                setting.argInfo.name = "Reference power";
                setting.argInfo.description = "The channel's reference TX power level. Note: this setting's range depends on various hardware settings.";
                setting.argInfo.units = "dBm";
                setting.argInfo.type = SoapySDR::ArgInfo::FLOAT;

                _txChannelSettings[chan].emplace(
                    std::move(key),
                    std::move(setting));
            }
        }
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_RX); ++chan)
        {
            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            if(_dev->has_rx_power_reference(chan))
            {
                std::string key = "reference_power";

                ChannelSetting setting
                {
                    {},
                    [this](const size_t chan) -> std::string
                    {
                        return SoapySDR::SettingToString(_dev->get_rx_power_reference(chan));
                    },
                    [this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_rx_power_reference(
                            SoapySDR::StringToSetting<double>(value),
                            chan);
                    },
                    [this](const size_t chan) -> SoapySDR::Range
                    {
                        return metaRangeToRange(_dev->get_rx_power_range(chan));
                    },
                    true // queryRangeAtRuntime
                };
                setting.argInfo.key = key;
                setting.argInfo.value = "0.0";
                setting.argInfo.name = "Reference power";
                setting.argInfo.description = "The channel's reference RX power level. Note: this setting's range depends on various hardware settings.";
                setting.argInfo.units = "dBm";
                setting.argInfo.type = SoapySDR::ArgInfo::FLOAT;

                _rxChannelSettings[chan].emplace(
                    std::move(key),
                    std::move(setting));
            }
        }
#endif
#ifdef UHD_HAS_GAIN_PROFILE
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_TX); ++chan)
        {
            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            auto txGainProfileNames = _dev->get_tx_gain_profile_names(chan);
            if(not txGainProfileNames.empty())
            {
                std::string key = "gain_profile";

                ChannelSetting setting
                {
                    {},
                    [this](const size_t chan) -> std::string
                    {
                        return _dev->get_tx_gain_profile(chan);
                    },
                    [this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_tx_gain_profile(value, chan);
                    },
                };
                setting.argInfo.key = key;
                setting.argInfo.name = "Gain profile";
                setting.argInfo.description = "The radio's gain profile for this channel.";
                setting.argInfo.type = SoapySDR::ArgInfo::STRING;
                setting.argInfo.options = std::move(txGainProfileNames);

                _txChannelSettings[chan].emplace(
                    std::move(key),
                    std::move(setting));
            }
        }
        for(size_t chan = 0; chan < this->getNumChannels(SOAPY_SDR_RX); ++chan)
        {
            // Generating a getter+setter that takes in a channel for each channel
            // isn't ideal, but we can't guarantee that each channel will have the
            // same settings. Since this is done at construction, it won't be a
            // bottleneck.
            auto rxGainProfileNames = _dev->get_rx_gain_profile_names(chan);
            if(not rxGainProfileNames.empty())
            {
                std::string key = "gain_profile";

                ChannelSetting setting
                {
                    {},
                    [this](const size_t chan) -> std::string
                    {
                        return _dev->get_rx_gain_profile(chan);
                    },
                    [this](const size_t chan, const std::string &value) -> void
                    {
                        _dev->set_rx_gain_profile(value, chan);
                    },
                };
                setting.argInfo.key = key;
                setting.argInfo.name = "Gain profile";
                setting.argInfo.description = "The radio's gain profile for this channel.";
                setting.argInfo.type = SoapySDR::ArgInfo::STRING;
                setting.argInfo.options = std::move(rxGainProfileNames);

                _rxChannelSettings[chan].emplace(
                    std::move(key),
                    std::move(setting));
            }
        }
#endif
    }

private:

    uhd::property_tree::sptr _get_tree(void) const
    {
        #if UHD_VERSION >= 4000000
        return _dev->get_tree();
        #else
        return _dev->get_device()->get_tree();
        #endif
    }

    uhd::usrp::multi_usrp::sptr _dev;
    const std::string _type;
    const bool _isNetworkDevice;

    // These are non-trivial to instantiate, so we'll do it on
    // construction rather than on-demand.

    struct Setting
    {
        SoapySDR::ArgInfo argInfo{};
        std::function<std::string(void)> getter{nullptr};
        std::function<void(const std::string&)> setter{nullptr};
        std::function<SoapySDR::Range(void)> rangeGetter{nullptr};
        bool queryRangeAtRuntime{false};
    };
    std::unordered_map<std::string, Setting> _settings;

    struct ChannelSetting
    {
        SoapySDR::ArgInfo argInfo{};
        std::function<std::string(const size_t)> getter{nullptr};
        std::function<void(const size_t, const std::string&)> setter{nullptr};
        std::function<SoapySDR::Range(const size_t)> rangeGetter{nullptr};
        bool queryRangeAtRuntime{false};
    };
    std::vector<std::unordered_map<std::string, ChannelSetting>> _txChannelSettings;
    std::vector<std::unordered_map<std::string, ChannelSetting>> _rxChannelSettings;
};

/***********************************************************************
 * Register into logger
 **********************************************************************/
#ifdef UHD_HAS_MSG_HPP
static void SoapyUHDLogger(uhd::msg::type_t t, const std::string &s)
{
    if (s.empty()) return;
    if (s[s.size()-1] == '\n') return SoapyUHDLogger(t, s.substr(0, s.size()-1));
    switch (t)
    {
    case uhd::msg::status: SoapySDR::log(SOAPY_SDR_INFO, s); break;
    case uhd::msg::warning: SoapySDR::log(SOAPY_SDR_WARNING, s); break;
    case uhd::msg::error: SoapySDR::log(SOAPY_SDR_ERROR, s); break;
    case uhd::msg::fastpath: SoapySDR::log(SOAPY_SDR_SSI, s); break;
    }
}
#else
static void SoapyUHDLogger(const uhd::log::logging_info &info)
{
    //build a log message formatted from the information
    std::string message;

    if (not info.file.empty())
    {
        std::string shortfile = info.file.substr(info.file.find_last_of("/\\") + 1);
        message += "[" + shortfile + ":" + std::to_string(info.line) + "] ";
    }

    if (not info.component.empty())
    {
        message += "[" + info.component + "] ";
    }

    message += info.message;

    switch(info.verbosity)
    {
    case uhd::log::trace:   SoapySDR::log(SOAPY_SDR_TRACE, message); break;
    case uhd::log::debug:   SoapySDR::log(SOAPY_SDR_DEBUG, message); break;
    case uhd::log::info:    SoapySDR::log(SOAPY_SDR_INFO, message); break;
    case uhd::log::warning: SoapySDR::log(SOAPY_SDR_WARNING, message); break;
    case uhd::log::error:   SoapySDR::log(SOAPY_SDR_ERROR, message); break;
    case uhd::log::fatal:   SoapySDR::log(SOAPY_SDR_FATAL, message); break;
    default: break;
    }
}
#endif

/***********************************************************************
 * Registration
 **********************************************************************/
std::vector<SoapySDR::Kwargs> find_uhd(const SoapySDR::Kwargs &args_)
{
    //prevent going into the the UHDSoapyDevice
    SoapySDR::Kwargs args(args_);
    if (args.count(SOAPY_UHD_NO_DEEPER) != 0) return std::vector<SoapySDR::Kwargs>();
    args[SOAPY_UHD_NO_DEEPER] = "";

    //perform the discovery
    #ifdef UHD_HAS_DEVICE_FILTER
    const uhd::device_addrs_t addrs = uhd::device::find(kwargsToDict(args), uhd::device::USRP);
    #else
    const uhd::device_addrs_t addrs = uhd::device::find(kwargsToDict(args));
    #endif

    //convert addrs to results
    std::vector<SoapySDR::Kwargs> results;
    for (size_t i = 0; i < addrs.size(); i++)
    {
        SoapySDR::Kwargs result(dictToKwargs(addrs[i]));

        //create displayable label if not present
        if (result.count("label") == 0)
        {
            if (result.count("product") != 0)
                result["label"] = result.at("product");

            if (result.count("serial") != 0)
                result["label"] += " " + result.at("serial");
        }

        result.erase(SOAPY_UHD_NO_DEEPER);
        results.push_back(result);
    }
    return results;
}

SoapySDR::Device *make_uhd(const SoapySDR::Kwargs &args)
{
    if(std::string(UHD_VERSION_ABI_STRING) != uhd::get_abi_string()) throw std::runtime_error(str(boost::format(
        "SoapySDR detected ABI compatibility mismatch with UHD library.\n"
        "SoapySDR UHD support was build against ABI: %s,\n"
        "but UHD library reports ABI: %s\n"
        "Suggestion: install an ABI compatible version of UHD,\n"
        "or rebuild SoapySDR UHD support against this ABI version.\n"
    ) % UHD_VERSION_ABI_STRING % uhd::get_abi_string()));
    #ifdef UHD_HAS_MSG_HPP
    uhd::msg::register_handler(&SoapyUHDLogger);
    #else
    uhd::log::add_logger("SoapyUHDDevice", &SoapyUHDLogger);
    #endif
    return new SoapyUHDDevice(uhd::usrp::multi_usrp::make(kwargsToDict(args)), args);
}

static SoapySDR::Registry register__uhd("uhd", &find_uhd, &make_uhd, SOAPY_SDR_ABI_VERSION);
