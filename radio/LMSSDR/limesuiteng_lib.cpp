// Work in progress

#include <limesuiteng/LimePlugin.h>
#include <limesuiteng/StreamConfig.h>
#include <limesuiteng/StreamMeta.h>

#include <math.h>

#include <libconfig.h>
#include "common/utils/LOG/log.h"
#include "common_lib.h"

#include <sys/resource.h>

#ifdef __SSE4_1__
  #include <smmintrin.h>
#endif

#ifdef __AVX2__
  #include <immintrin.h>
#endif

#ifdef __arm__
  #include <arm_neon.h>
#endif

using namespace std;
using namespace lime;

// OAI instance uses only one port/cell
static const int DEFAULT_PORT = 0;

static LimeRuntimeParameters params;

static void LogCallback(LogLevel lvl, const std::string& msg)
{
  switch(lvl)
  {
    case LogLevel::Info: LOG_I(HW, "%s\n", msg.c_str()); break;
    case LogLevel::Warning: LOG_W(HW, "%s\n", msg.c_str()); break;
    case LogLevel::Error: LOG_E(HW, "%s\n", msg.c_str()); break;
    case LogLevel::Verbose: LOG_I(HW, "%s\n", msg.c_str()); break;
    //case LogLevel::Debug: LOG_D(HW, "%s\n", msg.c_str()); break;
    case LogLevel::Debug: LOG_I(HW, "%s\n", msg.c_str()); break;
    default:
      LOG_I(HW, "%s\n", msg.c_str()); break;
  }
}

class OAIParamProvider : public LimeSettingsProvider
{
public:
    OAIParamProvider()
    {
      config_init(&state);
    }
    virtual ~OAIParamProvider()
    {
      config_destroy(&state);
    };

    // read the .cfg file
    int Init(const char* filename)
    {
      if (filename && config_read_file(&state, filename) == CONFIG_FALSE)
      {
        LOG_E(HW, "%s parse failed(%i): %s\n", filename, config_error_line(&state), config_error_text(&state));
        return -1;
      }
      return 0;
    }

    virtual bool GetString(std::string& dest, const char* varname) override
    {
      const char* ctemp = nullptr;
      if (config_lookup_string(&state, varname, &ctemp) == CONFIG_FALSE)
        return false;

      dest = std::string(ctemp);
      return true;
    }

    virtual bool GetDouble(double& dest, const char* varname) override
    {
      if (config_lookup_float(&state, varname, &dest) == CONFIG_TRUE)
        return true;

      // attempt to look up integer, if float was not available
      int ivalue = 0;
      if (config_lookup_int(&state, varname, &ivalue) == CONFIG_FALSE)
        return false;

      dest = ivalue;
      return true;
    }
private:
    config_t state;
};

int trx_lms7002m_set_gains(openair0_device *device, openair0_config_t *openair0_cfg)
{
  LOG_D(HW, "trx_lms7002m_set_gain, Tx:%f Rx:%f:\n", openair0_cfg[0].tx_gain[0], openair0_cfg[0].rx_gain[0]);
  LimePluginContext *context = (LimePluginContext*)device->priv;

  OpStatus status = context->rxChannels[0].parent->device->SetGain(0, TRXDir::Rx, 0, lime::eGainTypes::GENERIC, openair0_cfg[0].rx_gain[0]);
  if (status != OpStatus::Success)
  {
    LOG_E(HW, "Failed to set Rx gain %f", openair0_cfg[0].rx_gain[0]);
    return -1;
  }

  status = context->txChannels[0].parent->device->SetGain(0, TRXDir::Tx, 0, lime::eGainTypes::GENERIC, openair0_cfg[0].tx_gain[0]);
  if (status != OpStatus::Success)
  {
    LOG_E(HW, "Failed to set Tx gain %f", openair0_cfg[0].tx_gain[0]);
    return -1;
  }

  return(0);
}

static int trx_lms7002m_start(openair0_device *device)
{
  LimePluginContext* context = static_cast<LimePluginContext*>(device->priv);
  LimePlugin_Start(context);
  return 0;
}

static int trx_lms7002m_stop(openair0_device *device) {
  LimePluginContext *context = static_cast<LimePluginContext*>(device->priv);
  return LimePlugin_Stop(context);
}

static void trx_lms7002m_end(openair0_device *device) {
  LimePluginContext *context = static_cast<LimePluginContext*>(device->priv);
  LimePlugin_Destroy(context);
  delete context;
}

static int trx_lms7002m_write(openair0_device *device, openair0_timestamp timestamp,
                          void **buff, int nsamps, int channelCount, int flags) 
{
  if (!buff) // Nothing to transmit
    return 0;

  StreamTxMeta meta;
  timestamp -= device->openair0_cfg->command_line_sample_advance + device->openair0_cfg->tx_sample_advance;
  meta.timestamp = lime::Timespec(int64_t(timestamp));
  meta.hasTimestamp = true;
  meta.flags = ((flags == TX_BURST_END) || (flags == TX_BURST_START_AND_END)) ? StreamTxMeta::Flags::EndOfBurst : 0;

  // samples format conversion is done internally
  LimePluginContext* context = static_cast<LimePluginContext*>(device->priv);

  // OAI stores samples as 16bit I + 16bit Q, but actually uses only 12bit LSB
  lime::complex12_t** samples = reinterpret_cast<lime::complex12_t**>(buff);
  return LimePlugin_Write_complex12(context, samples, nsamps, DEFAULT_PORT, meta);
}

static int trx_lms7002m_read(openair0_device *device, openair0_timestamp *ptimestamp,
                          void **buff, int nsamps, int channelCount)
{
  LimePluginContext *context = (LimePluginContext*)device->priv;

  // OAI stores samples as 16bit I + 16bit Q, but actually uses only 12bit LSB
  lime::complex12_t** samples = reinterpret_cast<lime::complex12_t**>(buff);

  StreamRxMeta meta;

  int samplesGot = LimePlugin_Read_complex12(context, samples, nsamps, DEFAULT_PORT, meta);
  if (samplesGot <= 0)
    return samplesGot;

  *ptimestamp = meta.timestamp.GetTicks();
  return samplesGot;
}

static int trx_lms7002m_get_stats(openair0_device *device) {
  return(0);
}

static int trx_lms7002m_reset_stats(openair0_device *device) {
  return(0);
}

static int trx_lms7002m_set_freq(openair0_device *device, openair0_config_t *openair0_cfg) {
  LOG_D(HW, "trx_lms7002m_set_freq TX Freq %f, RX Freq %f, tune_offset: %f\n", openair0_cfg[0].tx_freq[0], openair0_cfg[0].rx_freq[0], openair0_cfg[0].tune_offset);

  LimePluginContext *context = (LimePluginContext*)device->priv;

  OpStatus status = context->rxChannels[0].parent->device->SetFrequency(0, TRXDir::Rx, 0, openair0_cfg[0].rx_freq[0]);
  if (status != OpStatus::Success)
  {
    LOG_E(HW, "Failed to set Rx Freq %f", openair0_cfg[0].rx_freq[0]);
    return -1;
  }

  status = context->txChannels[0].parent->device->SetFrequency(0, TRXDir::Tx, 0, openair0_cfg[0].tx_freq[0]);
  if (status != OpStatus::Success)
  {
    LOG_E(HW, "Failed to set Tx Freq %f", openair0_cfg[0].tx_freq[0]);
    return -1;
  }

  return(0);
}

static int trx_lms7002m_write_init(openair0_device *device) {
  return(0);
}

static OAIParamProvider configProvider;

extern "C" {
/*! \brief Initialize Openair limesuite target. It returns 0 if OK
 * \param device the hardware to use
 * \param openair0_cfg RF frontend parameters set by application
 * \returns 0 on success
 */
int device_init(openair0_device *device,
          openair0_config_t *openair0_cfg)
{
  double val = 0;
  std::string cwd; // current working dir

  std::string configFilePath;
  if (!openair0_cfg[0].configFilename)
  {
    LOG_E(HW, "--rf-config-file not provided\n");
    return -1;
  }
  else
    configFilePath = openair0_cfg[0].configFilename;

  LOG_I(HW, "rf-config-file: %s\n", configFilePath.c_str());
  if (configProvider.Init(configFilePath.c_str()) != 0)
    return -1;

  size_t cwdLength = configFilePath.find_last_of("/");
  if (cwdLength != std::string::npos)
    cwd = configFilePath.substr(0, cwdLength);
  LOG_D(HW, "cwd: %s\n", cwd.c_str());

  LimePluginContext* context = new LimePluginContext();
  context->currentWorkingDirectory = cwd;
  context->samplesFormat = DataFormat::I12;

  int status = LimePlugin_Init(context, LogCallback, &configProvider);
  if (status != 0)
    return status;

  int rxCount = openair0_cfg->rx_num_channels;
  int txCount = openair0_cfg->tx_num_channels;

  params.rf_ports.clear();
  LimeRuntimeParameters::PortParams port;
  port.sample_rate = openair0_cfg->sample_rate;
  port.rx_channel_count = rxCount;
  port.tx_channel_count = txCount;
  params.rf_ports.push_back(port);

  AssignCArrayToVector(params.rx.freq, openair0_cfg->rx_freq, rxCount);
  if (configProvider.GetDouble(val, "rx_gain"))
    params.rx.gain.assign(rxCount, val);
  else
    AssignCArrayToVector(params.rx.gain, openair0_cfg->rx_gain, rxCount);
  params.rx.bandwidth.assign(rxCount, openair0_cfg->rx_bw);
  for (int c=0; c<rxCount; ++c)
  {
    if (params.rx.bandwidth[c] == 0)
      params.rx.bandwidth[c] = openair0_cfg->sample_rate;
  }

  AssignCArrayToVector(params.tx.freq, openair0_cfg->tx_freq, txCount);
  if (configProvider.GetDouble(val, "tx_gain"))
    params.tx.gain.assign(txCount, val);
  else
    AssignCArrayToVector(params.tx.gain, openair0_cfg->tx_gain, txCount);
  params.tx.bandwidth.assign(txCount, openair0_cfg->tx_bw);
  for (int c=0; c<txCount; ++c)
  {
    if (params.tx.bandwidth[c] == 0)
      params.tx.bandwidth[c] = openair0_cfg->sample_rate;
  }

  /* Set callbacks */
  device->priv = (void*)context;
  device->openair0_cfg          = openair0_cfg;
  device->type                  = LMSSDR_DEV;
  device->trx_start_func        = trx_lms7002m_start;
  device->trx_end_func          = trx_lms7002m_end;
  device->trx_stop_func         = trx_lms7002m_stop;
  device->trx_read_func         = trx_lms7002m_read;
  device->trx_write_func        = trx_lms7002m_write;
  device->trx_get_stats_func    = trx_lms7002m_get_stats;
  device->trx_reset_stats_func  = trx_lms7002m_reset_stats;
  device->trx_set_freq_func     = trx_lms7002m_set_freq;
  device->trx_set_gains_func    = trx_lms7002m_set_gains;
  device->trx_write_init        = trx_lms7002m_write_init;

  if (configProvider.GetDouble(val, "tx_sample_advance"))
    openair0_cfg->tx_sample_advance = val;

  return LimePlugin_Setup(context, &params);
};

} // extern "C"
