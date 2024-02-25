// Work in progress

#include <limesuite/SDRDevice.h>
#include <limesuite/DeviceRegistry.h>
#include "common.h"

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

static void LogCallback(SDRDevice::LogLevel lvl, const char* msg)
{
  switch(lvl)
  {
    case SDRDevice::LogLevel::INFO: LOG_I(HW, "%s\n", msg); break;
    case SDRDevice::LogLevel::WARNING: LOG_W(HW, "%s\n", msg); break;
    case SDRDevice::LogLevel::ERROR: LOG_E(HW, "%s\n", msg); break;
    case SDRDevice::LogLevel::VERBOSE: LOG_I(HW, "%s\n", msg); break;
    //case SDRDevice::LogLevel::DEBUG: LOG_D(HW, "%s\n", msg); break;
    case SDRDevice::LogLevel::DEBUG: LOG_I(HW, "%s\n", msg); break;
    default:
      LOG_I(HW, "%s\n", msg); break;
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
  // TODO: implement
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

  int nsamps2;  // aligned to upper 32 or 16 byte boundary
#if defined(__x86_64) || defined(__i386__)
    nsamps2 = (nsamps+7)>>3;
    __m256i buff_tx[2][65536];
#elif defined(__arm__) || defined(__aarch64__)
    nsamps2 = (nsamps+3)>>2;
    int16x8_t buff_tx[2][65536];
#else
#error Unsupported CPU architecture
#endif

  // TODO: implement data shift in limesuite for efficiency
  // (copied from USRP)
  // bring TX data from 12 LSBs softmodem to 16bits
  for (int i=0; i<channelCount; i++) {
    for (int j=0; j<nsamps2; j++) {
#if defined(__x86_64__) || defined(__i386__)
      if ((((uintptr_t) buff[i])&0x1F)==0) {
        buff_tx[i][j] = simde_mm256_slli_epi16(((__m256i *)buff[i])[j],4);
      }
      else
      {
        __m256i tmp = simde_mm256_loadu_si256(((__m256i *)buff[i])+j);
        buff_tx[i][j] = simde_mm256_slli_epi16(tmp,4);
      }
#elif defined(__arm__) || defined(__aarch64__)
      buff_tx[i][j] = vshlq_n_s16(((int16x8_t *)buff[i])[j],4);
#endif
    }
  }

  SDRDevice::StreamMeta meta;
  meta.timestamp = timestamp;
  meta.useTimestamp = true;
  meta.flush = (flags == TX_BURST_END) || (flags == TX_BURST_START_AND_END);

  // samples format conversion is done internally
  LimePluginContext* context = static_cast<LimePluginContext*>(device->priv);

  // OAI stores samples as 16bit I + 16bit Q, but actually uses only 12bit LSB
  //lime::complex16_t** samples = reinterpret_cast<lime::complex16_t**>(buff);
  const lime::complex16_t* samples[2] = {(lime::complex16_t*)buff_tx[0], (lime::complex16_t*)buff_tx[1]};
  return LimePlugin_Write_complex16(context, samples, nsamps, DEFAULT_PORT, meta);
}

static int trx_lms7002m_read(openair0_device *device, openair0_timestamp *ptimestamp,
                          void **buff, int nsamps, int channelCount)
{
  int nsamps2;  // aligned to upper 32 or 16 byte boundary
#if defined(__x86_64) || defined(__i386__)
  nsamps2 = (nsamps+7)>>3;
  __m256i buff_tmp[2][nsamps2];
#elif defined(__arm__) || defined(__aarch64__)
  nsamps2 = (nsamps+3)>>2;
  int16x8_t buff_tmp[2][nsamps2];
#endif

  LimePluginContext *context = (LimePluginContext*)device->priv;

  // OAI stores samples as 16bit I + 16bit Q, but actually uses only 12bit LSB
  lime::complex16_t* samples[2] = {(lime::complex16_t*)buff_tmp[0], (lime::complex16_t*)buff_tmp[1]};

  SDRDevice::StreamMeta meta;
  meta.useTimestamp = false;
  meta.flush = false;

  int samplesGot = LimePlugin_Read_complex16(context, samples, nsamps, DEFAULT_PORT, meta);
  if (samplesGot <= 0)
    return samplesGot;

  *ptimestamp = meta.timestamp;
  // TODO: implement Rx data shift in limesuite for efficiency

  // (copied from USRP)
  // bring RX data into 12 LSBs for softmodem RX
  const int rxshift=4;
  for (int i=0; i<channelCount; i++) {
    for (int j=0; j<nsamps2; j++) {
#if defined(__x86_64__) || defined(__i386__)
      // FK: in some cases the buffer might not be 32 byte aligned, so we cannot use avx2

      if ((((uintptr_t) buff[i])&0x1F)==0) {
        ((__m256i *)buff[i])[j] = simde_mm256_srai_epi16(buff_tmp[i][j],rxshift);
      } else {
        __m256i tmp = simde_mm256_srai_epi16(buff_tmp[i][j],rxshift);
        simde_mm256_storeu_si256(((__m256i *)buff[i])+j, tmp);
      }
    }
#elif defined(__arm__) || defined(__aarch64__)
      for (int j=0; j<nsamps2; j++)
        ((int16x8_t *)buff[i])[j] = vshrq_n_s16(buff_tmp[i][j],rxshift);
#endif
  }
  return samplesGot;
}

static int trx_lms7002m_get_stats(openair0_device *device) {
  return(0);
}

static int trx_lms7002m_reset_stats(openair0_device *device) {
  return(0);
}

static int trx_lms7002m_set_freq(openair0_device *device, openair0_config_t *openair0_cfg) {
  // TODO: implement
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

  LOG_D(HW, "configFilename: %s\n", openair0_cfg[0].configFilename);
  if (configProvider.Init(openair0_cfg[0].configFilename) != 0)
    return -1;

  std::string configFilePath(openair0_cfg[0].configFilename);
  size_t cwdLength = configFilePath.find_last_of("/");
  if (cwdLength != std::string::npos)
    cwd = configFilePath.substr(0, cwdLength);
  LOG_D(HW, "cwd: %s\n", cwd.c_str());

  LimePluginContext* context = new LimePluginContext();
  context->currentWorkingDirectory = cwd;
  context->samplesFormat = SDRDevice::StreamConfig::DataFormat::I16;

  int status = LimePlugin_Init(context, LogCallback, &configProvider);
  if (status != 0)
    return status;

  int rxCount = openair0_cfg->rx_num_channels;
  int txCount = openair0_cfg->tx_num_channels;

  params.rf_ports = {{openair0_cfg->sample_rate, rxCount, txCount}};

  AssignCArrayToVector(params.rx.freq, openair0_cfg->rx_freq, rxCount);
  if (configProvider.GetDouble(val, "rx_gain"))
    params.rx.gain.assign(rxCount, val);
  else
    AssignCArrayToVector(params.rx.gain, openair0_cfg->rx_gain, rxCount);
  params.rx.bandwidth.assign(rxCount, openair0_cfg->rx_bw);

  AssignCArrayToVector(params.tx.freq, openair0_cfg->tx_freq, txCount);
  if (configProvider.GetDouble(val, "tx_gain"))
    params.tx.gain.assign(txCount, val);
  else
    AssignCArrayToVector(params.tx.gain, openair0_cfg->tx_gain, txCount);
  params.tx.bandwidth.assign(txCount, openair0_cfg->tx_bw);

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
