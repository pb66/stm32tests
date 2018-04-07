#define MAX_ADC_READING 4095
#define MID_ADC_READING 2047
#define MAX_CHANNELS 4
#define SAMPLES_PER_BATCH  147059        // 147059 * 68usecs = ~10 seconds worth
#define DUMP_MAX 600
#define DUMP_CHANS 2

typedef struct power_stats_ {
  float sigma_power;
  float sigma_i_sq;
  float sigma_v_sq;
  int count;
  int sigma_i;
  int sigma_v;
  bool clipped;
  bool data_ready;
} power_stats_t;

extern void process_VI_pair(uint16_t voltage, uint16_t current, int channel);
extern void process_power_data(void);
extern void init_power(void);

