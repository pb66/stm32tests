
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "usart.h"
#include "adc.h"
#include "power.h"
#include "calib.h"

#define ENERGY_MONITORING 1

//
// Volatile power stats are where the stats get accumulated
// on each interrupt, i.e. each call to process_VI_pair()
//
static volatile power_stats_t vol_power_stats[MAX_CHANNELS];

//
// Interval power stats are were the stats get moved to when
// process_VI_pair() has decided it's accumulated enough
// (147,059 currently... about 10 seconds worth).
// This data structure is still volatile, just not as volatile.
// process_power_stats() works on these intervals stats once they're
// ready.
//
static volatile power_stats_t intvl_power_stats[MAX_CHANNELS];


#ifdef DUMPING
static volatile uint16_t dump[DUMP_CHANS][DUMP_MAX];
static volatile int dump_index[DUMP_CHANS];
#endif

//
// Called from the ADC/DMA interrupt when a new V,I pair has arrived.
// Basically does the accumulation maths on the new pair.  It starts
// accumulating after the first zero crossing on V, accumulates for
// ~10 seconds (rounded to another zero crossing on V) and continues
// forever.  All zero crossing detection is done on the raw readings
// minus the nominal mid-rail, so won't be perfect, but should be better
// than jumping in/out at the peak.
//
void process_VI_pair (uint16_t voltage, uint16_t current, int channel) {
  int signed_volt, signed_curr;
  volatile power_stats_t* stats_p;

  stats_p = &vol_power_stats[channel];

#ifdef DUMPING
  if ((channel == 0) && dump_index[0] < DUMP_MAX)
    dump[0][dump_index[0]++] = voltage;
  else if ((channel == 3) && dump_index[1] < DUMP_MAX)
    dump[1][dump_index[1]++] = voltage;
#endif
  
  if (current == MAX_ADC_READING)                 // Make a note if we've clipped
    stats_p->clipped = true;

  signed_volt = voltage - MID_ADC_READING;        // Remove the nominal mid-rail
  signed_curr = current - MID_ADC_READING;

  //
  // If it's the very first time through, we don't have a useful
  // last_v to check for a zero crossing, so use this first one
  // to prime last_v but otherwise ignore it.
  //
  if (stats_p->state == INIT) {
    stats_p->last_v = signed_volt;
    stats_p->state = HUNTING_ZX_HEAD;
    return;
  }

  //
  // Are we waiting for the leading zero cross?
  //
  if (stats_p->state == HUNTING_ZX_HEAD) {
    if (stats_p->last_v * signed_volt < 0) {         // Found a zero crossing
      stats_p->state = ACCUMULATE;                   // promote the state and use this sample
      stats_p->last_v = signed_volt;                 // not really needed, but keep it consistent
    } else {
      stats_p->last_v = signed_volt;                 // Make a note for next time
      return;                                        // but otherwise ignore this sample
    }
  }
    
  //
  // Are we waiting for the trailing zero cross?
  //
  if (stats_p->state == HUNTING_ZX_TAIL) {
    if (stats_p->last_v * signed_volt < 0) {         // Found a zero crossing
      volatile power_stats_t* intvl_stats_p;
      //
      // We've got a batch worth, and we've just seen a zero crossing
      // The first sample after the zero crossing goes into the next
      // batch, so before including this one we need to flush out the accumulation.
      // This sample will then become the fist sample in the new batch.
      // Copy the accumulated stats into interval stats for process level to deal with
      // and zero out our stats ready for the next batch (including this new sample).
      //
      intvl_stats_p = &intvl_power_stats[channel];
      memcpy ((void*)intvl_stats_p, (void*)stats_p, sizeof(power_stats_t));
      intvl_stats_p->data_ready = true;
      memset((void*)stats_p, 0, sizeof(power_stats_t));
      stats_p->last_v = signed_volt;                 // not really needed, but keep it consistent
      stats_p->state = ACCUMULATE;                   // promote the state and...
                                                     // use this sample by not returning
    } else {
      stats_p->last_v = signed_volt;                 // Make a note for next time and....
                                                     // use this sample by not returning
    }
  }

  //
  // See if we've got a batch full.  Once we have,
  // we'll keep going until the next zero crossing.
  //
  if (stats_p->count >= SAMPLES_PER_BATCH) {             // Got almost 10 seconds worth?
    stats_p->last_v = signed_volt;                       // Make a note for next time and....
    stats_p->state = HUNTING_ZX_TAIL;                    // Start looking for a trailing ZX and...
                                                         // use this sample by not returning
  }

  //
  // If we get to here, the state machine has decided this sample pair should be included
  // in the stats.  It bails early when it's decided we want to ignore this sample.
  // In the interest of continuous sampling, that only ever happens when we're looking
  // for the very first zero crossing.  Almost all samples make it through to here.
  //
  stats_p->sigma_power += (float)(signed_volt * signed_curr);
  stats_p->count++;
  stats_p->sigma_i += signed_curr;
  stats_p->sigma_i_sq += (float)(signed_curr * signed_curr);
  stats_p->sigma_v += signed_volt;
  stats_p->sigma_v_sq += (float)(signed_volt * signed_volt);
}

//
// Called often from the infinite loop in main().  Check to see if there are new interval
// stats we haven't processed yet, and if so, process them and flag them as processed.
//
void process_power_data () {


#ifdef ENERGY_MONITORING
  //
  // If any of them are not ready, come back later.  This ensures they'll always
  // come out in 0..3 order.
  //
  for (int chan=0; chan<MAX_CHANNELS; chan++)
    if (!intvl_power_stats[chan].data_ready)
      return;
    
  for (int chan=0; chan<MAX_CHANNELS; chan++) {
    if (intvl_power_stats[chan].data_ready) {
      power_stats_t local_stats;
      int Vmean, Imean;
      int count;
      float Vrms, Irms, Preal, Papp, PF;

      //
      // Copy them to a local stack copy that is completely non-volatile.  Not essential
      // as these aren't going to get overwritten for ages, but removing the volatility
      // means the compiler can make optimisations it might not otherwise.
      //
      memcpy(&local_stats, (void*)&intvl_power_stats[chan], sizeof(power_stats_t));
      intvl_power_stats[chan].data_ready = false;                 // flag it as done
      count = local_stats.count;

      //
      // The nominal mid-rail voltage was removed above in process_VI_pair(), here
      // we calculate what's left.
      //
      Vmean = (local_stats.sigma_v + count/2)/count;
      Imean = (local_stats.sigma_i + count/2)/count;

      //
      // And remove its RMS from the accumulated RMS
      //
      local_stats.sigma_v_sq /= count;
      local_stats.sigma_i_sq /= count;
      local_stats.sigma_v_sq -= (float)(Vmean * Vmean);
      local_stats.sigma_i_sq -= (float)(Imean * Imean);

      //
      // Calculate the RMS values and apparent power.
      //
      Vrms = sqrt(local_stats.sigma_v_sq);
      Irms = sqrt(local_stats.sigma_i_sq);
      Papp = Vrms * Irms;

      //
      // Remove the offset power from the accumulated real power and
      // calculate the power factor.
      //
      Preal = local_stats.sigma_power / (float)count - (float)(Vmean * Imean);
      PF = Preal / Papp;
      
      //
      // Dump it out on the console.  If your %f's come out as blanks you need
      // to add "-u _printf_float" to your link command.  See project_name.mak
      //
      snprintf(log_buffer, sizeof(log_buffer),
	       "%d%c Vrms: %6.2f, Irms: %5.2f, Papp: %7.2f, Preal: %7.2f, PF: %.3f, Count:%d\n",
	       chan, local_stats.clipped?'>':':', Vrms*VCAL, Irms*ICAL[chan],
	       Papp*VCAL*ICAL[chan], Preal*VCAL*ICAL[chan], PF, count);
      debug_printf(log_buffer);
    }
  }
#endif

#ifdef DUMPING
  if ((dump_index[0] == DUMP_MAX) && (dump_index[1] == DUMP_MAX)) {
    for (int i=0; i<DUMP_MAX; i++) {
      snprintf(log_buffer, sizeof(log_buffer),
	       "%d, %d, %d\n", i, dump[0][i], dump[1][i]);
      debug_printf(log_buffer);
    }
    dump_index[0] = dump_index[1] = DUMP_MAX+1;
  }
#endif
  
}

void init_power (void) {

  start_ADCs(ADC_LAG);                 // start ADC with x usec lag
}
