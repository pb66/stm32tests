//
// Change these all to 1.0 if you want raw A/D units
//

const float VCAL = 0.24119479813;

const float ICAL[MAX_CHANNELS] = {
  0.04874034071,
  0.04897653045,
  0.04883745917,
  0.01989279489
};

const int ADC_LAG = 269;             // ~4.8 degrees

