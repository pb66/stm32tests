//
// Change these all to 1.0 if you want raw A/D units
//

const float VCAL = 0.24124403350;

const float ICAL[MAX_CHANNELS] = {
  0.04877123100,
  0.04888602091,
  0.04883471491,
  0.01989940309
};

const int ADC_LAG = 269;             // ~4.8 degrees
