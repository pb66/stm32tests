//
// Change these all to 1.0 if you want raw A/D units
//

const float VCAL = 0.24142421201;

const float ICAL[MAX_CHANNELS] = {
  0.04943131519,
  0.04894363325,
  0.04890794487,
  0.01963339104
};


const int ADC_LAG = 269;             // ~4.8 degrees
