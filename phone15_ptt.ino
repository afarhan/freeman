#include <LoRa.h>
#include <AESLib.h>

char is_transmitting = 0;
char is_debug = 0;
#define PTT_PIN (3)

//this transmits an 8-bit value via the lora at 8

// adc_to_pwm.pde
// ADC to PWM converter
// guest - openmusiclabs.com - 1.9.13
// options table at http://wiki.openmusiclabs.com/wiki/PWMDAC
// takes in audio data from the ADC and plays it out on
// Timer1 PWM.  16b, Phase Correct, 31.25kHz - although ADC is 10b.


// set to 15.6 khz sampling of 10 bits, fast
#define PWM_FREQ 0x03FF // pwm frequency - see table
#define PWM_MODE 1 // Fast (1) or Phase Correct (0)
#define PWM_QTY 1 // number of pwms, either 1 or 2

/*
 * LORA RA-02 module PINS
 * GND  - GND
 * NSS  - D8
 * MOSI - D11 
 * MISO - D12
 * SCK - D13
 * RST  - D7
 */

#define LORA_NSS (8)
#define LORA_NRESET (7)


/*
 * ADPCM
 */

/* Intel ADPCM step variation table */

int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

int32_t stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

int32_t coder_state_valprev = 0;
int32_t coder_state_index = 0;
int32_t decoder_state_valprev = 0;
int32_t decoder_state_index = 0;

void adpcm_init(){
  coder_state_valprev = 0;
  coder_state_index = 0;
}

void adpcm_coder(int16_t *indata, char *outdata, int len){
    int *inp;      /* Input buffer pointer */
    signed char *outp;    /* output buffer pointer */
    int32_t val;      /* Current input sample value */
    int sign;     /* Current adpcm sign bit */
    int32_t delta;      /* Current adpcm output value */
    int diff;     /* Difference between val and valprev */
    int32_t step;     /* Stepsize */
    int32_t valpred;    /* Predicted output value */
    int32_t vpdiff;     /* Current change to valpred */
    int index;      /* Current step change index */
    int outputbuffer;   /* place to keep previous 4-bit value */
    int bufferstep;   /* toggle between outputbuffer/output */
    int32_t bias = 0;

    outp = (signed char *)outdata;
    inp = indata;

    valpred = coder_state_valprev;
    index = coder_state_index;
    step = stepsizeTable[index];
    
    bufferstep = 1;

    for ( ; len > 0 ; len-- ) {
      val = *inp++;

      /* Step 1 - compute difference with previous value */
      diff = val - valpred;
      sign = (diff < 0) ? 8 : 0;
      if ( sign ) diff = (-diff);

      /* Step 2 - Divide and clamp */
      /* Note:
      ** This code *approximately* computes:
      **    delta = diff*4/step;
      **    vpdiff = (delta+0.5)*step/4;
      ** but in shift step bits are dropped. The net result of this is
      ** that even if you have fast mul/div hardware you cannot put it to
      ** good use since the fixup would be too expensive.
      */
      delta = 0;
      vpdiff = (step >> 3);
      
      if ( diff >= step ) {
          delta = 4;
          diff -= step;
          vpdiff += step;
      }
      step >>= 1;
      if ( diff >= step  ) {
          delta |= 2;
          diff -= step;
          vpdiff += step;
      }
      step >>= 1;
      if ( diff >= step ) {
          delta |= 1;
          vpdiff += step;
      }

    /* Step 3 - Update previous value */
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;
  
    /* Step 4 - Clamp previous value to 16 bits */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;

    /* Step 5 - Assemble value, update index and step values */
    delta |= sign;
    
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;
    step = stepsizeTable[index];
  
    /* Step 6 - Output value */
    if ( bufferstep ) {
        outputbuffer = (delta << 4) & 0xf0;
    } else {
        *outp++ = (delta & 0x0f) | outputbuffer;
    }
    bufferstep = !bufferstep;

     // added by farhan, to correct the bias
     bias += valpred;
  }

  /* Output last step, if needed */
  if ( !bufferstep )
    *outp++ = outputbuffer;

  // farhan: calculate the average bias and remove it, if it gets too big
  bias  = bias/len;
  if (bias > 10 || bias < -10)
    valpred -= bias;


  coder_state_valprev = valpred;
  coder_state_index = index;
}

void adpcm_decoder(char *indata, int *outdata, int len){
  signed char *inp;    /* Input buffer pointer */
  short *outp;    /* output buffer pointer */
  int sign;     /* Current adpcm sign bit */
  int32_t delta;      /* Current adpcm output value */
  int32_t step;     /* Stepsize */
  int32_t valpred;    /* Predicted value */
  int32_t vpdiff;     /* Current change to valpred */
  int index;      /* Current step change index */
  int inputbuffer;    /* place to keep next 4-bit value */
  int bufferstep;   /* toggle between inputbuffer/input */
  int32_t bias=0;
  int nsamples;

  outp = outdata;
  inp = (signed char *)indata;

  valpred = decoder_state_valprev;
  index = decoder_state_index;
  step = stepsizeTable[index];
  nsamples = len;

  bufferstep = 0;
  
  for ( ; len > 0 ; len-- ) {

    /* Step 1 - get the delta value */
    if ( bufferstep ) {
        delta = inputbuffer & 0xf;
    } else {
        inputbuffer = *inp++;
        delta = (inputbuffer >> 4) & 0xf;
    }
    bufferstep = !bufferstep;
  
    /* Step 2 - Find new index value (for later) */
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;
  
    /* Step 3 - Separate sign and magnitude */
    sign = delta & 8;
    delta = delta & 7;
  
    /* Step 4 - Compute difference and new predicted value */
    /*
    ** Computes 'vpdiff = (delta+0.5)*step/4', but see comment
    ** in adpcm_coder.
    */
    vpdiff = step >> 3;
    if ( delta & 4 ) vpdiff += step;
    if ( delta & 2 ) vpdiff += step>>1;
    if ( delta & 1 ) vpdiff += step>>2;
  
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;
  
    /* Step 5 - clamp output value */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;
  
    /* Step 6 - Update step value */
    step = stepsizeTable[index];

    /* Step 7 - Output value */
    *outp++ = (int) valpred;

    bias += valpred;
  }

  // farhan: calculate the average bias and remove it, if it gets too big
  bias  = bias/nsamples;
  valpred -= bias;
    
  decoder_state_valprev = valpred;
  decoder_state_index = index;
}

/**
 * Audio sampling queues for playback and recording
 */

#define MAX_Q (200)
struct Queue
{
  int id;
  volatile uint8_t head;
  volatile uint8_t tail;
//  volatile uint8_t count;
  int  stall;
  int data[MAX_Q+1];
};
struct Queue qout;

void q_init(struct Queue *p){
  p->head = 0;
  p->tail = 0;
  p->stall = 1;
}



unsigned long overflow_count = 0;

void checkQueue(int line, struct Queue *q){
  if (q->head <= MAX_Q && q->tail <= MAX_Q )
    return;

  Serial.print(line);Serial.print('=');
  if (q->head >= MAX_Q)
    Serial.print('H');
  if (q->tail >= MAX_Q)
    Serial.print('T');
  Serial.print('h');
  Serial.print(q->head); Serial.print('t');
  Serial.print(q->tail); Serial.print('s');
  Serial.println(q->stall);
 // delay(10000);
}

int q_length(struct Queue *p){
  if ( p->head >= p->tail)
    return p->head - p->tail;
  else
    return ((p->head + MAX_Q) - p->tail);
}

void q_write(struct Queue *p, int w){

  if (p->head + 1 == p->tail || p->tail == 0 && p->head == MAX_Q-1){
    //Serial.print("overflow at head=");Serial.println(p->head);
    overflow_count++;
    return;
  }

//  checkQueue(__LINE__, p);
  
  p->data[p->head++] = w;
  if (p->head > MAX_Q){
    //Serial.print("Resetting head to 0 at ");Serial.println(p->head);
    p->head = 0;
  }
    
//  checkQueue(__LINE__, p);  
}

unsigned long underflow = 0;
int q_read(struct Queue *p){
 int16_t data;
    
//  checkQueue(__LINE__, p);

  if (p->tail == p->head){
    underflow++;
    //Serial.print(p->id);Serial.print(":underflow tail = ");Serial.println(p->tail);
    return (int)0;
  }
    
  data = p->data[p->tail++];
  if (p->tail > MAX_Q)
    p->tail = 0;

//  checkQueue(__LINE__, p);  
  return data;
}

/**
 * ADC, DAC to record and playback the audio
 */

void initAudio(){
  // setup ADC
  ADMUX = 0x60; // left adjust, adc0, internal vcc
  ADCSRA = 0xe5; // turn on adc, ck/32, auto trigger
  ADCSRB =0x07; // t1 capture for trigger
  DIDR0 = 0x01; // turn off digital inputs for adc0
  
  // setup PWM
  TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); // 
  TCCR1B = ((PWM_MODE << 3) | 0x11); // ck/1
  TIMSK1 = 0x20; // interrupt on capture interrupt
  ICR1H = (PWM_FREQ >> 8);
  ICR1L = (PWM_FREQ & 0xff);
  DDRB |= ((PWM_QTY << 1) | 0x02); // turn on outputs
  
  sei(); // turn on interrupts - not really necessary with arduino  
}

unsigned int prev = 0;
/*
 * The Audio is sampled at 15.6 khz we use the prev variable to sample down
 */
int count =0, adcCount;
int previouslyReadSample = 0;

ISR(TIMER1_CAPT_vect) {
   
  // get ADC data
  uint16_t temp1 = ADCL; // you need to fetch the low byte first
  uint16_t temp2 = ADCH;
  uint16_t s;
  int16_t rx_sample;
  
  if (count & 1){
    s = temp2 = prev;
  }
  else{
    s = (temp2 << 8)  + (temp1); 
    s = s >> 6;
    if (is_transmitting)
      q_write(&qout, s);

    //temp2 = ADCH;
    prev = s;
    adcCount++;
  }
  temp1 = 0;
  //temp2 = 0;
  // although ADCH and ADCL are 8b numbers, they are represented
  // here by unsigned ints, just to demonstrate how you would
  // use numbers larger than 8b.  also, be sure you use unsigned
  // ints for this operation.  if you have a signed int (a regular
  // int), add 0x8000 and cast it to an unsigned int before sending
  // it out to OCR1AH or OCR1AL.
  // example:
  // int temp3 = 87;
  // unsigned int temp4 = temp3 + 0x8000;
  // OCR1AH = temp4 >> 8;
  // OCR1AL = temp4;

  if (!is_transmitting){
    if (count & 1)
      s = previouslyReadSample;
    else{
      rx_sample = (q_read(&qout) << 1) + 128;
      if (rx_sample < 0)
        rx_sample = 0;
      else if (rx_sample > 255)
        rx_sample = 255;
      s = rx_sample; 
      previouslyReadSample = s;
    }
  }
  // output high byte on OC1A
  OCR1AH = s >> 8 & 0x03; // takes top 8 bits
  OCR1AL = s   & 0xff ; // takes bottom 8 bits
  
  // output low byte on OC1B
  //OCR1BH = temp1 >> 8;
  //OCR1BL = temp1;
  
  count++;
}



float r = 0.0;
int nextPhase(){
  double v = sin(r) * 1000.0;
  r += 0.1;
  return (int)v;
}

int number = 1;

int txpcm[200];
char txpack[100];
int txptr=0;


void dump(int all){
  Serial.print("\nhead="); Serial.print(qout.head);
  Serial.print(",tail="); Serial.print(qout.tail);
  Serial.print(",txptr="); Serial.println(txptr);

  if (all){
    Serial.println("adpcm:");
    for (int i = 0; i < 100; i++){
      if (i % 20 == 0)
        Serial.println("");
      Serial.print(' ');Serial.print((unsigned int)txpack[i], HEX);
    }
    Serial.println("\npcm:");    
    for (int i = 0; i < 200; i++){
      if (i % 20 == 0)
        Serial.println("");
      Serial.print(' ');Serial.print(txpcm[i]);
    }
    Serial.println("\nqueue:");    
    for (int i = 0; i < MAX_Q; i++){
      Serial.print(' ');Serial.print(qout.data[i], HEX);
      if (i % 20 == 0)
        Serial.println("");
    }

  }
}

unsigned long now = 0, lastTx = 0;
unsigned long txCount = 0;
unsigned long nsamples = 0;
unsigned long rxCount = 0;
unsigned nextComp = 0;

void tx2(){
  int s;

  if (qout.head == qout.tail)
    return;
    
  //fill the pcm encoder buffer with 200 samples
  while (qout.head != qout.tail && txptr < 200){
      s = q_read(&qout); //#1
      s -= 512;
      txpcm[txptr++] = s;
  }

  //Serial.print("\nTX head="); Serial.print(qout.head);
  //Serial.print(" , tail="); Serial.println(qout.tail);

  //proceed if 200 are there
  if (txptr < 200)
    return;

  //.. but only if we have transmitted the previous packet
  if (!LoRa.beginPacket(true)){
    //Serial.print('.');  
    //underflow++;
    return;
  }

/*
  Serial.println("\n************\nbefore:");
  for (int i = 0; i < 200; i++){
    Serial.print(txpcm[i]);Serial.print(' ');
    if (i % 50 == 0)
      Serial.println("");
  }
*/
  //we assume that txpcm is already filled with the samples
  adpcm_coder(txpcm, txpack, 200);
/*
  Serial.println("\nADPCM:");
  for (int i = 0; i < 100; i++){
    Serial.print((uint8_t)txpack[i], HEX);Serial.print(' ');
    if (i % 50 == 0)
      Serial.println("");
  }

  adpcm_decoder(txpack, txpcm, 200);
  int32_t bias=0;
  Serial.println("\nafter:");
  for (int i = 0; i < 200; i++){
    int s = txpcm[i];
    bias += s;
    Serial.print(s);Serial.print(' ');
    if (i % 50 == 0)
      Serial.println("");
  }
  Serial.print("\nbias:");Serial.println(bias/200);
  Serial.println("***");
  if (coder_state_valprev < 500 || coder_state_valprev > 500)
    coder_state_valprev = 0;
*/  
  //dump(1);
  //Serial.print('t');
  LoRa.write('v');
  LoRa.write(100);
  LoRa.write(txpack, 100);
  LoRa.endPacket(true); 
  //dump(1);
  lastTx = now;
  nsamples += 200;
  txptr = 0;
  txCount++;
}

void playSome(){

  //nothing more to be played?
  if (txptr >= 100 || (qout.head + 1 == qout.tail)){
    //Serial.print('*');
    return;
  }

  //Serial.print(txptr);Serial.print(' ');
  char *p = txpack + txptr;
  adpcm_decoder(p, txpcm,50);
  //Serial.print(" valprev:");Serial.print(decoder_state_valprev);
  //Serial.print(" index:");Serial.println(decoder_state_index);



  for (int y=0; y < 50; y++){
     int s = txpcm[y];
     //Serial.print(' ');Serial.print(s, HEX);
     q_write(&qout, s);
    /* Serial.print(s);Serial.print(' ');
    if (y % 50 == 0)
      Serial.println(""); */
     nsamples++;
  }
  txptr += 25;
  //Serial.print("\ntxptr");Serial.println(txptr);
}

void onReceive(int packetSize) {
  int  i, count;

  //Serial.print('{');
  //packetSize = LoRa.available();
  if (packetSize != 102){
    Serial.print('!');
    return;
  }
  if (LoRa.read() != 'v'){
    Serial.print('#');
    return;
  }
  rxCount++;
  count = LoRa.read();
  //Serial.println("packet:");
  //Serial.print(count);
  for (i = 0; i < 100; i++){
    txpack[i] = LoRa.read();
    //Serial.print((unsigned char)txpack[i], HEX);Serial.print(' ');
    //if (i % 50 == 0)
    //  Serial.println("");
    //txpack[i] = s;
  }
  //reset the pointer
  //memset(txpcm, 0, sizeof(txpcm));
  txptr = 0;
}

void initLoRa(){
 LoRa.setPins(LORA_NSS, LORA_NRESET, 2);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

//  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(6); 
  LoRa.setSignalBandwidth(500E3);
  LoRa.setCodingRate4(5);
  //LoRa.setTxPower(2);
  // register the receive callback
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive(102);
}

void setup(){
  Serial.begin(38400);
  Serial.println("azadphone v0.15");
  q_init(&qout);
  initAudio();
  initLoRa();
  pinMode(PTT_PIN, INPUT_PULLUP);
}

unsigned long nextDump = 0;
int commandMode = 0;


void loop() {
  int i, x =0;
  
  now = millis();

  if (!digitalRead(PTT_PIN)){
    is_transmitting = 1;
  }
  else
    is_transmitting = 0;
    
  if (is_transmitting)
    tx2();
  else
    playSome();
 
  if (nextDump < now && is_debug){
    nextDump = now + 1000;
    
    Serial.print("underflow:");Serial.print(underflow);
    Serial.print(" overflow:");Serial.print(overflow_count);
    Serial.print(" rxCount:");Serial.print(rxCount);
    Serial.print(" txCount:");Serial.print(txCount);
    Serial.print(" nsamples:");Serial.println(nsamples);
    
    nsamples = txCount = rxCount = overflow_count = underflow = 0;
  }
  if (Serial.available()){
    char c = Serial.read();
    if (c == 'd'){
      if (is_debug)
        is_debug = 0;
      else
        is_debug = 1; 
    }
    else if (c == 's')
      dump(1);
    else if (c == 'z')
      txptr = 0;
    else if (c == 'h')
      txptr = 100;
    else if (c == 'i')
      adpcm_init();
  }  
  
}
 
 
