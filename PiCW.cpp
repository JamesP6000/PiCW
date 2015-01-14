// See accompanying README and BUILD files for descriptions on how to use this
// code.

/*
License:
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <map>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <random>
#include <atomic>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <malloc.h>
#include <time.h>
#include <sys/time.h>
#include <getopt.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/timex.h>

//using namespace std;

#define ABORT(a) exit(a)
// Used for debugging
#define MARK std::cout << "Currently in file: " << __FILE__ << " line: " << __LINE__ << std::endl

// PLLD clock frequency.
// There seems to be a 2.5ppm offset between the NTP measured frequency
// error and the frequency error measured by a frequency counter. This fixed
// PPM offset is compensated for here.
#define F_PLLD_CLK   (500000000.0*(1-2.500e-6))
// Empirical value for F_PWM_CLK that produces WSPR symbols that are 'close' to
// 0.682s long. For some reason, despite the use of DMA, the load on the PI
// affects the TX length of the symbols. However, the varying symbol length is
// compensated for in the main loop.
#define F_PWM_CLK_INIT (31156186.6125761)

// WSRP nominal symbol time
//#define WSPR_SYMTIME (8192.0/12000.0)
// How much random frequency offset should be added to WSPR transmissions
// if the --offset option has been turned on.
//#define WSPR_RAND_OFFSET 80
//#define WSPR15_RAND_OFFSET 8

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// This must be declared global so that it can be used by the atexit
// function.
volatile unsigned *allof7e = NULL;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13) // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((long int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)
#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define PADS_GPIO_0_27  (0x7e10002c)
#define CM_GP0DIV (0x7e101074)
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)
#define PWMBASE  (0x7e20C000) /* PWM controller */

// g++ 4.7 in c++11 mode had trouble with this struct. I removed it
// and used bit shifts to create the word to be written.
/*
struct GPCTL {
    char SRC         : 4;
    char ENAB        : 1;
    char KILL        : 1;
    char             : 1;
    char BUSY        : 1;
    char FLIP        : 1;
    char MASH        : 2;
    unsigned int     : 13;
    char PASSWD      : 8;
};
*/

struct CB {
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int RES1;
    volatile unsigned int RES2;
};

struct DMAregs {
    volatile unsigned int CS;
    volatile unsigned int CONBLK_AD;
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int DEBUG;
};

struct PageInfo {
    void* p;  // physical address
    void* v;   // virtual address
};

// Get the physical address of a page of virtual memory
void getRealMemPage(void** vAddr, void** pAddr) {
    void* a = (void*)valloc(4096);

    ((int*)a)[0] = 1;  // use page to force allocation.

    mlock(a, 4096);  // lock into ram.

    *vAddr = a;  // yay - we know the virtual address

    unsigned long long frameinfo;

    int fp = open("/proc/self/pagemap", 'r');
    lseek(fp, ((long int)a)/4096*8, SEEK_SET);
    read(fp, &frameinfo, sizeof(frameinfo));

    *pAddr = (void*)((long int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr) {

    munlock(vAddr, 4096);  // unlock ram.

    free(vAddr);
}

void txon()
{
    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

    // Set GPIO drive strength, more info: http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 0;  //2mA -3.4dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 1;  //4mA +2.1dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 2;  //6mA +4.9dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 3;  //8mA +6.6dBm(default)
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 4;  //10mA +8.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 5;  //12mA +9.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 6;  //14mA +10.0dBm
    ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 7;  //16mA +10.6dBm

    //struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 3,0x5a};
    //ACCESS(CM_GP0CTL) = *((int*)&setupword);
    ACCESS(CM_GP0CTL) =
      // PW
      (0x5a<<24) |
      // MASH
      (3<<9) |
      // Flip
      (0<<8) |
      // Busy
      (0<<7) |
      // Kill
      (0<<5) |
      // Enable
      (1<<4) |
      // SRC
      (6<<0)
    ;
}

void txoff()
{
    //struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a};
    //ACCESS(CM_GP0CTL) = *((int*)&setupword);
    ACCESS(CM_GP0CTL) =
      // PW
      (0x5a<<24) |
      // MASH
      (1<<9) |
      // Flip
      (0<<8) |
      // Busy
      (0<<7) |
      // Kill
      (0<<5) |
      // Enable
      (0<<4) |
      // SRC
      (6<<0)
    ;
}

// Transmit tone tone_freq for tsym seconds.
//
// TODO:
// Upon entering this function at the beginning of a WSPR transmission, we
// do not know which DMA table entry is being processed by the DMA engine.
#define PWM_CLOCKS_PER_ITER_NOMINAL 1000
void txSym(
  std::atomic <bool> & terminate,
  const double & tone_freq,
  const double & tsym,
  const std::vector <double> & dma_table_freq,
  const double & f_pwm_clk,
  struct PageInfo instrs[],
  struct PageInfo & constPage,
  int & bufPtr
) {
  const int f0_idx=0;
  const int f1_idx=1;
  const double f0_freq=dma_table_freq[f0_idx];
  const double f1_freq=dma_table_freq[f1_idx];
  // Double check...
  assert((tone_freq>=f0_freq)&&(tone_freq<=f1_freq));
  const double f0_ratio=1.0-(tone_freq-f0_freq)/(f1_freq-f0_freq);
  //cout << "f0_ratio = " << f0_ratio << endl;
  assert ((f0_ratio>=0)&&(f0_ratio<=1));
  const long int n_pwmclk_per_sym=round(f_pwm_clk*tsym);

  long int n_pwmclk_transmitted=0;
  long int n_f0_transmitted=0;
  while ((!terminate)&&(n_pwmclk_transmitted<n_pwmclk_per_sym)) {
    // Number of PWM clocks for this iteration
    long int n_pwmclk=PWM_CLOCKS_PER_ITER_NOMINAL;
    // Iterations may produce spurs around the main peak based on the iteration
    // frequency. Randomize the iteration period so as to spread this peak
    // around.
    n_pwmclk+=round((rand()/((double)RAND_MAX+1.0)-.5)*n_pwmclk)*1;
    if (n_pwmclk_transmitted+n_pwmclk>n_pwmclk_per_sym) {
      n_pwmclk=n_pwmclk_per_sym-n_pwmclk_transmitted;
    }

    // Calculate number of clocks to transmit f0 during this iteration so
    // that the long term average is as close to f0_ratio as possible.
    const long int n_f0=round(f0_ratio*(n_pwmclk_transmitted+n_pwmclk))-n_f0_transmitted;
    const long int n_f1=n_pwmclk-n_f0;

    // Configure the transmission for this iteration
    // Set GPIO pin to transmit f0
    bufPtr++;
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.p + f0_idx*4;

    // Wait for n_f0 PWM clocks
    bufPtr++;
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f0;

    // Set GPIO pin to transmit f1
    bufPtr++;
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (long int)constPage.p + f1_idx*4;

    // Wait for n_f1 PWM clocks
    bufPtr=(bufPtr+1) % (1024);
    while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (long int)(instrs[bufPtr].p)) usleep(100);
    ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = n_f1;

    // Update counters
    n_pwmclk_transmitted+=n_pwmclk;
    n_f0_transmitted+=n_f0;
  }
}

void unSetupDMA(){
    //printf("exiting\n");
    struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
    DMA0->CS =1<<31;  // reset dma controller
    txoff();
}

void handSig(const int h) {
  exit(0);
}

double bit_trunc(
  const double & d,
  const int & lsb
) {
  return floor(d/pow(2.0,lsb))*pow(2.0,lsb);
}

// Setup the DMA table to produce the frequency we need.
// For PiFM, this table had 1024 entries but for this application, we only
// use the first two. The remaining values are filled with dummy data.
void setupDMATab(
  const double & tone_freq,
  const double & plld_actual_freq,
  std::vector <double> & dma_table_freq,
  struct PageInfo & constPage
){
  // Make sure that the tone can be produced solely by
  // varying the fractional part of the frequency divider.
  /*
  center_freq_actual=center_freq_desired;
  double div_lo=bit_trunc(plld_actual_freq/(center_freq_desired-1.5*tone_spacing),-12)+pow(2.0,-12);
  double div_hi=bit_trunc(plld_actual_freq/(center_freq_desired+1.5*tone_spacing),-12);
  if (floor(div_lo)!=floor(div_hi)) {
    center_freq_actual=plld_actual_freq/floor(div_lo)-1.6*tone_spacing;
    stringstream temp;
    temp << setprecision(6) << fixed << "  Warning: center frequency has been changed to " << center_freq_actual/1e6 << " MHz" << endl;
    cout << temp.str();
    cout << "  because of hardware limitations!" << endl;
  }
  */

  // We only really need two tuning words...
  // TODO: It seems to be safe to change the fractional part of the divisor
  // while the clock generator is enabled. Check to see that it is also safe
  // to change the integer part. If it is not safe to change the integer part,
  // then there will be some frequencies which are not synthesizeable.
  std::vector <long int> tuning_word(1024);
  double div=bit_trunc(plld_actual_freq/tone_freq,-12)+pow(2.0,-12);
  tuning_word[0]=((int)(div*pow(2.0,12)));
  div-=pow(2.0,-12);
  tuning_word[1]=((int)(div*pow(2.0,12)));
  // Fill the remaining table, just in case...
  for (int i=8;i<1024;i++) {
    double div=500+i;
    tuning_word[i]=((int)(div*pow(2.0,12)));
  }

  // Create DMA table of tuning words. WSPR tone i will use entries 2*i and
  // 2*i+1 to generate the appropriate tone.
  /*
  dma_table_freq.resize(1024);
  double tone0_freq=center_freq_actual-1.5*tone_spacing;
  vector <long int> tuning_word(1024);
  for (int i=0;i<8;i++) {
    double tone_freq=tone0_freq+(i>>1)*tone_spacing;
    double div=bit_trunc(plld_actual_freq/tone_freq,-12);
    if (i%2==0) {
      div=div+pow(2.0,-12);
    }
    tuning_word[i]=((int)(div*pow(2.0,12)));
  }
  // Fill the remaining table, just in case...
  for (int i=8;i<1024;i++) {
    double div=500+i;
    tuning_word[i]=((int)(div*pow(2.0,12)));
  }
  */

  // Program the table
  for (int i=0;i<1024;i++) {
    dma_table_freq[i]=plld_actual_freq/(tuning_word[i]/pow(2.0,12));
    ((int*)(constPage.v))[i] = (0x5a<<24)+tuning_word[i];
    if ((i%2==0)&&(i<8)) {
      assert((tuning_word[i]&(~0xfff))==(tuning_word[i+1]&(~0xfff)));
    }
  }
}

void setupDMA(
  struct PageInfo & constPage,
  struct PageInfo & instrPage,
  struct PageInfo instrs[]
){
   atexit(unSetupDMA);
   signal (SIGINT, handSig);
   signal (SIGTERM, handSig);
   signal (SIGHUP, handSig);
   signal (SIGQUIT, handSig);

   // Allocate a page of ram for the constants
   getRealMemPage(&constPage.v, &constPage.p);

   // Create 1024 instructions allocating one page at a time.
   // Even instructions target the GP0 Clock divider
   // Odd instructions target the PWM FIFO
   int instrCnt = 0;
   while (instrCnt<1024) {
     // Allocate a page of ram for the instructions
     getRealMemPage(&instrPage.v, &instrPage.p);

     // make copy instructions
     // Only create as many instructions as will fit in the recently
     // allocated page. If not enough space for all instructions, the
     // next loop will allocate another page.
     struct CB* instr0= (struct CB*)instrPage.v;
     int i;
     for (i=0; i<(signed)(4096/sizeof(struct CB)); i++) {
       instrs[instrCnt].v = (void*)((long int)instrPage.v + sizeof(struct CB)*i);
       instrs[instrCnt].p = (void*)((long int)instrPage.p + sizeof(struct CB)*i);
       instr0->SOURCE_AD = (unsigned long int)constPage.p+2048;
       instr0->DEST_AD = PWMBASE+0x18 /* FIF1 */;
       instr0->TXFR_LEN = 4;
       instr0->STRIDE = 0;
       //instr0->NEXTCONBK = (int)instrPage.p + sizeof(struct CB)*(i+1);
       instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
       instr0->RES1 = 0;
       instr0->RES2 = 0;

       // Shouldn't this be (instrCnt%2) ???
       if (i%2) {
         instr0->DEST_AD = CM_GP0DIV;
         instr0->STRIDE = 4;
         instr0->TI = (1<<26/* no wide*/) ;
       }

       if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (long int)instrs[instrCnt].p;
       instr0++;
       instrCnt++;
     }
   }
   // Create a circular linked list of instructions
   ((struct CB*)(instrs[1023].v))->NEXTCONBK = (long int)instrs[0].p;

   // set up a clock for the PWM
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;  // Source=PLLD and disable
   usleep(1000);
   //ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002000;  // set PWM div to 2, for 250MHz
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;  // Source=PLLD and enable
   usleep(1000);

   // set up pwm
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = 0;
   usleep(1000);
   ACCESS(PWMBASE + 0x4 /* status*/) = -1;  // clear errors
   usleep(1000);
   // Range should default to 32, but it is set at 2048 after reset on my RPi.
   ACCESS(PWMBASE + 0x10)=32;
   ACCESS(PWMBASE + 0x20)=32;
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
   usleep(1000);
   ACCESS(PWMBASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;

   //activate dma
   struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
   DMA0->CS =1<<31;  // reset
   DMA0->CONBLK_AD=0;
   DMA0->TI=0;
   DMA0->CONBLK_AD = (unsigned long int)(instrPage.p);
   DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
}


//
// Set up memory regions to access GPIO
//
void setup_io(
  int & mem_fd,
  char * & gpio_mem,
  char * & gpio_map,
  volatile unsigned * & gpio
) {
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit (-1);
    }

    /* mmap GPIO */

    // Allocate MAP block
    if ((gpio_mem = (char *)malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
        printf("allocation error \n");
        exit (-1);
    }

    // Make sure pointer is on 4K boundary
    if ((unsigned long)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

    // Now map it
    gpio_map = (char *)mmap(
                   gpio_mem,
                   BLOCK_SIZE,
                   PROT_READ|PROT_WRITE,
                   MAP_SHARED|MAP_FIXED,
                   mem_fd,
                   GPIO_BASE
               );

    if ((long)gpio_map < 0) {
        printf("mmap error %ld\n", (long int)gpio_map);
        exit (-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


}

void setup_gpios(
  volatile unsigned * & gpio
){
   int g;
   // Switch GPIO 7..11 to output mode

    /************************************************************************\
     * You are about to change the GPIO settings of your computer.          *
     * Mess this up and it will stop working!                               *
     * It might be a good idea to 'sync' before running this program        *
     * so at least you still have your code changes written to the SD-card! *
    \************************************************************************/

    // Set GPIO pins 7-11 to output
    for (g=7; g<=11; g++) {
        INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
        //OUT_GPIO(g);
    }

}

// Convert string to uppercase
/*
void to_upper(char *str)
{   while(*str)
    {
        *str = toupper(*str);
        str++;
    }
}
*/

// Encode call, locator, and dBm into WSPR codeblock.
/*
void wspr(const char* call, const char* l_pre, const char* dbm, unsigned char* symbols)
{
   // pack prefix in nadd, call in n1, grid, dbm in n2
   char* c, buf[16];
   strncpy(buf, call, 16);
   c=buf;
   to_upper(c);
   unsigned long ng,nadd=0;

   if(strchr(c, '/')){ //prefix-suffix
     nadd=2;
     int i=strchr(c, '/')-c; //stroke position
     int n=strlen(c)-i-1; //suffix len, prefix-call len
     c[i]='\0';
     if(n==1) ng=60000-32768+(c[i+1]>='0'&&c[i+1]<='9'?c[i+1]-'0':c[i+1]==' '?38:c[i+1]-'A'+10); // suffix /A to /Z, /0 to /9
     if(n==2) ng=60000+26+10*(c[i+1]-'0')+(c[i+2]-'0'); // suffix /10 to /99
     if(n>2){ // prefix EA8/, right align
       ng=(i<3?36:c[i-3]>='0'&&c[i-3]<='9'?c[i-3]-'0':c[i-3]-'A'+10);
       ng=37*ng+(i<2?36:c[i-2]>='0'&&c[i-2]<='9'?c[i-2]-'0':c[i-2]-'A'+10);
       ng=37*ng+(i<1?36:c[i-1]>='0'&&c[i-1]<='9'?c[i-1]-'0':c[i-1]-'A'+10);
       if(ng<32768) nadd=1; else ng=ng-32768;
       c=c+i+1;
     }
   }

   int i=(isdigit(c[2])?2:isdigit(c[1])?1:0); //last prefix digit of de-suffixed/de-prefixed callsign
   int n=strlen(c)-i-1; //2nd part of call len
   unsigned long n1;
   n1=(i<2?36:c[i-2]>='0'&&c[i-2]<='9'?c[i-2]-'0':c[i-2]-'A'+10);
   n1=36*n1+(i<1?36:c[i-1]>='0'&&c[i-1]<='9'?c[i-1]-'0':c[i-1]-'A'+10);
   n1=10*n1+c[i]-'0';
   n1=27*n1+(n<1?26:c[i+1]-'A');
   n1=27*n1+(n<2?26:c[i+2]-'A');
   n1=27*n1+(n<3?26:c[i+3]-'A');

   //if(rand() % 2) nadd=0;
   if(!nadd){
     // Copy locator locally since it is declared const and we cannot modify
     // its contents in-place.
     char l[4];
     strncpy(l, l_pre, 4);
     to_upper(l); //grid square Maidenhead locator (uppercase)
     ng=180*(179-10*(l[0]-'A')-(l[2]-'0'))+10*(l[1]-'A')+(l[3]-'0');
   }
   int p = atoi(dbm);    //EIRP in dBm={0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
   int corr[]={0,-1,1,0,-1,2,1,0,-1,1};
   p=p>60?60:p<0?0:p+corr[p%10];
   unsigned long n2=(ng<<7)|(p+64+nadd);

   // pack n1,n2,zero-tail into 50 bits
   char packed[11] = {n1>>20, n1>>12, n1>>4, ((n1&0x0f)<<4)|((n2>>18)&0x0f),
n2>>10, n2>>2, (n2&0x03)<<6, 0, 0, 0, 0};

   // convolutional encoding K=32, r=1/2, Layland-Lushbaugh polynomials
   int k = 0;
   int j,s;
   int nstate = 0;
   unsigned char symbol[176];
   for(j=0;j!=sizeof(packed);j++){
      for(i=7;i>=0;i--){
         unsigned long poly[2] = { 0xf2d05351L, 0xe4613c47L };
         nstate = (nstate<<1) | ((packed[j]>>i)&1);
         for(s=0;s!=2;s++){   //convolve
            unsigned long n = nstate & poly[s];
            int even = 0;  // even := parity(n)
            while(n){
               even = 1 - even;
               n = n & (n - 1);
            }
            symbol[k] = even;
            k++;
         }
      }
   }

   // interleave symbols
   const unsigned char npr3[162] = {
      1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,
      0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,
      0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
      0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,
      0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,
      0,0 };
   for(i=0;i!=162;i++){
      // j0 := bit reversed_values_smaller_than_161[i]
      unsigned char j0;
      p=-1;
      for(k=0;p!=i;k++){
         for(j=0;j!=8;j++)   // j0:=bit_reverse(k)
           j0 = ((k>>j)&1)|(j0<<1);
         if(j0<162)
           p++;
      }
      symbols[j0]=npr3[j0]|symbol[i]<<1; //interleave and add sync vector
   }
}
*/

// Wait for the system clock's minute to reach one second past 'minute'
/*
void wait_every(int minute)
{
  time_t t;
  struct tm* ptm;
  for(;;){
    time(&t);
    ptm = gmtime(&t);
    if((ptm->tm_min % minute) == 0 && ptm->tm_sec == 0) break;
    usleep(1000);
  }
  usleep(1000000); // wait another second
}
*/

void print_usage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "  PiCW [options] \"MORSE TEXT TO SEND\"" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -h --help" << std::endl;
  std::cout << "    Print out this help screen." << std::endl;
  std::cout << "  -f --freq f" << std::endl;
  std::cout << "    Specify the frequency to be used for the transmission" << std::endl;
  std::cout << "  -w --wpm w" << std::endl;
  std::cout << "    Specify the transmission speed in Words Per Minute" << std::endl;
  std::cout << "  -p --ppm ppm" << std::endl;
  std::cout << "    Known PPM correction to 19.2MHz RPi nominal crystal frequency." << std::endl;
  std::cout << "  -s --self-calibration" << std::endl;
  std::cout << "    Call ntp_adjtime() periodically to obtain the PPM error of the crystal." << std::endl;
}

void parse_commandline(
  // Inputs
  const int & argc,
  char * const argv[],
  // Outputs
  double & tone_freq,
  double & wpm,
  double & ppm,
  bool & self_cal,
  std::string & str
) {
  // Default values
  tone_freq=0;
  wpm=20;
  ppm=0;
  self_cal=false;
  str="";

  static struct option long_options[] = {
    {"help",             no_argument,       0, 'h'},
    {"freq",             required_argument, 0, 'f'},
    {"wpm",              required_argument, 0, 'w'},
    {"ppm",              required_argument, 0, 'p'},
    {"self-calibration", no_argument,       0, 's'},
    {0, 0, 0, 0}
  };

  while (1) {
    /* getopt_long stores the option index here. */
    int option_index = 0;
    int c = getopt_long (argc, argv, "hf:w:p:s",
                     long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      char * endp;
      case 0:
        // Code should only get here if a long option was given a non-null
        // flag value.
        std::cout << "Check code!" << std::endl;
        ABORT(-1);
        break;
      case 'h':
        print_usage();
        ABORT(-1);
        break;
      case 'f':
        tone_freq=strtod(optarg,&endp);
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse frequency" << std::endl;
          ABORT(-1);
        }
        break;
      case 'w':
        wpm=strtod(optarg,&endp);
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse wpm value" << std::endl;
          ABORT(-1);
        }
        break;
      case 'p':
        ppm=strtod(optarg,&endp);
        if ((optarg==endp)||(*endp!='\0')) {
          std::cerr << "Error: could not parse ppm value" << std::endl;
          ABORT(-1);
        }
        break;
      case 's':
        self_cal=true;
        break;
      case '?':
        /* getopt_long already printed an error message. */
        ABORT(-1);
      default:
        ABORT(-1);
    }

  }

  // Parse the non-option parameters
  while (optind<argc) {
    if (!str.empty()) {
      str+=" ";
    }
    str==argv[optind++];
  }

  // Check consistency among command line options.
  if (ppm&&self_cal) {
    std::cout << "Warning: ppm value is being ignored!" << std::endl;
    ppm=0.0;
  }

  // Print a summary of the parsed options
  std::cout << "PiCW parsed command line options:" << std::endl;
  std::stringstream temp;
  temp << std::setprecision(6) << std::fixed;
  temp << tone_freq/1e6 << " MHz";
  std::cout << "  TX frequency: " << temp.str() << std::endl;
  temp.str("");
  std::cout << "  WPM: " << wpm << std::endl;
  if (self_cal) {
    temp << "  ntp_adjtime() will be used to periodically calibrate the transmission frequency" << std::endl;
  } else if (ppm) {
    temp << "  PPM value to be used for all transmissions: " << ppm << std::endl;
  }
}

// Call ntp_adjtime() to obtain the latest calibration coefficient.
void update_ppm(
  double & ppm
) {
  struct timex ntx;
  int status;
  double ppm_new;

  ntx.modes = 0; /* only read */
  status = ntp_adjtime(&ntx);

  if (status != TIME_OK) {
    //cerr << "Error: clock not synchronized" << std::endl;
    //return;
  }

  ppm_new = (double)ntx.freq/(double)(1 << 16); /* frequency scale */
  if (abs(ppm_new)>200) {
    std::cerr << "Warning: absolute ppm value is greater than 200 and is being ignored!" << std::endl;
  } else {
    if (ppm!=ppm_new) {
      std::cout << "  Obtained new ppm value: " << ppm_new << std::endl;
    }
    ppm=ppm_new;
  }
}

/* Return 1 if the difference is negative, otherwise 0.  */
// From StackOverflow:
// http://stackoverflow.com/questions/1468596/c-programming-calculate-elapsed-time-in-milliseconds-unix
/*
int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1) {
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return (diff<0);
}
*/

/*
void timeval_print(struct timeval *tv) {
    char buffer[30];
    time_t curtime;

    //printf("%ld.%06ld", tv->tv_sec, tv->tv_usec);
    curtime = tv->tv_sec;
    //strftime(buffer, 30, "%m-%d-%Y %T", localtime(&curtime));
    strftime(buffer, 30, "UTC %m-%d-%Y %T", gmtime(&curtime));
    printf("%s.%03ld", buffer, (tv->tv_usec+500)/1000);
}
*/

// This thread manages the tone being produced. If the desired frequency
// changes, or if the PPM value is updated, this thread will take appropriate
// measures to ensure that the tone being produced is as close as possible
// to the frequency that is desired.
void tone_main(
  std::atomic <bool> & terminate,
  const bool & self_cal,
  const double & ppm_init,
  std::atomic <double> & freq,
  struct PageInfo instrs[],
  struct PageInfo & constPage
) {
  // Initialize
  double ppm=ppm_init;
  if (self_cal) {
    update_ppm(ppm);
  }
  double ppm_old=ppm;
  double freq_old=freq;
  std::vector <double> dma_table_freq;
  setupDMATab(freq_old,F_PLLD_CLK*(1-ppm_old/1e6),dma_table_freq,constPage);
  int bufPtr=0;

  while (!terminate) {
    // Read the current values of the atomics.
    double freq_new=freq;
    double ppm_new=ppm;

    // Update table if necessary.
    if (
      (ppm_new!=ppm_old) ||
      (freq_new<dma_table_freq[0]) ||
      (freq_new>dma_table_freq[1])
    ) {
      setupDMATab(freq_new,F_PLLD_CLK*(1-ppm_new/1e6),dma_table_freq,constPage);
    }

    // Transmit for a small amount of time before checking for updates to
    // frequency or PPM.
    double tx_time_secs=1.0;
    txSym(
      terminate,
      freq_new,
      tx_time_secs,
      dma_table_freq,
      F_PWM_CLK_INIT,
      instrs,
      constPage,
      bufPtr
    );

    freq_old=freq_new;
    ppm_old=ppm_new;
  }
}

class time_value {
  public:
    std::chrono::duration <double> time;
    unsigned int value;
};

// Raised cosine pulse shapes.
// Rise:
//y=(-cos(t*pi)+1)/2;
//2*y-1=-cos(t*pi);
//1-2*y=cos(t*pi);
//acos(1-2*y)=t*pi;
//acos(1-2*y)/pi=t;
// Fall:
//y=1-(-cos(t*pi)+1)/2;
//y-1=-(-cos(t*pi)+1)/2;
//2*y-2=-(-cos(t*pi)+1);
//2-2*y=-cos(t*pi)+1;
//1-2*y=-cos(t*pi);
//2*y-1=cos(t*pi);
//acos(2*y-1)=t*pi;
//acos(2*y-1)/pi=t;
// Instead of uniform sampling of the x axis, we're using uniform
// sampling of the y axis.
void raised_cosine(
  const double & width_secs,
  std::vector <time_value> & rise,
  std::vector <time_value> & fall
) {
  rise.resize(0);
  rise.reserve(10);
  fall.resize(0);
  fall.reserve(10);
  {
    time_value rec;
    rec.value=0;
    rec.time=std::chrono::duration <double> (0);
    rise.push_back(rec);
    rec.value=8;
    fall.push_back(rec);
  }
  for (double y=0.5/8.0;y<1;y+=1.0/8.0) {
    time_value rec;
    rec.value=round(y*8.0+0.5);
    rec.time=std::chrono::duration <double> (acos(1-2*y)/M_PI*width_secs);
    rise.push_back(rec);
  }
  for (double y=7.5/8.0;y>0;y-=1.0/8.0) {
    time_value rec;
    rec.value=round(y*8.0-0.5);
    rec.time=std::chrono::duration <double> (acos(2*y-1)/M_PI*width_secs);
    fall.push_back(rec);
  }
  {
    time_value rec;
    rec.value=8;
    rec.time=std::chrono::duration <double> (width_secs);
    rise.push_back(rec);
    rec.value=0;
    fall.push_back(rec);
  }
}

// Adjust the drive current on the pin.
void set_current(
  unsigned int value
) {
  if (value>8) {
    value=8;
  }
  if (value==0) {
    //struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a};
    //ACCESS(CM_GP0CTL) = *((int*)&setupword);
    ACCESS(CM_GP0CTL) =
      // PW
      (0x5a<<24) |
      // MASH
      (1<<9) |
      // Flip
      (0<<8) |
      // Busy
      (0<<7) |
      // Kill
      (0<<5) |
      // Enable
      (0<<4) |
      // SRC
      (6<<0)
    ;
  } else {
    ACCESS(PADS_GPIO_0_27) = 0x5a000018 + ((value - 1)&0x7);
    //struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 3,0x5a};
    //ACCESS(CM_GP0CTL) = *((int*)&setupword);
    ACCESS(CM_GP0CTL) =
      // PW
      (0x5a<<24) |
      // MASH
      (3<<9) |
      // Flip
      (0<<8) |
      // Busy
      (0<<7) |
      // Kill
      (0<<5) |
      // Enable
      (1<<4) |
      // SRC
      (6<<0)
    ;
  }
}

// Send either a dit or a dah
void send_dit_dah(
  std::atomic <bool> & terminate,
  const char & sym,
  const double & dot_duration_sec,
  std::mt19937 & gen
) {
  // Setting ramp_excess to 0 will produce hard keying. A ramp_excess value
  // of 1.0 will produce a dit that has a ramp going up, a ramp going down,
  // and no flat portion.
  const double ramp_excess=0.3;
  const std::chrono::duration <double> ramp_time(dot_duration_sec*ramp_excess);
  const std::chrono::duration <double> flat_time(dot_duration_sec*(1-ramp_excess)+((sym=='-')?(2*dot_duration_sec):(0)));
  // Jitter adjusts the timing of the rising and falling ramp. This serves
  // to spread out the harmonics that are created.
  const double jitter_factor=0.1;
  std::uniform_real_distribution<> dis(0,jitter_factor*dot_duration_sec);
  const std::chrono::duration <double> jitter_rise(dis(gen));
  const std::chrono::duration <double> jitter_fall(dis(gen));

  // Calculate the rise and fall ramps.
  static bool initialized=false;
  static std::chrono::duration <double> ramp_time_prev(0);
  static std::vector <time_value> rise;
  static std::vector <time_value> fall;
  if ((!initialized)||(ramp_time_prev!=ramp_time)) {
    raised_cosine(
      ramp_time.count(),
      rise,
      fall
    );
    initialized=true;
  }

  // Pulse will be timed relative to the current time.
  std::chrono::high_resolution_clock::time_point ref=std::chrono::high_resolution_clock::now();

  // Delay the rising ramp.
  //std::chrono::duration <double> jitter_rise_duration(jitter_rise);
  std::this_thread::sleep_until(ref+jitter_rise);
  if (terminate) {
    return;
  }
  // Rising ramp.
  for (auto & tv:rise) {
    std::this_thread::sleep_until(ref+jitter_rise+tv.time);
    if (terminate) {
      return;
    }
    set_current(tv.value);
  }
  // Keep transmitting at full power until after the flat portion and after
  // the second jitter delay.
  std::this_thread::sleep_until(ref+ramp_time+flat_time+jitter_fall);
  if (terminate) {
    return;
  }
  // Falling ramp.
  for (auto & tv:fall) {
    std::this_thread::sleep_until(ref+ramp_time+flat_time+jitter_fall+tv.time);
    if (terminate) {
      return;
    }
    set_current(tv.value);
  }
}

// This is the thread that modulates the carrier and produces the dits and
// dahs.
void am_main(
  std::atomic <bool> & terminate,
  std::deque <char> & queue,
  std::mutex & queue_mutex,
  std::condition_variable & queue_signal,
  std::map <char,std::string> & morse_table,
  std::atomic <double> & wpm,
  std::atomic <bool> & busy
) {
  bool prev_char_whitespace=true;
  std::chrono::time_point <std::chrono::high_resolution_clock,std::chrono::duration <double>> earliest_tx_time=std::chrono::high_resolution_clock::now();

  std::random_device rd;
  std::mt19937 gen(rd());

  while (true) {
    busy=false;

    // Get the next character from the queue.
    char tx_char='\0';
    {
      std::unique_lock <std::mutex> lock(queue_mutex);
      if (terminate) {
        return;
      }
      while (queue.empty()) {
        queue_signal.wait_for(lock,std::chrono::milliseconds(100));
        if (terminate) {
          return;
        }
      }
      tx_char=queue.front();
      queue.pop_front();
      busy=true;
    }

    // Sample (and hold) wpm.
    const double dot_duration_sec=1.2/wpm;

    // Handle whitespace.
    if ((tx_char==' ')||(tx_char=='\n')) {
      if (prev_char_whitespace) {
        // Ignore multiple whitespaces.
        continue;
      } else {
        earliest_tx_time=earliest_tx_time+std::chrono::duration <double> (4*dot_duration_sec);
        prev_char_whitespace=true;
        continue;
      }
    }
    prev_char_whitespace=false;

    if (morse_table.find(tx_char)==morse_table.end()) {
      // We should never get here... Only characters in morse code table
      // should ever get forwarded here.
      MARK;
      ABORT(-1);
    }

    // See if we have already waited enough time between characters.
    if (std::chrono::high_resolution_clock::now()>=earliest_tx_time) {
      earliest_tx_time=std::chrono::high_resolution_clock::now();
    }

    // Send the dits and dahs
    std::string tx_pattern=morse_table[tx_char];
    for (unsigned int t=0;t<tx_pattern.length();t++) {
      std::this_thread::sleep_until(earliest_tx_time);
      if (terminate) {
        return;
      }
      const char sym=tx_pattern[t];
      send_dit_dah(terminate,sym,dot_duration_sec,gen);
      if (sym=='.') {
        earliest_tx_time+=std::chrono::duration <double> (2*dot_duration_sec);
      } else {
        earliest_tx_time+=std::chrono::duration <double> (4*dot_duration_sec);
      }
    }
    earliest_tx_time+=std::chrono::duration <double> (3*dot_duration_sec);

  }
}

void morse_table_init(
  std::map <char,std::string> & morse_table
) {
  morse_table.clear();
  morse_table['A']=".-";
  morse_table['B']="-...";
  morse_table['C']="-.-.";
  morse_table['D']="-..";
  morse_table['E']=".";
  morse_table['F']="..-.";
  morse_table['G']="--.";
  morse_table['H']="....";
  morse_table['I']="..";
  morse_table['J']=".---";
  morse_table['K']="-.-";
  morse_table['L']=".-..";
  morse_table['M']="--";
  morse_table['N']="-.";
  morse_table['O']="---";
  morse_table['P']=".--.";
  morse_table['Q']="--.-";
  morse_table['R']=".-.";
  morse_table['S']="...";
  morse_table['T']="-";
  morse_table['U']="..-";
  morse_table['V']="...-";
  morse_table['W']=".--";
  morse_table['X']="-..-";
  morse_table['Y']="-.--";
  morse_table['Z']="--..";
  morse_table['0']="-----";
  morse_table['1']=".----";
  morse_table['2']="..---";
  morse_table['3']="...--";
  morse_table['4']="....-";
  morse_table['5']=".....";
  morse_table['6']="-....";
  morse_table['7']="--...";
  morse_table['8']="---..";
  morse_table['9']="----.";
  morse_table['.']=".−.−.−";
  morse_table[',']="−−..−−";
  morse_table[':']="−−−...";
  morse_table['?']="..−−..";
  morse_table['\'']=".−−−−.";
  morse_table['-']="−....−";
  morse_table['/']="−..−.";
  morse_table['(']="−.−−.";
  morse_table[')']="−.−−.−";
  morse_table['"']=".−..−.";
  morse_table['=']="−...−";
  morse_table['+']=".−.−.";
  morse_table['*']="−..−";
  morse_table['@']=".––.–.";
}

int main(const int argc, char * const argv[]) {
  // Initialize the RNG
  //srand(time(NULL));

  // Parse arguments
  double freq_init;
  double wpm_init;
  double ppm_init;
  bool self_cal;
  std::string str;
  parse_commandline(
    argc,
    argv,
    freq_init,
    wpm_init,
    ppm_init,
    self_cal,
    str
  );

  // Initial configuration
  int mem_fd;
  char *gpio_mem, *gpio_map;
  volatile unsigned *gpio = NULL;
  setup_io(mem_fd,gpio_mem,gpio_map,gpio);
  setup_gpios(gpio);
  allof7e = (unsigned *)mmap(
              NULL,
              0x01000000,  //len
              PROT_READ|PROT_WRITE,
              MAP_SHARED,
              mem_fd,
              0x20000000  //base
          );
  if ((long int)allof7e==-1) {
    std::cerr << "Error: mmap error!" << std::endl;
    ABORT(-1);
  }
  txon();
  struct PageInfo constPage;
  struct PageInfo instrPage;
  struct PageInfo instrs[1024];
  setupDMA(constPage,instrPage,instrs);
  txoff();

  // Morse code table.
  std::map <char,std::string> morse_table;
  morse_table_init(morse_table);

  // Atomics used for IPC
  std::atomic <double> tone_freq;
  tone_freq=freq_init;
  std::atomic <double> wpm;
  wpm=wpm_init;

  // Start tone thread.
  std::atomic <bool> terminate_tone_thread;
  terminate_tone_thread=false;
  std::thread tone_thread(tone_main,
    std::ref(terminate_tone_thread),
    std::ref(self_cal),
    std::ref(ppm_init),
    std::ref(tone_freq),
    instrs,
    std::ref(constPage)
  );

  // Start AM thread
  std::atomic <bool> terminate_am_thread;
  terminate_am_thread=false;
  std::deque <char> queue;
  std::mutex queue_mutex;
  std::condition_variable queue_signal;
  std::atomic <bool> am_thread_busy;
  am_thread_busy=false;
  std::thread am_thread(am_main,
    std::ref(terminate_am_thread),
    std::ref(queue),
    std::ref(queue_mutex),
    std::ref(queue_signal),
    std::ref(morse_table),
    std::ref(wpm),
    std::ref(am_thread_busy)
  );

  // Push text into AM thread
  {
    std::unique_lock <std::mutex> lock(queue_mutex);
    for (unsigned int t=0;t<str.length();t++) {
      char ch=toupper(str[t]);
      if ((ch==' ')||(ch=='\n')||(morse_table.find(ch)!=morse_table.end())) {
        queue.push_back(ch);
      }
    }
    queue_signal.notify_one();
  }

  // Wait for queue to be emptied.
  while (true) {
    {
      std::unique_lock <std::mutex> lock(queue_mutex);
      if (queue.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Wait for final character to be transmitted.
  while (am_thread_busy) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Terminate subthreads
  terminate_am_thread=true;
  terminate_tone_thread=true;
  if (am_thread.joinable()) {
    am_thread.join();
  }
  if (tone_thread.joinable()) {
    tone_thread.join();
  }

  /*
  if (mode==TONE) {
    // Test tone mode...
    double wspr_symtime = WSPR_SYMTIME;
    double tone_spacing=1.0/wspr_symtime;

    stringstream temp;
    temp << setprecision(6) << fixed << "Transmitting test tone on frequency " << test_tone/1.0e6 << " MHz" << endl;
    cout << temp.str();
    cout << "Press CTRL-C to exit!" << endl;

    txon();
    int bufPtr=0;
    vector <double> dma_table_freq;
    // Set to non-zero value to ensure setupDMATab is called at least once.
    double ppm_prev=123456;
    double center_freq_actual;
    while (true) {
      if (self_cal) {
        update_ppm(ppm);
      }
      if (ppm!=ppm_prev) {
        setupDMATab(test_tone+1.5*tone_spacing,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
        //cout << setprecision(30) << dma_table_freq[0] << endl;
        //cout << setprecision(30) << dma_table_freq[1] << endl;
        //cout << setprecision(30) << dma_table_freq[2] << endl;
        //cout << setprecision(30) << dma_table_freq[3] << endl;
        if (center_freq_actual!=test_tone+1.5*tone_spacing) {
          cout << "  Warning: because of hardware limitations, test tone will be transmitted on" << endl;
          stringstream temp;
          temp << setprecision(6) << fixed << "  frequency: " << (center_freq_actual-1.5*tone_spacing)/1e6 << " MHz" << endl;
          cout << temp.str();
        }
        ppm_prev=ppm;
      }
      txSym(0, center_freq_actual, tone_spacing, 60, dma_table_freq, F_PWM_CLK_INIT, instrs, constPage, bufPtr);
    }

    // Should never get here...

  } else {
    // WSPR mode

    // Create WSPR symbols
    unsigned char symbols[162];
    wspr(callsign.c_str(), locator.c_str(), tx_power.c_str(), symbols);

    printf("Ready to transmit (setup comlete)...\n");
    int band=0;
    int n_tx=0;
    for(;;) {
      // Calculate WSPR parameters for this transmission
      double center_freq_desired;
      center_freq_desired = center_freq_set[band];
      bool wspr15 =
           (center_freq_desired > 137600 && center_freq_desired < 137625) || \
           (center_freq_desired > 475800 && center_freq_desired < 475825) || \
           (center_freq_desired > 1838200 && center_freq_desired < 1838225);
      double wspr_symtime = (wspr15) ? 8.0 * WSPR_SYMTIME : WSPR_SYMTIME;
      double tone_spacing=1.0/wspr_symtime;

      // Add random offset
      if ((center_freq_desired!=0)&&random_offset) {
        center_freq_desired+=(2.0*rand()/((double)RAND_MAX+1.0)-1.0)*(wspr15?WSPR15_RAND_OFFSET:WSPR_RAND_OFFSET);
      }

      // Status message before transmission
      stringstream temp;
      temp << setprecision(6) << fixed;
      temp << "Desired center frequency for " << (wspr15?"WSPR-15":"WSPR") << " transmission: "<< center_freq_desired/1e6 << " MHz" << endl;
      cout << temp.str();

      // Wait for WSPR transmission window to arrive.
      if (no_delay) {
        cout << "  Transmitting immediately (not waiting for WSPR window)" << endl;
      } else {
        printf("  Waiting for next WSPR transmission window...\n");
        wait_every((wspr15) ? 15 : 2);
      }

      // Update crystal calibration information
      if (self_cal) {
        update_ppm(ppm);
      }

      // Create the DMA table for this center frequency
      vector <double> dma_table_freq;
      double center_freq_actual;
      if (center_freq_desired) {
        setupDMATab(center_freq_desired,tone_spacing,F_PLLD_CLK*(1-ppm/1e6),dma_table_freq,center_freq_actual,constPage);
      } else {
        center_freq_actual=center_freq_desired;
      }

      // Send the message!
      //cout << "TX started!" << endl;
      if (center_freq_actual){
        // Print a status message right before transmission begins.
        struct timeval tvBegin, tvEnd, tvDiff;
        gettimeofday(&tvBegin, NULL);
        cout << "  TX started at: ";
        timeval_print(&tvBegin);
        cout << endl;

        struct timeval sym_start;
        struct timeval diff;
        int bufPtr=0;
        txon();
        for (int i = 0; i < 162; i++) {
          gettimeofday(&sym_start,NULL);
          timeval_subtract(&diff, &sym_start, &tvBegin);
          double elapsed=diff.tv_sec+diff.tv_usec/1e6;
          //elapsed=(i)*wspr_symtime;
          double sched_end=(i+1)*wspr_symtime;
          //cout << "symbol " << i << " " << wspr_symtime << endl;
          //cout << sched_end-elapsed << endl;
          double this_sym=sched_end-elapsed;
          this_sym=(this_sym<.2)?.2:this_sym;
          this_sym=(this_sym>2*wspr_symtime)?2*wspr_symtime:this_sym;
          txSym(symbols[i], center_freq_actual, tone_spacing, sched_end-elapsed, dma_table_freq, F_PWM_CLK_INIT, instrs, constPage, bufPtr);
        }
        n_tx++;

        // Turn transmitter off
        txoff();

        gettimeofday(&tvEnd, NULL);
        cout << "  TX ended at:   ";
        timeval_print(&tvEnd);
        timeval_subtract(&tvDiff, &tvEnd, &tvBegin);
        printf(" (%ld.%03ld s)\n", tvDiff.tv_sec, (tvDiff.tv_usec+500)/1000);

      } else {
        cout << "  Skipping transmission" << endl;
        usleep(1000000);
      }

      // Advance to next band
      band=(band+1)%nbands;
      if ((band==0)&&!repeat) {
        break;
      }
      if ((terminate>0)&&(n_tx>=terminate)) {
        break;
      }

    }
  }
  */

  return 0;
}

