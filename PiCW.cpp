// See accompanying README and BUILD files for descriptions on how to use this
// code.

// License:
//   This program is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Authors:
//   Oliver Mattos, Oskar Weigl, Dan Ankers (MD1CLV), Guido (PE1NNZ),
//   Michael Tatarinov, James Peroulas (AB0JP)

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
    // Turn off GPIO clock
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

  // Program the table
  dma_table_freq.resize(1024);
  for (int i=0;i<1024;i++) {
    dma_table_freq[i]=plld_actual_freq/(tuning_word[i]/pow(2.0,12));
    ((int*)(constPage.v))[i] = (0x5a<<24)+tuning_word[i];
    //if ((i%2==0)&&(i<8)) {
    //  assert((tuning_word[i]&(~0xfff))==(tuning_word[i+1]&(~0xfff)));
    //}
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

void print_usage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "  PiCW [options] \"text to send in Morse code\"" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -h --help" << std::endl;
  std::cout << "    Print out this help screen." << std::endl;
  std::cout << "  -f --freq f" << std::endl;
  std::cout << "    Specify the frequency to be used for the transmission." << std::endl;
  std::cout << "  -w --wpm w" << std::endl;
  std::cout << "    Specify the transmission speed in Words Per Minute (default 20 WPM)." << std::endl;
  std::cout << "  -p --ppm ppm" << std::endl;
  std::cout << "    Known PPM correction to 19.2MHz RPi nominal crystal frequency." << std::endl;
  std::cout << "  -s --self-calibration" << std::endl;
  std::cout << "    Call ntp_adjtime() periodically to obtain the PPM error of the crystal." << std::endl;
  std::cout << "  -d --ditdit" << std::endl;
  std::cout << "    Transmit an endless series of dits. Can be used to measure TX spectrum." << std::endl;
  std::cout << "  -t --test-tone" << std::endl;
  std::cout << "    Continuously transmit a test tone at the requested frequency." << std::endl;
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
  std::string & str,
  bool & ditdit,
  bool & test_tone
) {
  // Default values
  tone_freq=NAN;
  wpm=20;
  ppm=0;
  self_cal=false;
  str="";
  ditdit=false;
  test_tone=false;

  static struct option long_options[] = {
    {"help",             no_argument,       0, 'h'},
    {"freq",             required_argument, 0, 'f'},
    {"wpm",              required_argument, 0, 'w'},
    {"ppm",              required_argument, 0, 'p'},
    {"self-calibration", no_argument,       0, 's'},
    {"ditdit",           no_argument,       0, 'd'},
    {"test-tone",        no_argument,       0, 't'},
    {0, 0, 0, 0}
  };

  while (1) {
    /* getopt_long stores the option index here. */
    int option_index = 0;
    int c = getopt_long (argc, argv, "hf:w:p:sdt",
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
      case 'd':
        ditdit=true;
        break;
      case 't':
        test_tone=true;
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
    str+=argv[optind++];
  }

  // Check consistency among command line options.
  if (ppm&&self_cal) {
    std::cout << "Warning: ppm value is being ignored!" << std::endl;
    ppm=0.0;
  }
  if (isnan(tone_freq)) {
    std::cerr << "Error: must specify TX frequency (try --help)" << std::endl;
    ABORT(-1);
  }
  if ((!str.empty())&&ditdit) {
    std::cerr << "Error: cannot transmit text when ditdit mode is requested" << std::endl;
    ABORT(-1);
  }
  if ((!str.empty())&&test_tone) {
    std::cerr << "Error: cannot transmit text when test-tone mode is requested" << std::endl;
    ABORT(-1);
  }
  if (test_tone&&ditdit) {
    std::cerr << "Error: cannot request test-tone and ditdit modes at the same time" << std::endl;
    ABORT(-1);
  }

  // Print a summary of the parsed options
  std::cout << "PiCW parsed command line options:" << std::endl;
  std::stringstream temp;
  temp << std::setprecision(6) << std::fixed;
  temp << tone_freq/1e6 << " MHz";
  std::cout << "  TX frequency: " << temp.str() << std::endl;
  temp.str("");
  if (!test_tone) {
    std::cout << "  WPM: " << wpm << std::endl;
  }
  if (self_cal) {
    temp << "  ntp_adjtime() will be used to periodically calibrate the transmission frequency" << std::endl;
  } else if (ppm) {
    temp << "  PPM value to be used for all transmissions: " << ppm << std::endl;
  }
  if (ditdit) {
    std::cout << "Will transmit an endless series of dits. CTRL-C to exit." << std::endl;
  } else if (test_tone) {
    std::cout << "Will transmit continuous tone on frequency. CTRL-C to exit." << std::endl;
  } else {
    std::cout << "Message to be sent:" << std::endl;
    std::cout << '"' << str << '"' << std::endl;
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
      //std::cout << "  Obtained new ppm value: " << ppm_new << std::endl;
    }
    ppm=ppm_new;
  }
}

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
  struct PageInfo & constPage,
  std::atomic <bool> & tone_thread_ready
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
    tone_thread_ready=true;
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

// The rise and fall ramps are stored as collections of time/value pairs.
class time_value {
  public:
    std::chrono::duration <double> time;
    unsigned int value;
};

// Rectangular ramp that simply goes high or low in the middle of the ramp
// period.
void rectangle(
  const double & width_secs,
  std::vector <time_value> & rise,
  std::vector <time_value> & fall
) {
  rise.resize(0);
  rise.reserve(3);
  fall.resize(0);
  fall.reserve(3);
  {
    time_value rec;
    rec.value=0;
    rec.time=std::chrono::duration <double> (0);
    rise.push_back(rec);
    rec.value=8;
    fall.push_back(rec);
  }
  {
    time_value rec;
    rec.value=8;
    rec.time=std::chrono::duration <double> (width_secs/2);
    rise.push_back(rec);
    rec.value=0;
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

// Raised cosine rise/ fall ramps.
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
    // Turn off output
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
    // Set drive strength
    ACCESS(PADS_GPIO_0_27) = 0x5a000018 + ((value - 1)&0x7);
    // Turn on output
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

  // Calculate the rise and fall ramps, if needed.
  static bool initialized=false;
  static std::chrono::duration <double> ramp_time_prev(0);
  static std::vector <time_value> rise;
  static std::vector <time_value> fall;
  if ((!initialized)||(ramp_time_prev!=ramp_time)) {
#if 1
    raised_cosine(
      ramp_time.count(),
      rise,
      fall
    );
#else
    rectangle(
      ramp_time.count(),
      rise,
      fall
    );
#endif
    initialized=true;
  }

  // Dit or dah pulse will be timed relative to the current time.
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
  std::atomic <bool> & busy,
  const bool & ditdit,
  const bool & test_tone
) {
  // In the case of a test tone, set the drive strength to maximum and
  // turn on output. Nothing else.
  if (test_tone) {
    set_current(8);
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (terminate) {
        return;
      }
    }
  }

  bool prev_char_whitespace=true;
  std::chrono::time_point <std::chrono::high_resolution_clock,std::chrono::duration <double>> earliest_tx_time=std::chrono::high_resolution_clock::now();

  std::random_device rd;
  std::mt19937 gen(rd());

  while (true) {
    busy=false;

    // Get the next character from the queue.
    char tx_char='\0';
    if (!ditdit) {
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
      std::cout << tx_char;
      std::cout.flush();
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

    if ((!ditdit)&&(morse_table.find(tx_char)==morse_table.end())) {
      // We should never get here... Only characters in morse code table
      // should ever get forwarded here.
      ABORT(-1);
    }

    // See if we have already waited enough time between characters.
    if (std::chrono::high_resolution_clock::now()>=earliest_tx_time) {
      earliest_tx_time=std::chrono::high_resolution_clock::now();
    }

    // Send the dits and dahs
    std::string tx_pattern;
    if (ditdit) {
      tx_pattern=".....";
    } else {
      tx_pattern=morse_table[tx_char];
    }
    for (unsigned int t=0;t<tx_pattern.length();t++) {
      std::this_thread::sleep_until(earliest_tx_time);
      if (terminate) {
        return;
      }
      if ((!ditdit)&&(t==0)) {
        std::cout << tx_char;
        std::cout.flush();
      }
      const char sym=tx_pattern[t];
      send_dit_dah(terminate,sym,dot_duration_sec,gen);
      if (sym=='.') {
        earliest_tx_time+=std::chrono::duration <double> (2*dot_duration_sec);
      } else {
        earliest_tx_time+=std::chrono::duration <double> (4*dot_duration_sec);
      }
      if (ditdit) {
        t=0;
      }
    }
    earliest_tx_time+=std::chrono::duration <double> (2*dot_duration_sec);
  }
}

// Initialize the morse code table.
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
  morse_table['.']=".-.-.-";
  morse_table[',']="--..--";
  morse_table[':']="---...";
  morse_table['?']="..--..";
  morse_table['\'']=".----.";
  morse_table['-']="-....-";
  morse_table['/']="-..-.";
  morse_table['(']="-.--.";
  morse_table[')']="-.--.-";
  morse_table['"']=".-..-.";
  morse_table['=']="-...-";
  morse_table['+']=".-.-.";
  morse_table['*']="-..-";
  morse_table['@']=".--.-.";
}

int main(const int argc, char * const argv[]) {
  // Parse arguments
  double freq_init;
  double wpm_init;
  double ppm_init;
  bool self_cal;
  std::string str;
  bool ditdit;
  bool test_tone;
  parse_commandline(
    argc,
    argv,
    freq_init,
    wpm_init,
    ppm_init,
    self_cal,
    str,
    ditdit,
    test_tone
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

  // Configure GPIO4
  SETBIT(GPFSEL0 , 14);
  CLRBIT(GPFSEL0 , 13);
  CLRBIT(GPFSEL0 , 12);
  struct PageInfo constPage;
  struct PageInfo instrPage;
  struct PageInfo instrs[1024];
  setupDMA(constPage,instrPage,instrs);

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
  std::atomic <bool> tone_thread_ready;
  tone_thread_ready=false;
  std::thread tone_thread(tone_main,
    std::ref(terminate_tone_thread),
    std::ref(self_cal),
    std::ref(ppm_init),
    std::ref(tone_freq),
    instrs,
    std::ref(constPage),
    std::ref(tone_thread_ready)
  );
  while (!tone_thread_ready) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

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
    std::ref(am_thread_busy),
    ditdit,
    test_tone
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

  // In ditdit or test-tone mode, can only exit using ctrl-c.
  while (ditdit||test_tone) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
  std::cout << std::endl;

  // Terminate subthreads
  terminate_am_thread=true;
  terminate_tone_thread=true;
  if (am_thread.joinable()) {
    am_thread.join();
  }
  if (tone_thread.joinable()) {
    tone_thread.join();
  }

  return 0;
}

