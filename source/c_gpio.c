/*
Copyright (c) 2012-2015 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include "c_gpio.h"

#define BCM2708_PERI_BASE_DEFAULT   0x20000000
#define BCM2709_PERI_BASE_DEFAULT   0x3f000000
#define GPIO_BASE_OFFSET            0x200000
#define FSEL_OFFSET                 0   // 0x0000
#define SET_OFFSET                  7   // 0x001c / 4
#define CLR_OFFSET                  10  // 0x0028 / 4
#define PINLEVEL_OFFSET             13  // 0x0034 / 4
#define EVENT_DETECT_OFFSET         16  // 0x0040 / 4
#define RISING_ED_OFFSET            19  // 0x004c / 4
#define FALLING_ED_OFFSET           22  // 0x0058 / 4
#define HIGH_DETECT_OFFSET          25  // 0x0064 / 4
#define LOW_DETECT_OFFSET           28  // 0x0070 / 4
#define PULLUPDN_OFFSET             37  // 0x0094 / 4
#define PULLUPDNCLK_OFFSET          38  // 0x0098 / 4

#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)

//
// For Pine A64/A64+ Board
//
#define PINEA64_GPIO_MASK	(0xFFFFFF80)
#define SUNXI_GPIO_BASE		0x01C20000
#define SUNXI_GPIO_REG_OFFSET   0x800
#define PINEA64_GPIO_BASE	(SUNXI_GPIO_BASE + SUNXI_GPIO_REG_OFFSET)
#define SUNXI_CFG_OFFSET	0x00
#define SUNXI_DATA_OFFSET	0x10
#define SUNXI_PUD_OFFSET	0x1C
#define SUNXI_BANK_SIZE		0x24

#define MAP_SIZE        (4096*2)
#define MAP_MASK        (MAP_SIZE - 1)

typedef struct sunxi_gpio {
    unsigned int CFG[4];
    unsigned int DAT;
    unsigned int DRV[2];
    unsigned int PULL[2];
} sunxi_gpio_t;

/* gpio interrupt control */
typedef struct sunxi_gpio_int {
    unsigned int CFG[3];
    unsigned int CTL;
    unsigned int STA;
    unsigned int DEB;
} sunxi_gpio_int_t;

typedef struct sunxi_gpio_reg {
    struct sunxi_gpio gpio_bank[9];
    unsigned char res[0xbc];
    struct sunxi_gpio_int gpio_int;
} sunxi_gpio_reg_t;

#define GPIO_BANK(pin)  ((pin) >> 5)
#define GPIO_NUM(pin)   ((pin) & 0x1F)

#define GPIO_CFG_INDEX(pin)     (((pin) & 0x1F) >> 3)
#define GPIO_CFG_OFFSET(pin)    ((((pin) & 0x1F) & 0x7) << 2)

#define GPIO_PUL_INDEX(pin)     (((pin) & 0x1F )>> 4) 
#define GPIO_PUL_OFFSET(pin)    (((pin) & 0x0F) << 1)

extern int pinea64_found;
static volatile uint32_t *pio_map;
// end of Pine A64/A64+

static volatile uint32_t *gpio_map;

void short_wait(void)
{
    int i;

    for (i=0; i<150; i++) {    // wait 150 cycles
        asm volatile("nop");
    }
}

int setup(void)
{
    int mem_fd;
    uint8_t *gpio_mem;
    uint32_t peri_base;
    uint32_t gpio_base;
    unsigned char buf[4];
    FILE *fp;
    char buffer[1024];
    char hardware[1024];
    int found = 0;

  if ( !pinea64_found )  {
    // try /dev/gpiomem first - this does not require root privs
    if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) > 0)
    {
        gpio_map = (uint32_t *)mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);
        if ((uint32_t)gpio_map < 0) {
            return SETUP_MMAP_FAIL;
        } else {
            return SETUP_OK;
        }
    }

    // revert to /dev/mem method - requires root

    // determine peri_base
    if ((fp = fopen("/proc/device-tree/soc/ranges", "rb")) != NULL) {
        // get peri base from device tree
        fseek(fp, 4, SEEK_SET);
        if (fread(buf, 1, sizeof buf, fp) == sizeof buf) {
            peri_base = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
        }
        fclose(fp);
    } else {
        // guess peri base based on /proc/cpuinfo hardware field
        if ((fp = fopen("/proc/cpuinfo", "r")) == NULL)
            return SETUP_CPUINFO_FAIL;

        while(!feof(fp) && !found) {
            fgets(buffer, sizeof(buffer), fp);
            sscanf(buffer, "Hardware	: %s", hardware);
            if (strcmp(hardware, "BCM2708") == 0 || strcmp(hardware, "BCM2835") == 0) {
                // pi 1 hardware
                peri_base = BCM2708_PERI_BASE_DEFAULT;
                found = 1;
            } else if (strcmp(hardware, "BCM2709") == 0 || strcmp(hardware, "BCM2836") == 0) {
                // pi 2 hardware
                peri_base = BCM2709_PERI_BASE_DEFAULT;
                found = 1;
            }
        }
        fclose(fp);
        if (!found)
            return SETUP_NOT_RPI_FAIL;
    }

    gpio_base = peri_base + GPIO_BASE_OFFSET;
  }

    // mmap the GPIO memory registers
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
        return SETUP_DEVMEM_FAIL;

    if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)
        return SETUP_MALLOC_FAIL;

    if ((uint32_t)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);

  if ( pinea64_found )  {
    gpio_map = (uint32_t *)mmap( (caddr_t)gpio_mem, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, SUNXI_GPIO_BASE);
    pio_map = gpio_map + (SUNXI_GPIO_REG_OFFSET>>2);
  } else {
    gpio_map = (uint32_t *)mmap( (void *)gpio_mem, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, gpio_base);
  }

    if ((uint32_t)gpio_map < 0)
        return SETUP_MMAP_FAIL;

    return SETUP_OK;
}

void clear_event_detect(int gpio)
{
  if ( !pinea64_found )  {
    int offset = EVENT_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    *(gpio_map+offset) |= (1 << shift);
    short_wait();
    *(gpio_map+offset) = 0;
  }
}

int eventdetected(int gpio)
{
  if ( !pinea64_found )  {
    int offset, value, bit;

    offset = EVENT_DETECT_OFFSET + (gpio/32);
    bit = (1 << (gpio%32));
    value = *(gpio_map+offset) & bit;
    if (value)
        clear_event_detect(gpio);
    return value;
  }
}

void set_rising_event(int gpio, int enable)
{
  if ( !pinea64_found )  {
    int offset = RISING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable)
        *(gpio_map+offset) |= 1 << shift;
    else
        *(gpio_map+offset) &= ~(1 << shift);
    clear_event_detect(gpio);
  }
}

void set_falling_event(int gpio, int enable)
{
  if ( !pinea64_found )  {
    int offset = FALLING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable) {
        *(gpio_map+offset) |= (1 << shift);
        *(gpio_map+offset) = (1 << shift);
    } else {
        *(gpio_map+offset) &= ~(1 << shift);
    }
    clear_event_detect(gpio);
  }
}

void set_high_event(int gpio, int enable)
{
  if ( !pinea64_found )  {
    int offset = HIGH_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable)
        *(gpio_map+offset) |= (1 << shift);
    else
        *(gpio_map+offset) &= ~(1 << shift);
    clear_event_detect(gpio);
  }
}

void set_low_event(int gpio, int enable)
{
  if ( !pinea64_found )  {
    int offset = LOW_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable)
        *(gpio_map+offset) |= 1 << shift;
    else
        *(gpio_map+offset) &= ~(1 << shift);
    clear_event_detect(gpio);
  }
}

uint32_t sunxi_readl(volatile uint32_t *addr)
{
    uint32_t val = 0;
    uint32_t mmap_base = (uint32_t)addr & (~MAP_MASK);
    uint32_t mmap_seek = ((uint32_t)addr - mmap_base) >> 2;
    val = *(gpio_map + mmap_seek);
    return val;
}   

void sunxi_writel(volatile uint32_t *addr, uint32_t val)
{
    uint32_t mmap_base = (uint32_t)addr & (~MAP_MASK);
    uint32_t mmap_seek =( (uint32_t)addr - mmap_base) >> 2;
    *(gpio_map + mmap_seek) = val;
}

void set_pullupdn(int gpio, int pud)
{
  if ( pinea64_found )  {
    uint32_t regval = 0;
    int bank = GPIO_BANK(gpio); //gpio >> 5
    int index = GPIO_PUL_INDEX(gpio); // (gpio & 0x1f) >> 4
    int offset = GPIO_PUL_OFFSET(gpio); // (gpio) & 0x0F) << 1

    sunxi_gpio_t *pio = &((sunxi_gpio_reg_t *) pio_map)->gpio_bank[bank];

    regval = *(&pio->PULL[0] + index);
    regval &= ~(3 << offset);
    regval |= pud << offset;
    *(&pio->PULL[0] + index) = regval;
  } else {
    int clk_offset = PULLUPDNCLK_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (pud == PUD_DOWN)
        *(gpio_map+PULLUPDN_OFFSET) = (*(gpio_map+PULLUPDN_OFFSET) & ~3) | PUD_DOWN;
    else if (pud == PUD_UP)
        *(gpio_map+PULLUPDN_OFFSET) = (*(gpio_map+PULLUPDN_OFFSET) & ~3) | PUD_UP;
    else  // pud == PUD_OFF
        *(gpio_map+PULLUPDN_OFFSET) &= ~3;

    short_wait();
    *(gpio_map+clk_offset) = 1 << shift;
    short_wait();
    *(gpio_map+PULLUPDN_OFFSET) &= ~3;
    *(gpio_map+clk_offset) = 0;
  }
}

void setup_gpio(int gpio, int direction, int pud)
{
  if ( pinea64_found )  {
    uint32_t regval = 0;
    int bank = GPIO_BANK(gpio); //gpio >> 5
    int index = GPIO_CFG_INDEX(gpio); // (gpio & 0x1F) >> 3
    int offset = GPIO_CFG_OFFSET(gpio); // ((gpio & 0x1F) & 0x7) << 2

    sunxi_gpio_t *pio = &((sunxi_gpio_reg_t *) pio_map)->gpio_bank[bank];

    set_pullupdn(gpio, pud);

    regval = *(&pio->CFG[0] + index);
    regval &= ~(0x7 << offset); // 0xf?
    if (INPUT == direction) {
        *(&pio->CFG[0] + index) = regval;
    } else if (OUTPUT == direction) {
        regval |=  (1 << offset);
        *(&pio->CFG[0] + index) = regval;
    } else {
        printf("line:%dgpio number error\n",__LINE__);
    }
  } else {
    int offset = FSEL_OFFSET + (gpio/10);
    int shift = (gpio%10)*3;

    set_pullupdn(gpio, pud);
    if (direction == OUTPUT)
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (1<<shift);
    else  // direction == INPUT
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift));
  }
}

// Contribution by Eric Ptak <trouch@trouch.com>
int gpio_function(int gpio)
{
  if ( pinea64_found )  {
    uint32_t regval = 0;
    int bank = GPIO_BANK(gpio); //gpio >> 5
    int index = GPIO_CFG_INDEX(gpio); // (gpio & 0x1F) >> 3
    int offset = GPIO_CFG_OFFSET(gpio); // ((gpio & 0x1F) & 0x7) << 2

    sunxi_gpio_t *pio = &((sunxi_gpio_reg_t *) pio_map)->gpio_bank[bank];

    regval = *(&pio->CFG[0] + index);
    regval >>= offset;
    regval &= 7;
    return regval; // 0=input, 1=output, 4=alt0
  } else {
    int offset = FSEL_OFFSET + (gpio/10);
    int shift = (gpio%10)*3;
    int value = *(gpio_map+offset);
    value >>= shift;
    value &= 7;
    return value; // 0=input, 1=output, 4=alt0
  }
}

void output_gpio(int gpio, int value)
{
  if ( pinea64_found )  {
    int bank = GPIO_BANK(gpio); //gpio >> 5
    int num = GPIO_NUM(gpio); // gpio & 0x1F

    sunxi_gpio_t *pio = &((sunxi_gpio_reg_t *) pio_map)->gpio_bank[bank];

    if (value == 0)
        *(&pio->DAT) &= ~(1 << num);
    else
        *(&pio->DAT) |= (1 << num);
  } else {
    int offset, shift;

    if (value) // value == HIGH
        offset = SET_OFFSET + (gpio/32);
    else       // value == LOW
       offset = CLR_OFFSET + (gpio/32);

    shift = (gpio%32);

    *(gpio_map+offset) = 1 << shift;
  }
}

int input_gpio(int gpio)
{
  if ( pinea64_found )  {
    uint32_t regval = 0;
    int bank = GPIO_BANK(gpio); //gpio >> 5
    int num = GPIO_NUM(gpio); // gpio & 0x1F

    sunxi_gpio_t *pio = &((sunxi_gpio_reg_t *) pio_map)->gpio_bank[bank];

    regval = *(&pio->DAT);
    regval = regval >> num;
    regval &= 1;
    return regval;
  } else {
   int offset, value, mask;

   offset = PINLEVEL_OFFSET + (gpio/32);
   mask = (1 << gpio%32);
   value = *(gpio_map+offset) & mask;
   return value;
  }
}

void cleanup(void)
{
    munmap((void *)gpio_map, BLOCK_SIZE);
}
