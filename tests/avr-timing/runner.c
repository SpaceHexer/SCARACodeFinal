// Fuehrt die echte Mega-ELF mit simavr aus. Kein Host-Ersatz fuer MotorController.
// Abhaengigkeit: libsimavr-dev. Aufruf: ./runner firmware.elf
// SETREF setzt in der Simulation eine bekannte Ausgangsstellung; kein Homing
// und keine echte Mechanik werden hier behauptet oder simuliert.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sim_avr.h>
#include <sim_elf.h>
#include <sim_interrupts.h>
#include <sim_io.h>
#include <avr_uart.h>

struct Scenario {
  const char *command, *expected;
};
static const struct Scenario scenarios[] = {
    {"SETREF ALL 0 0 0 0\n",                       "OK SETREF"         },
    {"MOVEJ 1 1 1 0 2\n",                          "DONE MOVE"         },
    {"MOVEJ 90 90 90 40 4\n",                      "DONE MOVE"         },
    {"MOVEJ 30 40 -45 20 3\n",                     "DONE MOVE"         },
    {"MOVEJ 0 0 0 0 3\n",                          "DONE MOVE"         },
    {"BEGIN\n",                                    "OK"                },
    {"SEG 10 30 -30 10 3 V1 2 0 0 0 A1 1 0 0 0\n", "OK"                },
    {"SEG 20 30 -30 10 3 V0 2 0 0 0 A0 1 0 0 0\n", "OK"                },
    {"RUN\n",                                      "DONE MOVE"         },
    {"STOP\n",                                     "STOPPED; RESETSTOP"},
    {"SETREF ALL 0 0 0 0\n",                       "ERR STOPPED"       },
    {"RESETSTOP\n",                                "OK"                },
    {"SETREF J1 5\n",                              "OK SETREF"         },
    {"SETREF ALL 0 0 0 0\n",                       "OK SETREF"         },
};
static avr_t *cpu;
static uint64_t entered, maxCycles, entries;
static unsigned scenario, lineLength, doneCount;
static int matched, failed;
static char line[512];

static void output(struct avr_irq_t *irq, uint32_t value, void *param) {
  (void)irq;
  (void)param;
  char c = (char)(value & 255);
  if (c == '\r')
    return;
  putchar(c);
  if (c != '\n') {
    if (lineLength + 1 < sizeof(line))
      line[lineLength++] = c;
    else
      failed = 1;
    return;
  }
  line[lineLength] = 0;
  const char *expected = scenarios[scenario].expected;
  if (!strncmp(line, expected, strlen(expected)))
    matched = 1;
  else if (!strncmp(line, "ERR ", 4) || !strncmp(line, "STOPPED", 7))
    failed = 1;
  if (!strcmp(line, "DONE MOVE"))
    ++doneCount;
  lineLength = 0;
}

static void timing(struct avr_irq_t *irq, uint32_t value, void *param) {
  (void)irq;
  (void)param;
  if (value) {
    entered = cpu->cycle;
    ++entries;
  } else if (cpu->cycle - entered > maxCycles)
    maxCycles = cpu->cycle - entered;
}

int main(int argc, char **argv) {
  elf_firmware_t fw = {0};
  if (argc != 2 || elf_read_firmware(argv[1], &fw))
    return 1;
  cpu = avr_make_mcu_by_name("atmega2560");
  if (!cpu || avr_init(cpu))
    return 1;
  cpu->frequency = 16000000;
  avr_load_firmware(cpu, &fw);
  uint32_t flags = 0;
  avr_ioctl(cpu, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
  avr_irq_t *input = avr_io_getirq(cpu, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
  avr_irq_register_notify(avr_io_getirq(cpu, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), output, NULL);
  avr_irq_register_notify(avr_get_interrupt_irq(cpu, 32) + AVR_INT_IRQ_RUNNING, timing, NULL);
  const unsigned count = sizeof(scenarios) / sizeof(scenarios[0]);
  const char *sending = scenarios[0].command;
  uint64_t nextByte = 3200000, lastCommand = 3200000, nextTraffic = 11200000;
  // Maximal 40 s simulierte Zeit, jedes Szenario maximal 8 s.
  while (!failed && cpu->cycle < 640000000) {
    if (*sending && cpu->cycle >= nextByte) {
      avr_raise_irq(input, (uint8_t)*sending++);
      nextByte = cpu->cycle + 2400;  // 150 us pro Zeichen, langsamer als 115200 Baud.
    }
    if (matched) {
      if (++scenario == count)
        break;
      matched = 0;
      sending = scenarios[scenario].command;
      lastCommand = cpu->cycle;
      nextByte = cpu->cycle + 160000;  // Antwort vollstaendig abwarten.
      nextTraffic = cpu->cycle + 8000000;
    } else if (!*sending && !strcmp(scenarios[scenario].expected, "DONE MOVE") && cpu->cycle >= nextTraffic) {
      // Serial-RX/TX-Interrupts gleichzeitig mit der Motor-ISR belasten.
      sending = "PING\nSTATUS\n";
      nextByte = cpu->cycle;
      nextTraffic = cpu->cycle + 8000000;
    }
    if (cpu->cycle >= lastCommand && cpu->cycle - lastCommand > 128000000)
      failed = 1;
    int state = avr_run(cpu);
    if (state == cpu_Crashed || state == cpu_Done)
      failed = 1;
  }
  printf("Timer3: entries=%llu max_ISR_cycles=%llu max_us=%.3f\n", (unsigned long long)entries, (unsigned long long)maxCycles, maxCycles / 16.0);
  // Unter dem 100-us-Takt bleiben; echte Hardware muss separat gemessen werden.
  if (failed || scenario != count || doneCount != 5 || maxCycles >= 1600) {
    fprintf(stderr, "FAIL AVR timing scenario=%u/%u done=%u\n", scenario, count, doneCount);
    return 1;
  }
  puts("PASS AVR execution: manual reference, five joint moves, directions, chain, Serial load, STOP");
  return 0;
}
