#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pwm_waveform_output.pio.h"

// Note to Pavlos: At some point you may run out of memory. At that point, probably
// the best thing to do is to use the flash memory to store the waveform data.
// The lazy way out is to migrate to Raspberry Pi Pico 2 which has more memory.

#define MAX_CHANNELS 3
#define MAX_WAVEFORM_LENGTH 1024
#define PHASE_WRAP (1ULL << 32)

#define SINE_TABLE_SIZE 1024
float sine_table[SINE_TABLE_SIZE];

void init_sine_table()
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        sine_table[i] = (sinf(2.0f * M_PI * i / SINE_TABLE_SIZE) + 1.0f) / 2.0f;
    }
}

typedef struct
{
    uint gpio_pin;
    uint32_t waveform_length;
    uint32_t *waveform_data;
    uint dma_chan;
    uint32_t pwm_period;
    uint32_t sample_count;
} ChannelConfig;

ChannelConfig channels[MAX_CHANNELS] = {
    {.gpio_pin = 27},
    {.gpio_pin = 16},
    {.gpio_pin = 15}};

uint32_t desired_frequencies[MAX_CHANNELS] = {
    19980,
    20000,
    20020};

// feat.... double!!! I think is overkill but it's fine
void generate_pwm_waveform(ChannelConfig *channel, uint32_t system_clock, uint32_t desired_frequency)
{
    double total_cycles_per_waveform = (double)system_clock / (double)desired_frequency;
    uint32_t overhead_cycles_per_period = 8; // not sure why this is 8, intuitively should be 5 from the .pio file but I see 8 instructions so... it's fine
    uint32_t min_sample_count = 8;
    uint32_t max_sample_count = 64;
    uint32_t best_sample_count = 0;
    uint32_t best_pwm_period = 0;
    double min_frequency_error = 1e9;

    // also this is a horrendous way to find the best sample count and pwm period but
    // once more, it's fine
    for (uint32_t sample_count = min_sample_count; sample_count <= max_sample_count; sample_count++)
    {
        double pwm_period_f = (total_cycles_per_waveform / (double)sample_count) - (double)overhead_cycles_per_period;
        // printf("Sample count: %u, Calculated pwm_period_f: %f\n", sample_count, pwm_period_f);

        if (pwm_period_f <= 0.0)
        {
            continue;
        }
        uint32_t pwm_period = (uint32_t)(pwm_period_f + 0.5);

        double total_cycles = (double)sample_count * ((double)pwm_period + (double)overhead_cycles_per_period);
        double actual_frequency = (double)system_clock / total_cycles;
        double frequency_error = fabs(desired_frequency - actual_frequency);

        if (frequency_error < min_frequency_error)
        {
            min_frequency_error = frequency_error;
            best_sample_count = sample_count;
            best_pwm_period = pwm_period;
        }

        if (frequency_error < 0.001)
        {
            break;
        }
    }

    if (best_pwm_period == 0 || best_sample_count == 0)
    {
        printf("No pwm_period and sample_count for desired frequency %u Hz\n", desired_frequency);
        return;
    }

    channel->sample_count = best_sample_count;
    channel->pwm_period = best_pwm_period;

    channel->waveform_data = (uint32_t *)malloc(channel->sample_count * 2 * sizeof(uint32_t));
    if (!channel->waveform_data)
    {
        printf("Failed to allocate memory for channel %u\n", channel->gpio_pin);
        return;
    }

    for (uint32_t i = 0; i < channel->sample_count; i++)
    {
        double phase = (double)i / (double)channel->sample_count * 2.0 * M_PI;
        double sine_value = (sin(phase) + 1.0) / 2.0;

        uint32_t high_time = (uint32_t)(sine_value * channel->pwm_period + 0.5);

        if (high_time < 1)
            high_time = 1;
        if (high_time >= channel->pwm_period)
            high_time = channel->pwm_period - 1;

        uint32_t low_time = channel->pwm_period - high_time;

        if (low_time < 1)
            low_time = 1;

        channel->waveform_data[i * 2] = high_time;
        channel->waveform_data[i * 2 + 1] = low_time;
    }

    channel->waveform_length = channel->sample_count * 2;

    double total_cycles = (double)channel->sample_count * ((double)channel->pwm_period + (double)overhead_cycles_per_period);
    double actual_frequency = (double)system_clock / total_cycles;

    printf("Channel %u: Desired Frequency = %u Hz, Actual Frequency = %.6f Hz, Sample Count = %u, PWM Period = %u\n",
           channel->gpio_pin, desired_frequency, actual_frequency, channel->sample_count, channel->pwm_period);
}

void configure_dma_for_channel(PIO pio, uint sm, ChannelConfig *channel)
{
    uint dma_chan = dma_claim_unused_channel(true);
    channel->dma_chan = dma_chan;

    dma_channel_config c = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &c,
        &pio->txf[sm], // Write address (PIO TX FIFO) instead of compare pin from before with the .cc thing
        channel->waveform_data,
        channel->waveform_length,
        false);

    dma_channel_set_irq0_enabled(dma_chan, true);
}

void dma_handler()
{
    uint32_t dma_intr = dma_hw->ints0;
    dma_hw->ints0 = dma_intr;

    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (dma_intr & (1u << channels[i].dma_chan))
        {
            dma_channel_set_read_addr(channels[i].dma_chan, channels[i].waveform_data, true);
        }
    }
}

int main()
{
    stdio_init_all();
    set_sys_clock_khz(250000, true);
    uint32_t system_clock = clock_get_hz(clk_sys);

    sleep_ms(1000);

    printf("System Clock: %u Hz\n", system_clock);

    init_sine_table();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &pwm_waveform_output_program);

    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        uint sm = i; // One state machine per channel

        generate_pwm_waveform(&channels[i], system_clock, desired_frequencies[i]);
        pwm_waveform_output_program_init(pio, sm, offset, channels[i].gpio_pin);

        configure_dma_for_channel(pio, sm, &channels[i]);
    }

    irq_add_shared_handler(DMA_IRQ_0, dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        dma_start_channel_mask(1u << channels[i].dma_chan);
    }

    while (true)
    {
        tight_loop_contents();
    }

    return 0;
}
