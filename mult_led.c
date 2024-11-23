#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pwm_waveform_output.pio.h"

#define MAX_CHANNELS 2

typedef struct {
    uint gpio_pin;
    uint32_t waveform_length;
    uint32_t *waveform_data;
    uint dma_chan;
} ChannelConfig;

ChannelConfig channels[MAX_CHANNELS] = {
    { .gpio_pin = 27 }, // LED1
    { .gpio_pin = 16 }  // LED2
};

uint32_t desired_frequencies[MAX_CHANNELS] = {
    200000, // Not sure how high this can go. I think 200kHz is about the max.
    200000 
};

void generate_synchronized_pwm_waveform(ChannelConfig *channel, uint32_t system_clock, uint32_t desired_frequency, bool is_led2) {
    uint32_t cycles_per_period = system_clock / desired_frequency;
    uint32_t overhead_cycles_per_step = 4; 
    uint32_t total_overhead_cycles = 4 * overhead_cycles_per_step;

    if (cycles_per_period <= total_overhead_cycles) {
        printf("Total cycles per period is too small for channel %d.\n", channel->gpio_pin);
        return;
    }

    uint32_t total_duration_cycles = cycles_per_period - total_overhead_cycles;

    uint8_t state_percentages[] = {40, 10, 40, 10}; 
    uint32_t durations[4];

    for (int i = 0; i < 4; i++) {
        durations[i] = (total_duration_cycles * state_percentages[i]) / 100;
        if (durations[i] == 0) durations[i] = 1;
    }

    uint32_t sum_durations = durations[0] + durations[1] + durations[2] + durations[3];
    if (sum_durations < total_duration_cycles) {
        durations[3] += (total_duration_cycles - sum_durations);
    } else if (sum_durations > total_duration_cycles) {
        durations[3] -= (sum_durations - total_duration_cycles);
    }

    channel->waveform_length = 8; // 4 steps * 2 (duration + state)
    channel->waveform_data = (uint32_t *)malloc(channel->waveform_length * sizeof(uint32_t));
    if (!channel->waveform_data) {
        printf("Failed to allocate memory for channel %d\n", channel->gpio_pin);
        return;
    }

    if (is_led2) {
        channel->waveform_data[0] = durations[0]; // OFF
        channel->waveform_data[1] = 0;          // OFF
        channel->waveform_data[2] = durations[1]; // OFF
        channel->waveform_data[3] = 0;          // OFF
        channel->waveform_data[4] = durations[2]; // ON
        channel->waveform_data[5] = 1;          // ON
        channel->waveform_data[6] = durations[3]; // OFF
        channel->waveform_data[7] = 0;          // OFF
    } else {
        channel->waveform_data[0] = durations[0]; // ON
        channel->waveform_data[1] = 1;          // ON
        channel->waveform_data[2] = durations[1]; // OFF
        channel->waveform_data[3] = 0;          // OFF
        channel->waveform_data[4] = durations[2]; // OFF
        channel->waveform_data[5] = 0;          // OFF
        channel->waveform_data[6] = durations[3]; // OFF
        channel->waveform_data[7] = 0;          // OFF
    }

    // printf("Channel %d Waveform:\n", channel->gpio_pin);
    // for (int i = 0; i < 4; i++) {
    //     printf("  Step %d: Duration %u cycles, Pin State %u\n", i + 1, channel->waveform_data[i * 2], channel->waveform_data[i * 2 + 1]);
    // }
}

void pwm_waveform_output_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = pwm_waveform_output_program_get_default_config(offset);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    sm_config_set_out_pins(&c, pin, 1);

    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, offset, &c);
}

void configure_dma_for_channel(PIO pio, uint sm, ChannelConfig *channel) {
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
        &pio->txf[sm],            
        channel->waveform_data,
        channel->waveform_length,
        false
    );

    dma_channel_set_irq0_enabled(dma_chan, true);
}


void dma_handler() {
    uint32_t dma_intr = dma_hw->ints0;
    dma_hw->ints0 = dma_intr;

    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (dma_intr & (1u << channels[i].dma_chan)) {
            dma_channel_set_read_addr(channels[i].dma_chan, channels[i].waveform_data, false);
            dma_channel_set_trans_count(channels[i].dma_chan, channels[i].waveform_length, true);
        }
    }
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(250000, true);
    uint32_t system_clock = clock_get_hz(clk_sys);
    // printf("System clock: %u Hz\n", system_clock);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &pwm_waveform_output_program);

    for (int i = 0; i < MAX_CHANNELS; i++) {
        uint sm = i; 
        bool is_led2 = (i % 2 == 1);

        generate_synchronized_pwm_waveform(&channels[i], system_clock, desired_frequencies[i], is_led2);
        pwm_waveform_output_program_init(pio, sm, offset, channels[i].gpio_pin);
        configure_dma_for_channel(pio, sm, &channels[i]);
    }

    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    for (int i = 0; i < MAX_CHANNELS; i++) {
        pio_sm_set_enabled(pio, i, false);
        pio_sm_clear_fifos(pio, i);
        pio_sm_restart(pio, i);
    }

    for (int i = 0; i < MAX_CHANNELS; i++) {
        pio_sm_set_enabled(pio, i, true);
    }

    uint32_t dma_channel_mask = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        dma_channel_mask |= (1u << channels[i].dma_chan);
    }
    dma_start_channel_mask(dma_channel_mask);

    while (true) {
        tight_loop_contents();
    }

    return 0;
}
