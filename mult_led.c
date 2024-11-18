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
#define MAX_WAVEFORM_LENGTH 1024 

typedef struct {
    uint gpio_pin;
    uint32_t waveform_length;
    uint32_t *waveform_data;
    uint dma_chan;
    uint32_t pwm_period;
    uint32_t sample_count;
} ChannelConfig;

ChannelConfig channels[MAX_CHANNELS] = {
    { .gpio_pin = 27 },
    { .gpio_pin = 16 }
};

uint32_t desired_frequencies[MAX_CHANNELS] = {7000, 20000};

void generate_pwm_waveform(ChannelConfig *channel, uint32_t system_clock, uint32_t desired_frequency) {
    uint32_t pwm_frequency = 100000;
    channel->pwm_period = system_clock / pwm_frequency;

    channel->sample_count = pwm_frequency / desired_frequency;

    if (channel->sample_count > (MAX_WAVEFORM_LENGTH / 2)) {
        printf("Sample count too large for channel %d, reducing to %d\n", channel->gpio_pin, MAX_WAVEFORM_LENGTH / 2);
        channel->sample_count = MAX_WAVEFORM_LENGTH / 2;
    }

    channel->waveform_data = (uint32_t *)malloc(channel->sample_count * 2 * sizeof(uint32_t));
    if (!channel->waveform_data) {
        printf("Failed to allocate memory for channel %d\n", channel->gpio_pin);
        return;
    }

    for (uint32_t i = 0; i < channel->sample_count; i++) {
        float angle = 2.0f * M_PI * i / channel->sample_count;
        float sine_value = (sinf(angle) + 1.0f) / 2.0f;  // Normalize to 0 - 1

        uint32_t high_time = (uint32_t)(sine_value * channel->pwm_period);

        if (high_time < 1) high_time = 1;
        if (high_time >= channel->pwm_period) high_time = channel->pwm_period - 1;

        uint32_t low_time = channel->pwm_period - high_time;

        if (low_time < 1) low_time = 1;

        channel->waveform_data[i * 2] = high_time;
        channel->waveform_data[i * 2 + 1] = low_time;
    }

    channel->waveform_length = channel->sample_count * 2; 
}

void configure_dma_for_channel(PIO pio, uint sm, ChannelConfig *channel) {
    uint dma_chan = dma_claim_unused_channel(true);
    channel->dma_chan = dma_chan;

    dma_channel_config c = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    // Use the PIO's DREQ signal to pace the transfers
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &c,
        &pio->txf[sm],                // Write address (PIO TX FIFO) instead of compare pin from before with the .cc thing
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
            dma_channel_set_read_addr(channels[i].dma_chan, channels[i].waveform_data, true);
        }
    }
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(250000, true);
    uint32_t system_clock = clock_get_hz(clk_sys);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &pwm_waveform_output_program);


    for (int i = 0; i < MAX_CHANNELS; i++) {
        uint sm = i;  // One state machine per channel

        generate_pwm_waveform(&channels[i], system_clock, desired_frequencies[i]);
        pwm_waveform_output_program_init(pio, sm, offset, channels[i].gpio_pin);

        configure_dma_for_channel(pio, sm, &channels[i]);
    }

    irq_add_shared_handler(DMA_IRQ_0, dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    for (int i = 0; i < MAX_CHANNELS; i++) {
        dma_start_channel_mask(1u << channels[i].dma_chan);
    }

    while (true) {
        tight_loop_contents();
    }

    return 0;
}
