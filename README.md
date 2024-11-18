Install pico SDK, install vs-code extension

To compile .pio file:
~/.pico-sdk/tools/2.0.0/pioasm/pioasm pwm_waveform_output.pio pwm_waveform_output.pio.h

CMakeLists.txt does not currently have automatic building for the PIO file, but if you need it there must be a way to add it.

To program -- worked on my machine, your mialage may vary:
1) Unplug raspberry pi.
2) Hold bootsel button on the pico
3) Plug in while holding
4) Release
5) Press "run" on the extension.

