# LED Status

## Reality Check

The Stratosonde flight board has **one diagnostic LED on PA0** and the firmware
uses it for exactly one thing: a boot blink on the bench. There is no LED
module, no pattern engine, and no error-code blink vocabulary — an earlier
version of this document described one, but none of it exists in the tree.

Status visibility on the bench is **RTT logging** (`sonde_log.h`, SEGGER RTT
terminal 0, 4 KB non-blocking buffer), not the LED.

## The One Use: Boot Blink (Commissioning Only)

`leds_boot_seq()` in `main.c`: two 500 ms blinks at boot — a bench/commissioning
"liveness" signal. **Dark in flight (R09/DDR-0002)**: the function returns
immediately unless `MissionState_IsCommissioning()` — a flight unit rebooting
at altitude must not spend power or light on a LED nobody can see.

```
boot (commissioning)  →  ON 500 ms · OFF 500 ms · ON 500 ms · OFF 500 ms
boot (flight)         →  nothing
all other times       →  OFF
```

The only other PA0 write in the tree is a debug wiggle in `sht31.c`
(bench bring-up aid). `LED_PERIOD_TIME` in `lora_app.c` is an unused leftover
define.

## Inputs, Not Outputs

The board's one human input is the **SOS button on PA3** (`SOS_Button_Pin`) —
see the System module for its role.

## Power Rationale

A LED on a balloon is dead weight: no observer, real current. That is why the
blink is commissioning-gated rather than dimmed — flight units simply never
light it (DDR-0002 lifecycle: commissioning is a ground phase).