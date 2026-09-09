# Bumblebee firmware (AIMotionLab fork)

This is the firmware that runs *on board* the lab's Bumblebee drones. A Bumblebee is a custom-built quadrotor: the flight controller board is a [Crazyflie Bolt](https://www.bitcraze.io/products/crazyflie-bolt-1-1/), but everything around it is ours — a large frame (0.125 m arm length, versus 0.046 m on a stock Crazyflie), brushless motors driven through external ESCs, a multi-cell LiPo pack, and a hook/payload rig hanging under the drone. It is therefore a close cousin of the Crazyflie 2.1+ in software, and a very different machine in the air: roughly 20× heavier, with its own motor model, its own gains, and controllers that stock Crazyflies do not have.

Note that the drone actually has two microcontrollers on board: an STM32, responsible for flight control and application logic, and an nRF51, responsible for radio and power management. This firmware is meant to be flashed on the former. For the latter, visit [crazyflie2-nrf-firmware](https://github.com/bitcraze/crazyflie2-nrf-firmware).
This repo is forked from [skybrush-io/crazyflie-firmware](https://github.com/skybrush-io/crazyflie-firmware), which is in turn a fork of [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware).

## 1. Building and flashing

### 1.1 Prerequisites

The target architecture is an ARM Cortex-M4 core STM32 microcontroller, so when compiled on your PC, the firmware will be cross-compiled (the compiler runs on your PC but emits ARM machine code). That compiler is `arm-none-eabi-gcc`, as in "gcc compiler, for an ARM processor, without any vendor specific customization, with Embedded Application Binary interface (as opposed to a Linux syscall layer, for example).

On Ubuntu/Debian:

```bash
sudo apt-get install make gcc-arm-none-eabi
```

Flashing happens over the radio, using the `cfloader` tool that ships with the [Crazyflie Python client](https://github.com/bitcraze/crazyflie-clients-python):

```bash
pip install cfclient
```

You also need a [Crazyradio](https://www.bitcraze.io/products/crazyradio-2-0/) USB dongle plugged in, with its udev rules installed, otherwise `cfloader` will not see the drone.

### 1.2 Getting the source

This repository uses git submodules. A submodule is a reference from this repository to one exact commit of *another* repository. Git stores only the URL and the commit hash, not the files, so a plain `git clone` leaves those directories empty, causing the build to fail. Clone with `--recursive` to pull them in as well:

```bash
git clone --recursive https://github.com/AIMotionLab-SZTAKI/bumblebee-firmware.git
```

If you have already cloned without `--recursive`, fix it from inside the repo:

```bash
git submodule init      # register the submodules listed in .gitmodules
git submodule update    # check out the pinned commit of each one
```

### 1.3 Choosing a variant

The firmware is not built once and for all: which hardware it is built for, and which positioning system it expects, are compile-time options. The ready-made combinations live in [app/conf/](app/conf/):

| Variant | Board | Positioning |
| --- | --- | --- |
| `bumblebee` | Bolt (our Bumblebee frame) | Marker deck (motion capture) |
| `stock-marker` | stock Crazyflie 2.x | Marker deck (motion capture) |
| `stock-lh` | stock Crazyflie 2.x | Lighthouse |
| `stock-uwb` | stock Crazyflie 2.x | Ultra-wideband (Loco) |
| `show-proto-v2-lh` | Bolt | Lighthouse |
| `show-proto-v2-uwb` | Bolt | Ultra-wideband (Loco) |

For our Bumblebee drones, use `bumblebee`; the other variants are inherited from upstream and are not flown here. Each variant file only holds what is *specific* to that variant. The settings shared by all of them (app layer enabled, LED ring enabled, Kalman filter as the default estimator, ...) are in [app/conf/common.cfg](app/conf/common.cfg). At build time the two get concatenated into a single file called `app/app-config`, which is what the build system actually reads.

[app/conf/bumblebee.cfg](app/conf/bumblebee.cfg) is short and worth reading in full — it is the one place that says "this binary is for a Bumblebee". It selects the Bolt platform, motion-capture positioning, the OneShot125 ESC protocol (the stock Crazyflie drives its motors with plain PWM; brushless ESCs need a pulse-width protocol instead), a dimmed LED ring (motors and LEDs together draw too much power), a smoother takeoff, and the **geometric controller as the default** rather than the PID controller used on stock Crazyflies.

### 1.4 Building

Everything is driven from the [app/](app/) directory, which contains the Skybrush application layer:

```bash
cd app
./compile bumblebee
```

[app/compile](app/compile) is a short shell script that, for the variant you name: concatenates `conf/common.cfg` + `conf/bumblebee.cfg` into `app-config`, cleans and builds the firmware, then copies the result under a variant-specific name. The output lands in the `app/` directory:

```
app/bolt-skybrush-bumblebee.bin
```

`bolt` stands for the Bolt-based variants, `cf2` would be the alternative — the script derives this from the `CONFIG_PLATFORM_*` line in the config. Optionally, `make menuconfig` in `app/` opens a terminal menu where you can browse and toggle every configuration option, if you need something that no `.cfg` file covers.

### 1.5 Flashing

1. Turn the drone off.
2. Hold the power button for about 3 seconds, to put the drone in bootloader mode, as indicated by two flashing LEDs.
3. From the `app/` directory, with the Crazyradio plugged in:

```bash
python3 -m cfloader flash bolt-skybrush-bumblebee.bin stm32-fw
```

This runs the `cfloader` module (which we got along with cfclient) as a script. This expects an action, which is `flash`, and `flash` requires an image to write, which is the raw binary we just built: `bolt-skybrush-bumblebee.bin`. `stm32-fw` denotes which target to write in `<target>-<type>` format: `stm32` (rather than `nrf51`) and firmware as opposed to the bootloader itself.



## 2. How the firmware works

### 2.1 It is a FreeRTOS application

The firmware runs on top of [FreeRTOS](https://www.freertos.org/), a small real-time operating system that is compiled into the binary as a library (it lives in the `vendor/FreeRTOS` submodule). FreeRTOS has no filesystem, no processes, no shell; what it gives us is *tasks* and ways for them to talk to each other.

A **task** is a plain C function that never returns, with its own stack, that the scheduler can suspend and resume. Every task has a priority, and the scheduler always runs the highest-priority task that is *ready*. The priorities are all collected in [src/config/config.h](src/config/config.h), which is a good place to get a feel for what runs on the drone:

```c
#define STABILIZER_TASK_PRI     5
#define SENSORS_TASK_PRI        4
#define SYSTEM_TASK_PRI         2
#define CRTP_TX_TASK_PRI        2
#define CRTP_RX_TASK_PRI        2
#define KALMAN_TASK_PRI         2
#define LOG_TASK_PRI            1
#define PARAM_TASK_PRI          1
```

Because the scheduler is fixed-priority and preemptive (`configUSE_PREEMPTION` is 1 in [src/config/FreeRTOSConfig.h](src/config/FreeRTOSConfig.h)), the highest-priority ready task always runs, and a task that never blocks will starve everything below it forever (FreeRTOS does time-slice, but only between tasks at the *same* priority). Therefore, to not starve lower priority tasks, each task in this firmware spends some time blocked (waiting on a queue, waiting on a semaphore, sleeping with `vTaskDelay()`). Should new tasks be added, this rule should be followed for them as well.

The two communication primitives you will see everywhere are **queues** (a thread-safe FIFO; `xQueueSend` / `xQueueReceive`) and **semaphores/mutexes** (`xSemaphoreTake` / `xSemaphoreGive`).

Tasks are not created with the usual `xTaskCreate()` (which is the usual case in FreeRTOS), but through the macros in [src/modules/interface/static_mem.h](src/modules/interface/static_mem.h):

```c
STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);
...
STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);
```

`STATIC_MEM_TASK_ALLOC` declares the task's stack as a static array, so all task memory is reserved at compile time rather than malloc'd at runtime. This way, RAM usage is known when the firmware is linked. The flip side is that RAM is a hard budget: the LQR controllers below hold their gain schedules in static arrays tens of kilobytes wide, and `STABILIZER_TASK_STACKSIZE` in `config.h` had to be raised from 3 to 5 × `configMINIMAL_STACK_SIZE` here because the deeper controller math overflowed the stock stack. If you add something large, expect to trade it against something else.

### 2.2 Boot sequence

Execution starts in [src/init/main.c](src/init/main.c), and it is short enough to quote almost in full:

```c
int main()
{
  check_enter_bootloader();
  int err = platformInit();
  ...
  systemLaunch();
  vTaskStartScheduler();
  ...
}
```

`platformInit()` figures out which board this binary is running on and refuses to continue on the wrong hardware — for us that means one of the Bolt board IDs listed in [src/platform/src/platform_bolt.c](src/platform/src/platform_bolt.c) (`CB11` for a Bolt 1.1). This is also where the board's motor map (brushless, one PWM channel per ESC) and sensor implementation are chosen. `systemLaunch()` in [src/modules/src/system.c](src/modules/src/system.c) creates exactly one task, the system task, and then `vTaskStartScheduler()` hands control to FreeRTOS and never returns. From this point on, everything happens inside tasks.

The system task, `systemTask()` brings the drone up: it initializes hardware, drivers, communication, as well as higher-level modules in `xxxInit()` functions, which set up their own data structures, and create their own tasks. So the drone's task set is built up during this one pass, in a fixed order. One of these is `appInit`:
```c
#ifdef CONFIG_APP_ENABLE
  appInit();
#endif
```

`appInit()` is a weakly-defined stub in the stock firmware; in this repository it is provided by [app/src/app.c](app/src/app.c), and it is designed to be the entry point of user-defined extensions to the firmware, of which Skybrush is one.

### 2.3 The stabilizer loop

This is the heart of the firmware: `stabilizerTask()` in [src/modules/src/stabilizer.c](src/modules/src/stabilizer.c) runs a loop at 1 kHz that turns sensor readings into motor commands.
Stripped of logging and error handling, one iteration looks like this:

```c
  while(1) {
    sensorsWaitDataReady();
    sensorsAcquire(&sensorData, tick);

    stateEstimator(&state, tick);

    if (crtpCommanderHighLevelGetSetpoint(&tempSetpoint, &state, tick)) {
      commanderSetSetpoint(&tempSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
    }
    commanderGetSetpoint(&setpoint, &state);

    collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

    controller(&control, &setpoint, &sensorData, &state, tick);

    supervisorUpdate(&sensorData);

    if (emergencyStop || (systemIsArmed() == false)) {
      motorsStop();
    } else {
      powerDistribution(&control, &motorThrustUncapped);
      batteryCompensation(&motorThrustUncapped, &motorThrustBatCompUncapped);
      powerDistributionCap(&motorThrustUncapped, &motorPwm);
      setMotorRatios(&motorPwm);
    }
    tick++;
  }
```
Its loop does not sleep on a timer, rather, it waits for a semaphore that will be given by the sensor task. To turn sensor data into state, an extended Kalman filter runs in its own task ([estimator_kalman.c](src/modules/src/estimator/estimator_kalman.c), priority `KALMAN_TASK_PRI`). Measurements reach it through a queue, of which the estimator task is a consumer. Should the estimator task be starved, i.e. the queue not processed fast enough, the drone will emit warnings to the skybrush server's console.
The drone's setpoint will typically come from a high-level commander, which evaluates the active trajectory or motion primitive (takeoff, land, go-to, or a polynomial trajectory) at the current time.
Given a setpoint and a state, the `controller()` function (which comes from a dispatch table at [src/modules/src/controller/controller.c](src/modules/src/controller/controller.c), in order to be able to switch between controllers) calculates the forces/torques to achieve the desired trajectory. Section 4 lists the controllers that table holds on a Bumblebee.
A supervisor watches over the state and sensors, to decide whether motors should be cut. If it permits the control output, `powerDistribution()` in [src/modules/src/power_distribution_quadrotor.c](src/modules/src/power_distribution_quadrotor.c) maps thrust and torques onto four individual motor commands.

Two details in that last block are Bumblebee-specific and easy to trip over:

* `batteryCompensation()` is still called, but its output `motorThrustBatCompUncapped` is **thrown away** — the line that used it is commented out, and `powerDistributionCap()` is fed the uncompensated thrusts instead. Sag compensation is done inside the LQR/geometric controllers (they read the pack voltage themselves) rather than here.
* `powerDistributionCap()` clamps each motor to 40000 rather than the full `UINT16_MAX`, i.e. the ESCs are deliberately never commanded above ~61% of the PWM range.

### 2.4 Talking to the drone: CRTP

Everything the ground station does — reading telemetry, changing parameters, uploading trajectories, sending setpoints — goes over one protocol, **CRTP** (Crazy RealTime Protocol), implemented in [src/modules/src/crtp.c](src/modules/src/crtp.c). It is a packet protocol carried over the radio (or USB when debugging). A packet is small: at most 30 bytes of payload (`CRTP_MAX_DATA_SIZE`), plus a header naming a **port** (which subsystem this is for) and a **channel** (a sub-address within that subsystem).

The ports are enumerated in [src/modules/interface/crtp.h](src/modules/interface/crtp.h):

```c
typedef enum {
  CRTP_PORT_CONSOLE          = 0x00,
  CRTP_PORT_APP              = 0x01,
  CRTP_PORT_PARAM            = 0x02,
  CRTP_PORT_SETPOINT         = 0x03,
  CRTP_PORT_MEM              = 0x04,
  CRTP_PORT_LOG              = 0x05,
  CRTP_PORT_LOCALIZATION     = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_SETPOINT_HL      = 0x08,
  CRTP_PORT_PLATFORM         = 0x0D,
  CRTP_PORT_LINK             = 0x0F,
} CRTPPort;
```

A module claims a port by registering a callback, and from then on every packet on that port is handed to it. That single line is the whole extension mechanism, and it is how the Skybrush layer adds its own protocol on `CRTP_PORT_APP`:

```c
crtpRegisterPortCB(CRTP_PORT_APP, droneShowSrvCrtpCB);
```

Four subsystems built on top of CRTP are worth knowing by name, because between them they cover almost everything a ground station does.

**log**: A module exposes a variable by declaring it with macros. From [stabilizer.c](src/modules/src/stabilizer.c):

```c
LOG_GROUP_START(ctrltarget)
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)
LOG_GROUP_STOP(ctrltarget)
```

The ground station asks for `ctrltarget.x` by name, the drone sets up a "log block" that samples the address periodically, and the values are streamed back (or recorded to an SD card).

**param** ~ settings. Same idea with `PARAM_GROUP_START` / `PARAM_ADD`, except the ground station can also write:

```c
PARAM_GROUP_START(stabilizer)
PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
PARAM_GROUP_STOP(stabilizer)
```

Those two lines are what makes the runtime controller/estimator switching possible from the outside. On a Bumblebee this is how you move between the geometric controller and the LQR controllers in flight, and a lot of the physical constants (masses, rod length, motor-model coefficients) are exposed the same way, so they can be retuned without reflashing.

**mem** — bulk data. Log and param handle single scalars; anything larger (a trajectory, an LED pattern, a light program, an LQR gain schedule) goes through the memory subsystem in [src/modules/src/mem.c](src/modules/src/mem.c), which presents a set of virtual address spaces the ground station can read and write in chunks. A module registers a handler with a type tag and read/write callbacks. From [src/modules/src/crtp_commander_high_level.c](src/modules/src/crtp_commander_high_level.c):

```c
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_TRAJ,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};
...
  memoryRegisterHandler(&memDef);
```

The type tags are listed in [src/modules/interface/mem.h](src/modules/interface/mem.h); `MEM_TYPE_TRAJ` is how trajectories get uploaded, `MEM_TYPE_APP` and `MEM_TYPE_FENCE` are Skybrush additions, and `MEM_TYPE_LQR` / `MEM_TYPE_LQR_BOUNDS` are ours — they are how a whole time-varying LQR gain schedule is pushed into RAM before a flight (see section 4).

**setpoints** — the two commander ports. `CRTP_PORT_SETPOINT` and `CRTP_PORT_SETPOINT_GENERIC` carry low-level setpoints that go straight into the commander at `COMMANDER_PRIORITY_CRTP`; `CRTP_PORT_SETPOINT_HL` carries high-level commands (takeoff, land, go to, start trajectory) that are handled by [crtp_commander_high_level.c](src/modules/src/crtp_commander_high_level.c) and then played back inside the stabilizer loop. `CRTP_PORT_LOCALIZATION` is the other important one for us: external position and pose measurements from the motion capture system arrive there and are pushed into the Kalman filter.

### 2.5 Source layout and the build system

| Directory | Contents |
| --- | --- |
| [src/init/](src/init/) | `main()` and the reset/startup assembly. |
| [src/drivers/](src/drivers/) | Chip-level drivers: I2C, SPI, motors/PWM (including the ESC protocols), the IMU, the radio link to the nRF51. |
| [src/hal/](src/hal/) | Hardware abstraction one level up: sensors, power management, radio link abstraction. |
| [src/modules/](src/modules/) | The actual firmware logic — stabilizer, estimators, controllers, commander, CRTP, log/param/mem. This is where you will spend your time. |
| [src/deck/](src/deck/) | Drivers for expansion decks (marker deck, LED ring, Lighthouse, Loco/UWB, flow deck). |
| [src/platform/](src/platform/) | Per-board differences between the Crazyflie 2.x, the Bolt and the Roadrunner. `platform_bolt.c` and `platform_defaults_bolt.h` are the ones that apply to us. |
| [src/utils/](src/utils/) | Helpers: math, filters, ring buffers, assertions. |
| [app/](app/) | The Skybrush application layer. |

Inside `src/modules/`, headers live in `interface/` and implementations in `src/`, which is why an include is just `#include "controller.h"` — the interface directories are all on the include path via the top-level [Makefile](Makefile).

The build uses **Kbuild** and **Kconfig**, the same system the Linux kernel uses. Two file types matter:

A `Kconfig` file *declares* options — their type, default, help text and dependencies. From [src/modules/src/Kconfig](src/modules/src/Kconfig):

```
config CONTROLLER_PID
    bool "PID controller"
    help
        Use the PID (proportional–integral–derivative) controller as default
```

A `Kbuild` file *lists what to compile*, one object file per line. From [src/modules/src/controller/Kbuild](src/modules/src/controller/Kbuild):

```
obj-y += controller.o
obj-y += controller_geom.o
obj-y += controller_lqr_1dof.o
obj-y += controller_lqr_2dof.o
```

The chosen option values come from `app/app-config` (assembled from the `.cfg` files, see section 1.3). The build generates two things from them: `autoconf.h`, which defines `CONFIG_xxx` macros so C code can `#ifdef` on them, and variables that `Kbuild` files can test, so `obj-$(CONFIG_DECK_LIGHTHOUSE) += lighthouse.o` compiles a file only when its option is on.

The practical rule: **a new `.c` file is not compiled until you add an `obj-y` line for it to the `Kbuild` in its directory.** Forgetting this produces a linker error about an undefined function, not a compiler error, which is confusing the first time.

## 3. Skybrush app

Skybrush added functionality for drone shows which include a light program, a trajectory, and a start time. These are implemented as an *app*, in [app/](app/), rather than as changes to the stock firmware.

The app consists of eight modules, each with a header in [app/interface/](app/interface/) and an implementation in [app/src/](app/src/). One extra submodule comes with it, `app/vendor/libskybrush`, which is CollMot's own library for decoding the compressed trajectory and light program formats.

| Module | Responsibility |
| --- | --- |
| [drone_show.c](app/src/drone_show.c) | The show state machine. The central module. |
| [preflight.c](app/src/preflight.c) | Periodic preflight checks that gate takeoff. |
| [light_program.c](app/src/light_program.c) | Plays the uploaded light program on the LED ring. |
| [gcs_light_effects.c](app/src/gcs_light_effects.c) | Light effects triggered manually from the ground station. |
| [fence.c](app/src/fence.c) | Safety fence (geofence). |
| [arming.c](app/src/arming.c) | Automatic arming before takeoff and disarming after landing. |
| [crtp_drone_show_service.c](app/src/crtp_drone_show_service.c) | The radio protocol the Skybrush server talks to. |
| [custom_params.c](app/src/custom_params.c) | Parameter overrides applied at boot. |

Two of them carry Bumblebee-specific content. [custom_params.c](app/src/custom_params.c) holds the boot-time overrides for this airframe — most importantly the battery thresholds (`pm.lowVoltage` 13.2 V, `pm.criticalLowVoltage` 12.8 V, i.e. a 4-cell pack, versus 3.2/3.0 V for a single-cell Crazyflie), so this is the file to edit if the pack changes. [crtp_drone_show_service.c](app/src/crtp_drone_show_service.c) is where the payload/LQR extensions to the show protocol live; see below.

## Upstream documentation

* [Bitcraze firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
* [Bitcraze Crazyflie Bolt datasheet](https://www.bitcraze.io/documentation/hardware/crazyflie_bolt_1_1/crazyflie_bolt_1_1-datasheet.pdf)
* [Bitcraze building and flashing instructions](docs/building-and-flashing/build.md) (the generic, non-Skybrush build)

## License

The code is licensed under LGPL-3.0.
