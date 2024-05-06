#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dac::{DacCh1, DacChannel, Value};
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::DAC1;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_stm32::{gpio::Pull, interrupt::InterruptExt as _};
use embassy_time::{Duration, Instant, Timer, TICK_HZ};

use embassy_stm32::interrupt::{self, Priority};

use {defmt_rtt as _, panic_probe as _};

mod util;

#[embassy_executor::task]
async fn log_cpu_load() -> ! {
    /// 1000 is a scale factor, defmt macro should be adjusted according to it's length
    /// rounding + avoiding floating point operations
    ///
    /// XXX: as it's not an interrupt executor, tasks that take more than one second won't
    /// display load correctly
    const SCALE: i32 = 1000;
    loop {
        unsafe {
            Timer::after_secs(1).await;

            let idle_ticks = util::EXECUTOR.as_mut().unwrap().idle_duration.as_ticks();
            let cpu_load = (SCALE * (TICK_HZ - idle_ticks) as i32) / TICK_HZ as i32;
            info!("CPU load: 0.{:03}", cpu_load);
            util::EXECUTOR.as_mut().unwrap().idle_duration = Duration::from_ticks(0);
        }
    }
}

#[embassy_executor::task]
async fn pps_sync_task(
    mut pps_exti_input: ExtiInput<'static>,
    mut pll_vcxo: DacChannel<'static, DAC1, 1, NoDma>,
) -> ! {
    let mut pll_dac_val: u16 = 2394;
    pll_vcxo.set(Value::Bit12Right(pll_dac_val));

    let mut count_1000 = 1;
    let mut count_100 = 1;
    let mut count_10 = 1;

    let mut sum_1000 = 0u64;
    let mut sum_100 = 0u64;
    let mut sum_10 = 0u64;

    const AVERAGING_COUNT_1000: u64 = 1000;
    const AVERAGING_COUNT_100: u64 = 100;
    const AVERAGING_COUNT_10: u64 = 10;

    let mut short_before = 0;
    let mut before = 0;
    let mut is_first_measurement = true;

    loop {
        pps_exti_input.wait_for_rising_edge().await;
        let now = Instant::now().as_ticks();

        if is_first_measurement {
            is_first_measurement = false;
            continue;
        }

        sum_1000 += now - before;
        if count_1000 >= AVERAGING_COUNT_1000 {
            let avg_1000 = sum_1000 as f64 / AVERAGING_COUNT_1000 as f64;
            info!("Average diff (1000): {}", avg_1000);
            sum_1000 = 0;
            count_1000 = 1;
        } else {
            count_1000 += 1;
        }

        // For 100-averaging
        sum_100 += now - before;
        if count_100 >= AVERAGING_COUNT_100 {
            let avg_100 = sum_100 as f64 / AVERAGING_COUNT_100 as f64;
            info!("Average diff (100): {}", avg_100);

            if sum_100 > 10_000_000 * AVERAGING_COUNT_100 {
                pll_dac_val -= 1;
            } else if sum_100 < 10_000_000 * AVERAGING_COUNT_100 {
                pll_dac_val += 1;
            }

            info!("SET DAC VAL TO: {}", pll_dac_val);
            pll_vcxo.set(Value::Bit12Right(pll_dac_val));

            sum_100 = 0;
            count_100 = 1;
        } else {
            count_100 += 1;
        }

        // For 10-averaging
        sum_10 += now - short_before;
        if count_10 >= AVERAGING_COUNT_10 {
            let avg_10 = sum_10 as f64 / AVERAGING_COUNT_10 as f64;
            info!("Average diff (10): {}", avg_10);
            sum_10 = 0;
            count_10 = 1;
        } else {
            count_10 += 1;
        }

        short_before = now;
        before = now;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let e = util::Executor::take();

    interrupt::EXTI0.set_priority(Priority::P2);

    e.run(|spawner| {
        unwrap!(spawner.spawn(async_main(spawner)));
        // unwrap!(spawner.spawn(log_cpu_load()));
    })
}

#[embassy_executor::task]
async fn async_main(spawner: Spawner) -> ! {
    info!("Hello world");

    #[cfg(not(debug_assertions))]
    fn example() {
        info!("Enabling caches for release build");
        let mut cp = cortex_m::Peripherals::take().unwrap();
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);
    }

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = None;
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(10),
            mode: HseMode::Bypass,
        });
        config.rcc.csi = false;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL40,
            divp: Some(PllDiv::DIV1),
            divq: None,
            divr: None,
        });
        config.rcc.pll2 = None;
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let mut p = embassy_stm32::init(config);

    let mut pps_in = ExtiInput::new(p.PC0, p.EXTI0, Pull::Down);
    let mut pll_vcxo = DacCh1::new(p.DAC1, NoDma, p.PA4);

    unwrap!(spawner.spawn(pps_sync_task(pps_in, pll_vcxo)));
    info!("Clocks cocked");

    loop {
        Timer::after_millis(1000000000).await;
    }
}
