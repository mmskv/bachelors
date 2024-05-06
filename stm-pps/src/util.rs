pub use thread::*;
mod thread {
    pub(super) const THREAD_PENDER: usize = usize::MAX;

    pub static mut EXECUTOR: Option<Executor> = None;

    use core::arch::asm;
    use core::marker::PhantomData;
    use embassy_time::{Duration, Instant};

    use embassy_executor::{raw, Spawner};

    pub struct Executor {
        inner: raw::Executor,
        not_send: PhantomData<*mut ()>,
        pub idle_duration: Duration,
    }

    impl Executor {
        pub fn take() -> &'static mut Self {
            critical_section::with(|_| unsafe {
                assert!(EXECUTOR.is_none());

                EXECUTOR = Some(Self {
                    inner: raw::Executor::new(THREAD_PENDER as *mut ()),
                    not_send: PhantomData,
                    idle_duration: Duration::from_ticks(0),
                });

                EXECUTOR.as_mut().unwrap()
            })
        }

        pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
            init(self.inner.spawner());

            loop {
                unsafe {
                    self.inner.poll();
                    let wait_start = Instant::now();
                    asm!("wfe");
                    let wait_end = Instant::now();
                    self.idle_duration += wait_end
                        .checked_duration_since(wait_start)
                        .unwrap_or(Duration::from_ticks(0));
                };
            }
        }
    }
}
