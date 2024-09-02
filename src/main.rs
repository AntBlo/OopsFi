#![no_std]
#![no_main]

use core::net::{Ipv4Addr, SocketAddrV4};

use edge_dhcp::{
    io::DEFAULT_SERVER_PORT,
    server::{Server, ServerOptions},
};
use edge_nal::UdpBind;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{AnyInput, AnyOutput, Input, Io, Level, Output},
    peripherals::Peripherals,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_wifi::{
    initialize,
    wifi::{AccessPointConfiguration, Configuration, WifiApDevice, WifiController, WifiDevice},
    EspWifiInitFor,
};
use log::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let is_unstable_wifi = mk_static!(
        AnyInput,
        AnyInput::new(io.pins.gpio2, esp_hal::gpio::Pull::Down)
    );
    let led = mk_static!(AnyOutput, AnyOutput::new(io.pins.gpio3, Level::Low));

    let init = initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_ap_interface, _, mut controller) = esp_wifi::wifi::new_ap_sta(&init, wifi).unwrap();

    use esp_hal::timer::systimer::{SystemTimer, Target};
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(&clocks, systimer.alarm0);

    let ap_config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 1, 1), 24),
        gateway: Some(Ipv4Address::from_bytes(&[192, 168, 1, 1])),
        dns_servers: Default::default(),
    });

    let seed = 1234; // very random, much secure

    let ap_stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiApDevice>>,
        Stack::new(
            wifi_ap_interface,
            ap_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    let client_config = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: "esp-wifi".try_into().unwrap(),
        ..Default::default()
    });

    controller.set_configuration(&client_config).unwrap();

    spawner
        .spawn(connection(controller, is_unstable_wifi, led))
        .ok();
    spawner.spawn(ap_task(ap_stack)).ok();
    spawner.spawn(run(ap_stack)).ok();

    loop {
        if ap_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_secs(100000000)).await
    }
}

#[embassy_executor::task]
async fn run(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) {
    let mut buf = [0; 1500];
    let ip = Ipv4Addr::new(192, 168, 1, 1);

    let buffers = edge_nal_embassy::UdpBuffers::new();
    let socket = edge_nal_embassy::Udp::<WifiDevice<WifiApDevice>, 1>::new(stack, &buffers);
    let mut socket = socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::new(0, 0, 0, 0),
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    edge_dhcp::io::server::run(
        &mut Server::<64>::new(ip),
        &ServerOptions::new(ip, Some(&mut gw_buf)),
        &mut socket,
        &mut buf,
    )
    .await
    .unwrap();
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    is_unstable_wifi: &'static mut AnyInput<'static>,
    led: &'static mut AnyOutput<'static>,
) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.get_capabilities());

    info!("Starting wifi");
    controller.start().await.unwrap();
    led.set_high();
    loop {
        info!("Waiting for high");
        is_unstable_wifi.wait_for_high().await;
        loop {
            Timer::after_millis(100).await;
            info!("Stopping wifi");
            controller.stop().await.unwrap();
            led.set_low();
            select(Timer::after_secs(30), is_unstable_wifi.wait_for_low()).await;
            if is_unstable_wifi.is_low() {
                break;
            }
            info!("Starting wifi");
            controller.start().await.unwrap();
            led.set_high();
            select(Timer::after_secs(30), is_unstable_wifi.wait_for_low()).await;
            if is_unstable_wifi.is_low() {
                break;
            }
        }

        if !controller.is_started().unwrap() {
            info!("Starting wifi");
            controller.start().await.unwrap();
            led.set_high();
        }
    }
}

#[embassy_executor::task]
async fn ap_task(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) {
    stack.run().await
}
