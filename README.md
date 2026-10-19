# DC Motor Driver Firmware made from `cortex-m-quickstart`

[![Rust](https://github.com/chama1176/dc-motor-driver-stm32g0-rs/actions/workflows/rust.yml/badge.svg)](https://github.com/chama1176/dc-motor-driver-stm32g0-rs/actions/workflows/rust.yml)


> A template for building applications for ARM Cortex-M microcontrollers

This project is developed and maintained by the [Cortex-M team][team].

## Dependencies

To build embedded programs using this template you'll need:

- Rust 1.31, 1.30-beta, nightly-2018-09-13 or a newer toolchain. e.g. `rustup
  default beta`

- The `cargo generate` subcommand. [Installation
  instructions](https://github.com/ashleygwilliams/cargo-generate#installation).

- `rust-std` components (pre-compiled `core` crate) for the ARM Cortex-M
  targets. Run:

``` console
$ rustup target add thumbv6m-none-eabi thumbv7m-none-eabi thumbv7em-none-eabi thumbv7em-none-eabihf
```

## Using this template

**NOTE**: This is the very short version that only covers building programs. For
the long version, which additionally covers flashing, running and debugging
programs, check [the embedded Rust book][book].

[book]: https://rust-embedded.github.io/book

0. Before we begin you need to identify some characteristics of the target
  device as these will be used to configure the project:

- The ARM core. e.g. Cortex-M3.

- Does the ARM core include an FPU? Cortex-M4**F** and Cortex-M7**F** cores do.

- How much Flash memory and RAM does the target device has? e.g. 256 KiB of
  Flash and 32 KiB of RAM.

- Where are Flash memory and RAM mapped in the address space? e.g. RAM is
  commonly located at address `0x2000_0000`.

You can find this information in the data sheet or the reference manual of your
device.

In this example we'll be using the STM32F3DISCOVERY. This board contains an
STM32F303VCT6 microcontroller. This microcontroller has:

- A Cortex-M4F core that includes a single precision FPU

- 256 KiB of Flash located at address 0x0800_0000.

- 40 KiB of RAM located at address 0x2000_0000. (There's another RAM region but
  for simplicity we'll ignore it).

1. Instantiate the template.

``` console
$ cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart
 Project Name: app
 Creating project called `app`...
 Done! New project created /tmp/app

$ cd app
```

2. Set a default compilation target. There are four options as mentioned at the
   bottom of `.cargo/config`. For the STM32F303VCT6, which has a Cortex-M4F
   core, we'll pick the `thumbv7em-none-eabihf` target.

``` console
$ tail -n6 .cargo/config
```

``` toml
[build]
# Pick ONE of these compilation targets
# target = "thumbv6m-none-eabi"    # Cortex-M0 and Cortex-M0+
# target = "thumbv7m-none-eabi"    # Cortex-M3
# target = "thumbv7em-none-eabi"   # Cortex-M4 and Cortex-M7 (no FPU)
target = "thumbv7em-none-eabihf" # Cortex-M4F and Cortex-M7F (with FPU)
```

3. Enter the memory region information into the `memory.x` file.

``` console
$ cat memory.x
/* Linker script for the STM32F303VCT6 */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 40K
}
```

4. Build the template application or one of the examples.

``` console
$ cargo build
```

## VS Code

This template includes launch configurations for debugging CortexM programs with Visual Studio Code located in the `.vscode/` directory.  
See [.vscode/README.md](./.vscode/README.md) for more information.  
If you're not using VS Code, you can safely delete the directory from the generated project.

# License

This template is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, the [Cortex-M team][team], promises
to intervene to uphold that code of conduct.

[CoC]: https://www.rust-lang.org/policies/code-of-conduct
[team]: https://github.com/rust-embedded/wg#the-cortex-m-team

## Setup
Set flash regiseter like this from STM32CubeProgrammer.
Code below is currently not working for me.
```
// Switch boot configuration. Use nBOOT0 for boot configuration.
perip.FLASH.optr.modify(|_, w| w.n_swboot0().clear_bit());
perip.FLASH.optr.modify(|_, w| w.n_boot0().set_bit());
```

## How to Run 

In windows powershell,

```shell
$env:DEFMT_LOG="info"
cargo run -r
```

## Motor control

位置はエンコーダカウント、速度はカウント/秒を使う。
モーター軸11 pulse/rev、ハードウェア4逓倍後44 count/rev、減速比18.75:1で、出力軸は825 count/rev。
起動時の位置を原点とする。Dynamixel製品の位置・速度単位とは異なる。

| 設定 | デフォルト |
| --- | --- |
| OperatingMode | 3: 位置PID（1: 速度PI、16: PWM直接指令） |
| Position P / I / D raw | 8192 / 11830 / 32000 |
| Velocity P / I raw | 160 / 1640 |
| 位置範囲 / 速度上限 | 0〜825 count / ±630 count/s |
| PWMLimit / GoalPWM | 443 / 885（885がデューティ100%） |
| 制御周期 / LPF | 1 ms / 位置D項20 Hz、速度PI 10 Hz |
| TorqueEnable | 0（起動時停止） |

PID/PIは出力制限とアンチワインドアップを使い、トルク・モード変更時に積分をリセットする。
位置D項はフィルタ済み速度を使う。ゲインの連続時間換算はP=`raw/128`、I=`raw/65536/0.001`、D=`raw/16*0.001`。

ビルドと実機なしのテスト:

```powershell
cargo build -r
cargo test --offline --manifest-path tests/host/Cargo.toml --target x86_64-pc-windows-msvc
cargo fmt --all -- --check
cargo fmt --manifest-path tests/host/Cargo.toml --all -- --check
```

通常mainにベンチ処理は含めない。実機試験には[独立example](examples/control_bench.rs)を使う。
**次のコマンドはモーターを動かす。** テーブルへ位置・速度指令を直接書き込み、約18.4秒でTorque Offにする。
環境変数`CONTROL_POSITION_STUDY`、`CONTROL_POSITION_VERIFY`、`CONTROL_VELOCITY_STUDY`、
`CONTROL_VELOCITY_VERIFY`、`CONTROL_IDENTIFY`を一種類だけ設定すると、それぞれの試験を選べる。
試験後は環境変数を消し、通常ファームを書き戻す。ベンチのままリセットすると再び動く。

```powershell
cargo run -r --example control_bench -- --rtt-channel-mode no-block-skip --no-location --no-timestamps
```

## Memo
- `cargo size --bin dc-motor-driver-stm32g0 --release -- -A`
- 
#### TIM16
- 1ms周期

#### TIM14
- 1ms周期

#### UART2
- RS485

  
