# リポジトリガイドライン

## プロジェクト構成とモジュール

このリポジトリは、STM32G030（Cortex-M0+）向けの `no_std`、`no_main` Rust ファームウェアです。`src/main.rs` は起動処理、割り込みハンドラ、メインループを管理します。`src/app.rs` にはデバイス非依存のアプリケーション処理、`src/dc_motor_driver_stm32g0.rs` には STM32 のペリフェラル実装、`src/indicator.rs` にはインジケータの抽象化があります。

リンカおよびプローブの設定は `memory.x`、`build.rs`、`.cargo/config.toml`、`openocd.cfg`、`openocd.gdb` にあります。CI は `.github/workflows/rust.yml`、エディタの起動設定は `.vscode/` にあります。

このクレートは `../dynamixel-f-rs/` と `../motml-rs/` をパス依存関係として使用します。ローカルでビルドするときは、これらのリポジトリを同じ親ディレクトリに配置してください。

## ビルド、テスト、開発用コマンド

- `rustup target add thumbv6m-none-eabi`: `.cargo/config.toml` で指定されたターゲットを追加します。
- `cargo build`: デバッグ用ファームウェアをビルドします。
- `cargo build --release`: CI および実機実行向けの最適化済みイメージをビルドします。
- `cargo run --release`: 設定済みの STM32G030 とプローブ ID を使い、`probe-rs` で書き込みと実行を行います。実行前に接続対象を確認してください。
- `cargo fmt --all -- --check`: フォーマットを検証します。修正には `cargo fmt --all` を使用します。
- `cargo test`: CI と同じテストコマンドです。ただし、現在はバイナリのテストが無効で、専用テストもありません。
- `cargo size --bin dc-motor-driver-stm32g0 --release -- -A`: `cargo-binutils` が導入済みの場合にセクションサイズを表示します。

## コーディング規約と命名

標準の `rustfmt` 出力（4 スペースのインデント）に従ってください。関数とモジュールには `snake_case`、型とトレイトには `CamelCase`、static 変数と定数には `SCREAMING_SNAKE_CASE` を使用します。ハードウェアレジスタへのアクセスは STM32 アダプターモジュールに置き、アプリケーション上の判断は `app.rs` に置いてください。割り込み内のクリティカルセクションは短くし、メモリ確保やブロッキング処理を避けてください。

## テスト方針

可能な限り、単体テスト可能なロジックをハードウェア固有コードから分離してください。テスト名は `sets_led_when_control_flag_is_enabled` のように、確認する動作を表す名前にします。提出前にフォーマット検証、リリースビルド、STM32G030 実機での関連動作確認を行ってください。PR には使用したボードとプローブ、および UART、PWM、エンコーダ、LED などの確認結果を記載します。

## コミットとプルリクエスト

履歴では、`add control` や `fix warn` のような短く焦点を絞った英語または日本語の件名が使われています。命令形の要約を使い、1 コミットを 1 つのまとまった変更にしてください。`.github/PULL_REQUEST_TEMPLATE.md` に従い、変更内容、関連 Issue、テスト方法、補足事項を記載します。ハードウェア動作の変更にはログや測定結果を添付してください。スクリーンショットはデバッガやツールの UI 変更がある場合に添付します。
