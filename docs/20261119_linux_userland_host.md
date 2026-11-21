# Linux のユーザーランド版の確認 (2026年11月19日)

この文書は、macOS 版の `DriverHost_PX4` と `px4rec` (どちらも `libusb` 経由のユーザー空間実装) を Linux でもビルドして動かし、共通層の変更を、カーネルモジュールとは別の経路で実機確認できるようにした記録です。
Linux の常用経路は、これまでどおりカーネルモジュール (`driver/linux/`) です。ここで扱うのは、確認用の構成です。

## 目的

`driver/common/` と `host/` の共通層は、Linux カーネル、WinUSB、macOS の 3 つの経路から使われます。
共通層を変えたとき (例: DT295 のチューナー設定の追加) は、カーネル経路だけでなく、ホスト経路 (`host/devices/`) でも動作を確かめる必要があります。
従来は、ホスト経路の実機確認が Windows と macOS に限られていました。Linux でも `libusb` 経由のホストが動けば、同じ機材と環境で、カーネル経路との比較ができます。

## 設計

### `PX4_USERLAND` マクロ

共通ソースは、OS ごとに次のように分かれています。

| 条件 | 意味 |
|---|---|
| `__linux__` かつ `PX4_USERLAND` なし | Linux カーネルモジュール (`linux/device.h`、`linux/usb.h` などを使う) |
| `__APPLE__` または `PX4_USERLAND` | `libusb` と POSIX のユーザー空間 (`misc_posix.h`、`libusb_compat.h`) |
| `_WIN32` / `_WIN64` | WinUSB |

従来は、ユーザー空間の分岐が `__APPLE__` で書かれていたため、Linux では `libusb` 経由の経路を選べませんでした。
`PX4_USERLAND` は、ユーザー空間の `libusb` 経路を、OS に依らず選ぶためのマクロです。`macos/CMakeLists.txt` が、すべてのターゲットに `PX4_USERLAND=1` を定義します。

- **カーネルモジュールへの影響:** カーネルのビルドでは `PX4_USERLAND` が未定義のため、各分岐の結果は変わりません。
- **macOS への影響:** `__APPLE__` が真のままで、`PX4_USERLAND` も定義されるため、選ばれる分岐は変わりません。
- **Windows への影響:** `_WIN32` の分岐は変更していません。ただし、`winusb/src/DriverHost_PX4/misc_win.h` と `winusb_compat.h` の先頭の `#ifdef __APPLE__` は、`__APPLE__` または `PX4_USERLAND` に変えています (Windows では、どちらも未定義)。

変更したのは、`driver/common/` と `host/` の約 25 ファイルの `#ifdef __linux__`、`#ifdef __APPLE__`、`#elif defined(__APPLE__)` の判定だけです。

### 実行ファイルのパス

`macos/src/common/exe_path.h` の `px4::GetExecutablePath()` が、macOS では `_NSGetExecutablePath()`、Linux では `readlink("/proc/self/exe")` で、実行ファイルのパスを返します。
`util.cpp`、`px4rec.cpp`、`host/devices/` の 3 つのデバイス実装 (ファームウェアの探索) が使います。

### PC/SC の型

`macos/src/tools/b25/b_cas_card.c` は、PC/SC の API に、`uint32_t` へのポインターを渡していました。macOS では `DWORD` が 32 bit ですが、Linux の pcsc-lite では `unsigned long` (64 bit) のため、型を `DWORD` に合わせています。

### CMake

`macos/CMakeLists.txt` は、`-framework` の指定を、`PX4_PLATFORM_LIBS` と `PX4_PCSC_LIBS` の 2 つの変数にまとめました。macOS では従来の framework、Linux では何も足さず、PC/SC は `pkg-config` の `libpcsclite` を使います。

## ビルド (Linux)

`libusb-1.0-0-dev`、`libpcsclite-dev`、`cmake`、C++17 のコンパイラーが必要です。

```
cmake -S macos -B macos/build -DCMAKE_BUILD_TYPE=Release
cmake --build macos/build --target DriverHost_PX4 px4rec
```

`macos/build/` に、`DriverHost_PX4` (約 1 MB) と `px4rec` ができます。`DriverHost_PX4.ini` は、実行ファイルと同じディレクトリに置きます。

起動する前に、対象の USB 機材を、カーネルのドライバー (`px4_usb`) から外します (`/sys/bus/usb/drivers/px4_usb/unbind`)。同じ機材を、カーネルモジュールとユーザーランド版の両方で同時には開けません。

## 確認した範囲

### 環境

- Debian 13 の仮想マシン (x86_64、カーネル 6.12.117)。コンパイラーは GCC 14.2、`libusb` 1.0.28、pcsc-lite 2.3.3。
- 機材: PIX-DT295 Rev.01 と Rev.02 の 2 台 (USB パススルー)。アンテナの分岐から UHF 13/16/21/30/37ch を受信。
- ホストは、`DriverHost_PX4.ini` で DT295 の 2 つの定義だけを有効にして (他の機種の `ProductId` を `0xffff`)、起動しました。録画は `px4rec -i 0 -d <秒> T <ch> <出力>` で、TS は別の小さな解析ツールで数えました。

### 結果

| 確認項目 | 結果 |
|---|---|
| ビルド | `DriverHost_PX4` と `px4rec` が完成 (警告の数は、確認していません) |
| 起動 | `DT295 ready: host_index=1` と `host_index=2` が出て、2 台を認識 |
| ch21 の 600 秒 (2 台同時、`px4rec -i 0` を 2 つ) | 11,951,445 と 11,911,593 パケット (後の 1 台は 2 秒遅れで 598 秒)。同期ずれ、TEI、連続性の飛びとも 0 |
| ch30 の 600 秒 (同上) | 11,951,444 と 11,911,594 パケット。同期ずれ、TEI、連続性の飛びとも 0 |
| ch13、16、21、30、37 の切り替え 6 周 (各 8 秒、1 台) | 30 回すべて、同期ずれ、TEI、連続性の飛びとも 0 |
| 選局の所要時間 (ホストのログ) | `set_ms` は 59〜73、`lock_ms` は 450〜633、最初の TS までは約 1,000〜1,200 ms |

DT295 の「チューナー `0x08 = 0x05`」と「復調器 `0x01 = 0x40`」を足した共通層 ([DT295 の別紙](devices/dt295.md)) を、カーネルモジュールとは別の経路 (ホスト) で、同じ機材のまま確認できました。選局直後の TEI と連続性エラーが 0 になる点も、カーネル経路と同じです。

### 未確認のこと

- DT295 以外の機種 (PX-W3U4、PX-MLT5PE、TBS 5530、torne、GV-MVP/XZ、PX-S1UD、T230A) での、Linux のユーザーランド版の動作。
- R850 チューナーを使う機種 (PX4 系、ISDB2056 系、ISDBT2071 系) での、Linux のユーザーランド版の動作。ユーザーランドでは、`mdelay` が何もしない定義 (`winusb/src/DriverHost_PX4/misc_win.h` と `macos/src/DriverHost_PX4/misc_posix.h`) のため、`driver/common/tuner/r850.c` の `mdelay(2)` と `mdelay(5)` は待機しません (`msleep` は実際に待ちます)。Windows と macOS では、元から同じ動作で、Linux のユーザーランド版も同じ定義を使います。待機が不要かどうかは、R850 の仕様では確認していません。
- カードリーダー (B-CAS)、`--b25` での復号、`pcscd` との組み合わせ。
- ホストの常用 (起動の自動化、`udev` の権限、`root` なしでの起動)。常用する構成ではないため、整備していません。
- macOS のビルドと動作が、この変更で退行していないこと (macOS の環境がなく、未確認)。
- 24 時間級の連続受信、受信中の USB の抜去、ホストの再起動後の再列挙。

## 追加の確認 (2026年11月21日)

Debian 13 (カーネル 6.12.117) の実機のホスト (仮想マシンを使わない) で、`DT295`、`XZ`、`TBS 5530` などの変更を取り込んだ後の構成を確認しました。コンパイラーは GCC 14.2、`libusb` 1.0.28、pcsc-lite 2.3.3 です。

### ビルドと自動試験

- `cmake -S macos -B build` の全ターゲットが、ビルドできました。`macos` 専用の API を使う 2 本の試験、`itedtv_usb_failure_test` (`malloc/malloc.h`) と `stream_server_failure_test` (`SO_NOSIGPIPE`) は、Linux では登録しません (`if(APPLE)`)。
- `ctest` は、28 本すべてが通りました。通った中には、機種の定義が OS 間で一致することを検査する `device_parity_test` と、各機種の共通層の試験 (`torne`、`xz`、`dt295`、`t230c`、`siano` など) を含みます。
- 同じ変更で、Windows (MSVC v143 の x64 と x86) の全体のビルドと、共通層の自動試験 (TBS 5530、GV-MVP/XZ、PIX-DT295) が通ること、Linux のカーネルモジュールがビルドできることを確認しました。

### 実機

- `DriverHost_PX4` を、設定ファイルを変えずに起動すると、PIX-DT295 の 2 台、GV-MVP/XZ、Sony CECH-ZD1 J の 3 台を認識しました。ホストは、カーネルのドライバー (`px4_usb`) が付いている機材も、`libusb` が取り込みます。他の機材を使わないときは、`DriverHost_PX4.ini` で、使わない機種の `ProductId` を `0xffff` にしてください。
- `px4rec -d 30 T 21` で、1 台から約 17.27 Mbps の TS を 30 秒受信し、同期ずれ、TEI、連続性の飛びとも 0 でした (この録画で、どの機材が使われたかは、ログから特定できていません)。アンテナのない PIX-DT295 Rev.01 が選ばれた録画は、`TUNE failed` で終了しました。
- ホストを `SIGTERM` で止めて、カーネルモジュールを入れ直すと、すべての機材 (PX-Q3U4、TBS 5530、Sony CECH-ZD1 J 3 台、PIX-DT295 2 台、GV-MVP/XZ、PX-S1UD V2.0) が、元どおり登録されました。ホストの使用の後も、DT295 と GV-MVP/XZ の「起動直後」の状態が保たれました。
- `-i N` で、特定の機材を選ぶ確認は、できていません (`-i 0` 以外は `no T receiver available` になり、`-i 0` は、アンテナのない DT295 Rev.01 の選局の待ちで止まりました)。
- TBS 5530 は、ファームウェア (`dvb-usb-id5530.fw`) を、実行ファイルと同じディレクトリに置いていなかったため、初期化に失敗しました。PX-Q3U4 の初期化も、カーネルのドライバーから取り込む途中で失敗しました (`-22`)。いずれも、この構成での常用を想定していない部分です。
