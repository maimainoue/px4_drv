# Sony CECH-ZD1 J (torne)

## 対応範囲

CECH-ZD1 J は USB `054c:04b2` の ISDB-T チューナー1基です。公開実装の基板資料では、USB ブリッジに Fujitsu MB86H47A、チューナー / 復調器に DiBcom DiB8096 (DiB8000 / DiB0090) を使用しています。外部チューナーファームウェアは不要です。

内蔵カードスロットは MB86H47A の UART を通じて PC/SC リーダーとして利用できます。Windows は DriverHost_PX4 の CardServer とアプリケーションローカルの `WinSCard.dll`、macOS は `pcsc/` の IFD ハンドラーから CardServer、Linux は `/dev/px4card/<ID>` と同 IFD ハンドラーを通ります。TVTest では torne の BonDriver とカードリーダーを個別に選べます。

CECH-ZD1 J は地上デジタル専用です。BS / CS の受信とカード利用には対応しません。

対応状況の `△` は、コード経路が追加済みで実機品質の確認が残っている状態です。実機の確認は macOS と Linux の試験 (後述) で、Windows は自動試験とビルドの確認に限られます。

## 制御、カード UART、ストリーム

MB86H47A のメモリー読み書きは USB vendor control request `0xbc` で行い、DiB8096 へは内蔵 I2C master を通じてアクセスします。初期化はブリッジ起動、復調器 reset、DiB8096 初期化の順です。選局時は TS bridge を停止して設定し直し、受信中なら選局後にストリームを再開します。

受信 TS は bulk IN endpoint `0x81` の192-byte単位で届きます。各単位の先頭4-byte timestampを除き、188-byte MPEG-TS packetを共通ストリームバッファーへ渡します。endpoint 値は `torne_drv` のコードと `coax` の公開資料を参照しており、実機の endpoint descriptor の直接取得は未完了です。

統計は C/N を利用できます。Windows / macOS Host は共通 C/N 値を 0.001 dB 単位で返し、Linux の ptx `GET_CNR` は既存の torne 実装に合わせて 0.01 dB 単位です。Linux の信号強度は demodulator AGC 表からの推定値で、校正済み dBm ではありません。

カード UART は MB86H47A の `0xfea000`–`0xfea03c` のレジスターを使います。機種固有の reset、baud rate、FIFO の読み書きは [`torne_card.c`](../../driver/common/torne/torne_card.c) に置き、レジスター read/write、ロック、待機を callback に分離しています。ATR 完了待ち、解析、T=1 の I/R/S ブロック、再送、APDU 全体の絶対期限、セッション破棄は全機種共通の [`smart_card.cpp`](../../host/core/smart_card.cpp) が担当します。torne 固有層にはカード UART のハードウェア操作だけを置いています。

ホスト側はカード UART 用 mutex をレジスター1回の read/write ごとに取り、USB 制御転送 mutex も転送ごとに解放します。ATR / T=1 のポーリングと reset の待機中はどちらも保持せず、選局用 `mutex_` や TS の bulk 受信、framer、ストリームバッファーをカード通信で止めません。Linux も `operation_lock` とカード UART mutex を分離し、USB 制御転送を1回ずつ直列化します。待機はユーザー空間の PC/SC 層で行います。torne のカード VCC は UART が制御し、チューナーと共有する基板電源参照はありません。

## OS ごとの利用

- macOS: `DriverHost_PX4` の libusb 経路で受信し、`pcsc/` の IFD ハンドラーが同じ CardServer に接続します。DriverHost_PX4 の起動後、PC/SC アプリケーションから `Sony CECH-ZD1 J Smart Card Reader` を選べます。
- Windows: 同梱の `Torne.inf` を使って WinUSB を割り当て、`BonDriver_Torne.dll` と `BonDriver_Torne.ini` を配置します。
- Linux: `px4_drv` の kernel module が `/dev/tornevideoN` と `/dev/px4card/<ID>` を公開します。映像デバイスは `px4video*` と同じ ptx ioctl / read API です。例: `recisdb tune --device /dev/tornevideo0 --channel T33 -t 60 out.ts`。

## 実装元と検証

Linux 用 C 制御コアは [mihalinn/torne_drv](https://github.com/mihalinn/torne_drv) (`fc662902a3e4ad56566b00cae4a0ead922e3dd02`) から移植しました。ブリッジと DiB8096 のプロトコル資料は [Till0196/coax](https://github.com/Till0196/coax) (`6aaa1899991c875ba0482fa9098adc6c83b01fd8`) を参照しています。

自動試験は vendor control の値・index・big-endian データ、分割 USB input の TS framing、模擬ブリッジ起動、TS start/stop、カード UART の reset / baud rate / FIFO と lock-release を確認します。共通 SmartCard の模擬試験では ATR 完了待ち、T=1 I/R/S ブロック、NAD、再送、絶対期限、抜去後の再初期化を確認します。自動試験は実機の RF lock、カード応答、TS との同時負荷を証明しません。

### macOS の実機試験 (2026-11)

Apple Silicon Mac に CECH-ZD1 J を1台接続し、地上波 T21 のアンテナ入力と B-CAS カードを挿した状態で、専用の `PX4_RUNTIME_DIR` から起動した DriverHost_PX4 を使いました。カードの抜き差しは試験の対象外です。

| 試験 | 条件 | 結果 |
|---|---|---|
| 選局 (T21) | DriverHost_PX4 の `Tune timing` | 周波数設定 3118 ms、ロック 3121 ms、最初の TS 3147 ms。C/N 約 22.76 dB |
| 60秒受信 (T21) | `px4rec` で録画 | 688,682 パケットで同期エラー・TEI・CC 欠落はすべて0、17.27 Mbps |
| 選局と Open/Close の反復 | `px4rec -d 2` を T21 / T28 で交互に100回 | 100回すべて成功 |
| 30分連続受信 (T21) | `px4rec` で録画し、DriverHost_PX4 の5秒ごとの統計を記録 | 20,666,154 パケットで同期エラー・TEI・CC 欠落はすべて0。統計382行すべてで同じ。平均 17.27 Mbps |
| カード単独 | ATR 取得後、初期設定 APDU を200回 | ATR 13 バイト、T=1。成功200回、失敗0回、最大応答 57.4 ms、平均 53.2 ms |
| 受信と APDU の並行 | `px4rec -d 300 T 21` の受信中に 200 ms 間隔で APDU を290秒 | APDU 成功1117回、失敗0回、最大応答 66.0 ms。TS 3,444,266 パケットで同期エラー・TEI・CC 欠落はすべて0、17.27 Mbps、パケット間隔の最大 35 ms |
| リアルタイム復号 | T21 を120秒受信し、libaribb25 が torne のカードを使って復号 | 1,377,578 パケットでスクランブル残り・同期エラー・TEI・CC 欠落はすべて0。ECM の APDU 61回で失敗0 |

DriverHost_PX4 は各試験の後に SIGINT で正常終了しました。T28 の TS 品質の個別計測、カードを使いながらの長時間受信、抜き差し後の再利用、Windows の実機受信、Windows の TVTest での復号、Linux の PC/SC 経路は未確認です。Linux の実機受信は次の節に記録しています。

### Linux の実機試験 (2026-11)

CECH-ZD1 J を1台、Proxmox 上の Debian 13 (カーネル 6.12.116、デバッグ無し) の仮想マシンへ USB パススルーで接続しました。`px4_drv` はカーネルのヘッダーでビルドし、受信は UHF 27ch、カードは B-CAS を挿した状態です。
アンテナ入力は、PX-S1UD V2.0 と同じアンテナの分岐です。試験ツールは `/dev/tornevideo0` を ptx ioctl で選局して受信し、`/dev/px4card/<ID>` で T=1 の S ブロック (IFS 要求) の往復を 200 ms 間隔で繰り返します (当時は DT295 向けの名前だった `dt295_chrdev_test` と `dt295_card_chrdev_test`。現在は、機種に依らない [`ptx_chrdev_test` と `px4card_test`](../tools.md) にした)。

Linux 6.12 では、最初の `probe` が `-EAGAIN` で失敗しました。共有コアとカード層がスタック上の配列を `usb_control_msg` の転送バッファとして渡していたためで、`torne_control_timeout()` がヒープの一時領域を経由する形に直してあります ([20261113_common_fixes.md](../20261113_common_fixes.md))。修正後の結果は次のとおりです。

| 試験 | 条件 | 結果 |
|---|---|---|
| 60秒受信 + カード | 受信中に、IFS の往復を 200 ms 間隔で55秒 | 688,640 パケットで同期ずれ・TEI・連続性エラーはすべて0、17.3 Mbps。ATR 13 バイト、往復272回で失敗0、平均 90.1 ms、最大 230.5 ms |
| 開始と停止の反復、読み手の強制終了 | 2秒の受信を8周、受信中の `kill -9` を3周 | 失敗0。強制終了の後の受信も飛び0 |
| 受信中とカード中の unbind | 受信とカードの往復の最中に unbind し、再接続して受信を3周 | 再接続でき、最初の受信を含めて飛び0 |
| デバイスファイルが開いたままの unbind | 別のプロセスが開いたまま unbind し、そのプロセスを終了させて再接続を5周 | 毎回再接続でき、その後の受信も飛び0 |
| `rmmod` と `insmod` | 3周 | 毎回 probe でき、受信も飛び0 |
| dmesg | 上の試験の間 | 警告と異常は0件 |

カードの往復の最大 230.5 ms は、macOS の同種の試験 (最大 66.0 ms) より大きい値です。試験の条件が違うため、原因は調べていません。

10分間の同時受信 (PX-S1UD V2.0 と同時、同じアンテナの分岐) では、6,868,033 パケットで同期ずれ 0、TEI 0、連続性エラー 7,431 件でした。連続性エラーは 60 区間 (10秒ごと) のうち 27 区間に出ています。
PX-S1UD の問題区間 (TEI の出た区間) と時刻が一致する区間が 26 あり、受信信号の揺らぎによるとみられます (推定)。詳細は [Siano の別紙](../siano_tuners.md) にあります。

その後、別の環境の仮想マシンで、6.12.117 の 20 分の受信 (エラー 0) と、6.1.0 でカードを同時に使った 10 分の受信 (エラー 0) を確認しています。
カーネル 6.1 と 6.8 の仮想マシンと、実機のホストでの確認は、次の節と [20261121_linux_other_kernels_validation.md](../20261121_linux_other_kernels_validation.md) にあります。

### Linux の PC/SC 経路での確認 (2026-11)

Linux 6.12.116 (通常のカーネル、Proxmox の仮想マシンへ USB パススルー) で、`pcscd` 2.3.3 と IFD ハンドラー (`pcsc/`) を通して確認しました。
復号には、tsukumijima の libaribb25 0.2.10 を使い、読み取り機の名前を指定する小さな検査用プログラムから呼びました。

| 確認項目 | 結果 |
|---|---|
| `pcscd` の読み取り機の登録とカードの状態 | 登録でき、「Card inserted」で、B-CAS (地上デジタル) と識別された (ATR `3B F0 12 00 FF 91 81 B1 7C 45 1F 03 99`) |
| B-CAS の初期化コマンド (`90 30 00 00 00`、読み取り専用) を 10 回 | 10/10 成功、応答 61 バイト、SW=9000、69〜74 ms (T=1) |
| 自機の TS (UHF 27ch、20 秒、229,376 パケット) を、自分のカードで復号 | スクランブル 210,877 パケット (91.9%) が **0**。`put` と `get` のエラー 0 |
| PX-S1UD V2.0 の TS を、このカードで復号 | スクランブル 211,200 パケットが **0** |
| 受信しながらのカード通信 (4 機種の同時受信 60 秒と、APDU を毎秒 5 回) | torne の TEI 0、連続性エラー 0。APDU は 276 回すべて成功、最大遅延 97 ms |
| `pcscd` の動作中 (挿抜の監視の検出あり) と停止中の受信 (30 秒) | どちらも TEI 0、連続性エラー 0 で、差なし |

この確認の途中で、`pcscd` から状態が取得できない不具合を見つけて直しました (挿抜の監視が、カード I/O の期限を渡していなかった。`pcsc/ifdhandler.cpp` の `CheckPresence`)。
直す前は、`pcscd` の起動の約 1 秒後から、検出がすべて `ETIMEDOUT` になり、torne は、読み取り機として登録されませんでした。

### Linux の実機のホストでの確認 (2026-11-21)

Debian 13 (カーネル 6.12.117、デバッグ用の設定なし) の実機のホスト (仮想マシンを使わない) で、CECH-ZD1 J の 3 台 (`054c:04b2`) を同時に接続して確認しました。受信は ISDB-T の UHF 21ch、B-CAS カードは 3 台とも挿した状態です。

- **probe:** カーネルモジュールの修正前 (制御転送がスタック上の配列を渡す問題。[20261113_common_fixes.md](../20261113_common_fixes.md)) は、3 台とも `MB86H47A read 0x082008: -11` で失敗しました。修正後は、3 台とも `/dev/tornevideo0`〜`2` として登録されました。
- **受信:** 2 台が信号を受けました。この 2 台を、PX-Q3U4 の ISDB-T の 4 ノード、PIX-DT295、GV-MVP/XZ、PX-S1UD V2.0 と同時に 120 秒受信 (2 回) して、同期ずれ、TEI、連続性エラーとも 0、17.26 Mbps でした。残りの 1 台 (`tornevideo0`) は、選局できますが、全パケットがヌルで、信号がありません (アンテナの接続は確認していません)。
- **取り外し:** デバイスファイルを別のプロセスが開いたままの `unbind` (1 台で実施) は、閉じるまで待たされ、強制終了の 1 秒後に完了しました。再接続後に、再び受信できました。
- **カード** ([`px4card_test`](../tools.md)、3 台): ATR は 13 バイト (`3B F0 12 00 FF 91 81 B1 7C 45 1F 03 99`)、T=1 の S ブロックの往復 15 回が、3 台とも失敗 0、平均 78.4〜78.6 ms、最大 80.7〜81.5 ms でした。上の仮想マシンの試験 (平均 90.1 ms、最大 230.5 ms) より短い値です。
- **`pcscd`** (pcsc-lite 2.3.3、`px4-drv-pcsc` のバンドルを導入し、USB のホットプラグで検出): 修正前の IFD ハンドラーは、3 台とも登録できませんでした (`PX4_CARD_DETECT` が `ETIMEDOUT`。`px4card_test` と違い、検出の前に期限を渡していなかったため)。修正後は、3 台とも登録され、「Card inserted」と ATR を取得でき、B-CAS の初期化コマンド (`90 30 00 00 00`) が、確認した 2 台で SW=9000 で成功しました。`pcscd` が動いている間は、`px4card_test` は `EBUSY` で失敗します (カードリーダーを同時に開けるのは 1 つのプロセスだけです。[確認用ツール](../tools.md))。
- **カーネル 6.1 と 6.8:** Debian 12 (6.1.0-56) と Ubuntu 24.04 (6.8.0-151) の仮想マシン (QEMU、USB パススルー) で、`tornevideo1` を 10 機材の同時受信 (10 分) に入れて、エラー 0 でした (6.8 では `tornevideo2` も、エラー 0)。開いたままの `unbind` と再接続 3 回、`rmmod` と `insmod` 3 回も、問題ありませんでした。
- **初回の選局の失敗:** モジュールの読み込み直後の最初の選局が、`ETIMEDOUT` で失敗することがあります (3 台目 `tornevideo2` で、実機のホストと、6.1、6.8 の仮想マシンのいずれでも)。6.1 では、同じチャンネルの選局が、その後も数回失敗しました。別のチャンネルで選局すると、以降は受信できました。原因は確認していません。

Linux で未確認なのは、torne 単独での 20 分を超える連続受信 (実機のホストでは、10 分の同時受信まで)、USB の物理的な抜去、6.1、6.8、6.12 以外のカーネル版、`pcscd` を使いながらの長時間の受信です。
