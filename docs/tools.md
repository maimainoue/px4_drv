# 確認用ツール (Linux のカーネルモジュール用)

`tools/` に、Linux のカーネルモジュール (`px4_drv`) が作るデバイスファイルを使って、受信の質と内蔵カードリーダーを確かめる 3 つのコマンドがあります。
機種ごとに作り分けていないため、`/dev/px4video*`、`/dev/pxmlt*video*`、`/dev/isdb2056video*`、`/dev/tornevideo*`、`/dev/dt295video*`、`/dev/xzvideo*`、`/dev/tbs5530video*` など、どのデバイスファイルにも使えます。

| ツール | 用途 |
|---|---|
| [`ptx_tei_monitor`](#ptx_tei_monitor-長時間の受信を一定間隔で集計する) | 長時間、受信しながら、一定間隔ごとに誤りの数を表示する。誤りの出る時刻を調べる、複数の受信機を比べる |
| [`ptx_chrdev_test`](#ptx_chrdev_test-1-回の受信を集計する) | 指定の秒数だけ受信して、品質を 1 回集計する。TS の保存、選局直後の調査もできる |
| [`px4card_test`](#px4card_test-内蔵カードリーダーを確かめる) | 内蔵カードリーダーの電源、カードの検出、ATR、通信の往復を確かめる |

## 準備

### ビルド

`gcc` と、カーネルモジュールの標準ヘッダー (`linux/ioctl.h` など) があれば足ります。リポジトリの直下で、次のコマンドを実行します。

```
make -C tools
```

`tools/` に、`ptx_tei_monitor`、`ptx_chrdev_test`、`px4card_test` ができます。1 つだけ作る場合は、`make -C tools ptx_chrdev_test` のように指定します。

TS の集計の自己試験 (機材は要りません) は、次のコマンドで実行できます。

```
make -C tools check
```

### 実行の条件

- `px4_drv` が読み込まれ、対象の機材のデバイスファイルがあること (`ls /dev/px4video* /dev/px4card/*` など)。
- デバイスファイルを開く権限があること。udev ルール (`etc/99-px4video.rules`) は、`px4video*`、`pxmlt5video*`、`pxmlt8video*`、`isdb2056video*`、`isdb6014video*`、`pxm1urvideo*`、`pxs1urvideo*`、`isdbt2071video*`、`tbs5530video*` を `video` グループへ、`/dev/px4card/*` を `pcscd` グループ (モード 0660) へ許可します。それ以外の `tornevideo*`、`dt295video*`、`xzvideo*` は、ルールの対象外で、root だけが開けます。他のデバイスファイルも、root で実行すれば開けます。
- 同じデバイスファイルを、録画ソフトなどが開いていないこと (1 つの受信機を、2 つのプロセスで同時には使えません)。

## 選局のオプション (`ptx_tei_monitor` と `ptx_chrdev_test` で共通)

| オプション | 内容 |
|---|---|
| `-s SYSTEM` | 方式。`T` = ISDB-T (地上波、既定)、`S` = ISDB-S (BS / CS110)、`C` = ISDB-C (CATV のトランスモジュレーション)、`J` = J.83 Annex B |
| `-c CHANNEL` | チャンネル。方式ごとの意味は、下の表のとおり |
| `-f KHZ` | `C` と `J`: 中心周波数 (kHz)。`-c` より優先する。`J` では必須 (90000〜770000) |
| `-n FREQ_NO` | `T` と `S`: `PTX_SET_CHANNEL` の `freq_no` を直接指定する。`-c` より優先する |
| `-S SLOT` | `S`: `PTX_SET_CHANNEL` の `slot` (相対 TS 番号または TSID)。既定 0 |
| `-m N` | `C`: TSMF の TS のうち、0 から数えて N 番目を受ける (0〜14)。既定 0 |
| `-L VOLT` | `S`: LNB の電源を供給する。11 または 15 (V)。終了時に止める |

方式ごとの `-c` の意味は、次のとおりです。

| 方式 | `-c` に指定する値 | 内部で使う値 |
|---|---|---|
| `T` | UHF の物理チャンネル 13〜62 | `PTX_SET_CHANNEL` の `freq_no` = チャンネル + 50 |
| `S` | トランスポンダー番号 0〜23 (BS 0〜11、CS110 12〜23) | `PTX_SET_CHANNEL` の `freq_no` = トランスポンダー番号 |
| `C` | CATV の C チャンネル 13〜63 | `PTXT_SET_PARAMS` と `PTXT_TUNE`。中心周波数は `93000 + 物理チャンネル × 6000` kHz (C13〜C22 は物理チャンネル 3〜12、C23〜C63 は 22〜62。ただし C22 は +2000 kHz) |
| `J` | (使わない。`-f` で周波数を指定する) | `PTXT_SET_PARAMS` と `PTXT_TUNE` |

選局に失敗したときは、失敗した操作と理由を表示して、終了コード 1 で終わります。主な理由は、次のとおりです。

| errno | 意味 |
|---|---|
| `EAGAIN` | ロックしなかった (アンテナ未接続、信号なし、受信条件の不一致) |
| `EINVAL` | そのデバイスが、その方式またはチャンネルに対応していない (例: PX-Q3U4 の ISDB-T のノードに `-s S` を指定した) |
| `ETIMEDOUT` | 時間切れ。モジュールの読み込み直後の、最初の選局で起きることがある。別のチャンネルで選局し直すと、受信できる場合がある |

## ptx_tei_monitor: 長時間の受信を一定間隔で集計する

### 使い方

```
ptx_tei_monitor [options] DEVICE TOTAL_SEC
```

`TOTAL_SEC` 秒 (`0` なら、Ctrl+C まで) 受信しながら、`-i` で指定した間隔 (既定 10 秒) ごとに、その間の数を 1 行で表示します。TS は保存しません。最後に、全体の合計を 1 行表示します。

| オプション | 内容 |
|---|---|
| `-i SEC` | 集計して表示する間隔 (秒、既定 10) |
| `-r` | 各行に、`PTX_GET_CNR` の生の値を `cnr_raw=` として加える |
| `-T` | 各行の先頭に、時刻 (時:分:秒) を加える |
| `-e` | 全体に誤り (TEI、連続性の飛び、同期ずれ) が 1 つでもあれば、終了コードを 1 にする |
| `-h` | 説明を表示する |

選局のオプションは、上の表のとおりです。

### 例

```
$ ./tools/ptx_tei_monitor -s T -c 21 -i 10 -r -T /dev/px4video2 30
# /dev/px4video2: ISDB-T UHF 21ch、間隔 10 秒
16:39:05 10s packets=114339 null=39 tei=0 cc=0 dup=0 sync=0 cnr_raw=60720
16:39:15 20s packets=115037 null=130 tei=0 cc=0 dup=0 sync=0 cnr_raw=63130
16:39:25 30s packets=115036 null=81 tei=0 cc=0 dup=0 sync=0 cnr_raw=62145
total packets=344412 null=250 tei=0 cc=0 dup=0 sync=0
```

各項目の意味は、次のとおりです。

| 項目 | 意味 |
|---|---|
| `packets` | 同期できた TS パケットの数 (ヌルを含む) |
| `null` | ヌルパケット (PID `0x1fff`) の数。チャンネルの放送内容や、機種の出力の詰め物で変わる |
| `tei` | `transport_error_indicator` が立ったパケットの数。復調器が訂正できなかった誤り |
| `cc` | 連続性の飛びの数 (PID ごとの連続性カウンターが、前の値の次でも、同じ値でもない回数) |
| `dup` | 前と同じカウンターの値の数。規格で 1 回までは許される再送。参考値で、誤りには数えない |
| `sync` | 同期を失った回数 (`0x47` が 188 バイト間隔から外れた回数) |

### 複数の受信機を比べる

複数の受信機を同時に受信して、誤りの出る時刻を比べるときは、受信機ごとにプロセスを起動し、`-T` で時刻を付けます。出力をファイルへ保存して、時刻で並べると、同じ時刻に複数の受信機へ誤りが出たかを調べられます。

```
$ for d in px4video2 px4video3 tornevideo1; do
>   ./tools/ptx_tei_monitor -T -c 21 -i 10 /dev/$d 600 > /tmp/$d.log &
> done; wait
$ sort -s -k1,1 /tmp/px4video2.log /tmp/px4video3.log /tmp/tornevideo1.log
```

同じ時刻に、同じ回線の複数の受信機へ誤りが出る場合は、アンテナ側の信号の乱れの可能性があります。1 台だけに出る場合は、その受信機や USB の経路を疑います。

### 終了コード

| コード | 意味 |
|---|---|
| 0 | 受信を終えた (`-e` を指定しないときは、誤りがあっても 0) |
| 1 | 選局や受信に失敗した、TS を 1 つも受信できなかった、または `-e` を指定して誤りがあった |
| 2 | 使い方の誤り |

## ptx_chrdev_test: 1 回の受信を集計する

### 使い方

```
ptx_chrdev_test [options] DEVICE SECONDS
```

選局して、`SECONDS` 秒 (1〜3600) 受信し、品質をまとめて表示します。

| オプション | 内容 |
|---|---|
| `-o FILE` | 受信した TS を、`FILE` へ保存する。読み取ったバイト列を、そのまま書く |
| `-w SEC` | 選局してから、受信を始めるまで待つ秒数 (既定 0)。選局直後の TEI と連続性エラーを調べるときに使う |
| `-p` | 連続性の飛びが多かった PID を、上位 10 個まで表示する |
| `-h` | 説明を表示する |

選局のオプションは、上の表のとおりです。

### 例

```
$ ./tools/ptx_chrdev_test -c 21 -p -o /tmp/tbs.ts /dev/tbs5530video0 10
device: /dev/tbs5530video0
target: ISDB-T UHF 21ch
tune: ok
PTX_GET_CNR: raw=2776 (値の単位は機種による。docs/tools.md を参照)
PTX_STOP_STREAMING: ret=0
received 27262976 bytes in 10.03 s (21.74 Mbps)
first data after START_STREAMING: 36 ms
TS packets 145015 (null 22030, 15.2%), TEI 1, continuity jumps 5394 (duplicates 3371), sync loss 0
error rate (non-null packets): TEI 0.0008%, continuity jumps 4.3859%
seconds with errors: 10 of 11 (second: tei/cc/sync)
  0s: 1/437/0
  1s: 0/505/0
  ...
continuity jumps by PID (top 10):
  PID 0x0111: 3545
  PID 0x0811: 644
  ...
result: ERRORS
```

- **`first data after START_STREAMING`**: 受信を始めてから、最初のデータが届くまでの時間。
- **`seconds with errors`**: 誤りのあった秒を、`秒: TEI/連続性の飛び/同期ずれ` の形で、20 秒まで表示します。受信の開始直後だけに出る誤りか、一定に出る誤りかを見分けられます。最後の行は、1 秒に満たない端数の区間です。
- **`continuity jumps by PID`** (`-p`): 飛びの多い PID。特定の PID だけが多ければ、その番組のストリームだけが欠けています。
- **`result`**: `OK` (誤りなし)、`ERRORS` (TEI、連続性の飛び、同期ずれのいずれかがある)、`FAILED` (TS を受信できなかった)。

### 選局直後の調査

`-w` で、選局してから受信を始めるまでの待ちを変えて、`-o` で保存すると、選局直後の TEI と連続性エラーが、待ち時間でどう変わるかを調べられます。

```
$ ./tools/ptx_chrdev_test -c 21 -w 0 /dev/px4video2 10    # 選局してすぐ受信を始める
$ ./tools/ptx_chrdev_test -c 21 -w 2 /dev/px4video2 10    # 2 秒待ってから受信を始める
```

### 終了コード

| コード | 意味 |
|---|---|
| 0 | 誤りなし (`result: OK`) |
| 1 | 誤りがある、または選局や受信に失敗した |
| 2 | 使い方の誤り |

## px4card_test: 内蔵カードリーダーを確かめる

### 使い方

```
px4card_test [options] [DEVICE]
```

内蔵カードリーダーのデバイスファイル (`/dev/px4card/<ID>`) に対して、次の手順を行います。

1. `PX4_CARD_POWER_ON` で、UART を初期化する。
2. `PX4_CARD_DETECT` で、カードの有無を調べる (なければ、ここで終わる)。
3. `PX4_CARD_RESET` で、カードをリセットし、ATR を受信して表示する。
4. `-b` で指定した通信速度へ切り替える (`PX4_CARD_SET_BAUDRATE`)。
5. T=1 の S ブロック (IFS 要求、`00 c1 01 fe 3e`) を送り、S ブロックの応答 (PCB `0xe1`) が返るかを、`-I` の間隔で `-t` の秒数だけ繰り返す。成功、失敗、応答時間の平均と最大を表示する。
6. `PX4_CARD_POWER_OFF` で、電源を切る。

`DEVICE` を省略すると、`/dev/px4card/` に 1 つだけあれば、それを使います。複数あるときは、一覧を表示して終わります。

| オプション | 内容 |
|---|---|
| `-t SEC` | 往復を繰り返す秒数 (既定 5)。`0` なら、電源投入と ATR までで終える |
| `-I MS` | 往復の間隔 (ミリ秒、既定 200) |
| `-b BAUD` | ATR の後に切り替える通信速度。9600、19200、38400。`0` なら、切り替えない (既定 19200) |
| `-l` | `/dev/px4card/` のデバイスファイルを一覧にして終わる |
| `-h` | 説明を表示する |

### 例

```
$ ./tools/px4card_test -l
/dev/px4card/usb-3-4
/dev/px4card/usb-3-2
/dev/px4card/ED1A5DB35C10DE62
$ ./tools/px4card_test -t 3 /dev/px4card/usb-3-2
device: /dev/px4card/usb-3-2
POWER_ON: ret=0 (errno 0)
DETECT: ret=0 (errno 0) present=1
RESET: ret=0 (errno 0)
ATR (13 bytes): 3b f0 12 00 ff 91 81 b1 7c 45 1f 03 99
  TS=0x3b (direct convention)
SET_BAUDRATE 19200: ret=0 (errno 0)
exchanges 15, failures 0, mean 78.4 ms, max 80.7 ms
POWER_OFF: ret=0 (errno 0)
result: OK
```

### 注意

- 往復の確認は、T=1 のカード (B-CAS カードなど) を想定しています。T=0 のカードでは、S ブロックの応答が返らず、往復が失敗として数えられます。
- **`pcscd` (`px4-drv-pcsc`) や録画ソフトが、同じカードリーダーを開いている間は、実行できません。** カーネルモジュールは、T=1 の連番と再送の状態を 1 つのセッションだけが持てるよう、カードリーダーを同時に 1 つのプロセスにしか開かせません。2 つ目の `open` は `EBUSY` で失敗し、`px4card_test` は、原因を表示して、終了コード 3 で終わります。
  - `pcscd` は、リーダーを検出した時点で開くため、カードの有無にかかわらず、`pcscd` が動いている間は、その `pcscd` が使うすべての内蔵カードリーダーが対象です。
  - 失敗した `open` は、`pcscd` の動作へ影響しません (その後も、`pcscd` 経由の APDU が成功します)。
  - `pcscd` を止めると、`px4card_test` を実行できます。実行を終えて、`pcscd` を起動し直すと、リーダーは再び使えます。systemd のソケット起動 (`pcscd.socket`) を使う環境では、PC/SC のクライアントが `pcscd` を自動で起動し直すため、`pcscd.socket` も止めてください (この環境には systemd がなく、確認していません)。
  - `pcscd` のリーダーを、別のプロセスで試験する場合は、`pcscd` を止めてから、`px4card_test` を実行します。`pcscd` が使っているリーダーを、`pcscd` 経由で確かめるには、`pcsc_scan` や `scriptor` を使います。

### 終了コード

| コード | 意味 |
|---|---|
| 0 | すべて成功 (`result: OK`) |
| 1 | カードが検出されない、ATR が来ない、または往復に失敗した |
| 2 | 使い方の誤り、またはデバイスファイルを開けない |
| 3 | 他のプロセス (`pcscd` など) が、カードリーダーを開いている (`EBUSY`) |

## 集計のしかた

`ptx_tei_monitor` と `ptx_chrdev_test` は、同じ集計 (`tools/ptx_tool.h`) を使います。

- **同期:** 188 バイトごとに、先頭の `0x47` を確かめます。最初の位置合わせでは、次のパケットの `0x47` も確かめてから、同期したと見なします。同期した後に `0x47` が見つからなければ、同期を失った回数 (`sync`) に 1 回数え、1 バイトずつ捨てて、探し直します (最初の位置合わせは、数えません)。
- **TEI:** `transport_error_indicator` が立ったパケットを数えます。
- **連続性の飛び (`cc`):** ペイロードのあるパケット (`adaptation_field_control` の下位ビットが 1) を、PID ごとに検査します。前の値の次でも、同じ値でもなければ、1 回数えます。
  - 同じ値は、規格で 1 回までの再送が許されるため、飛びには数えず、`dup` に別に数えます。
  - ヌルパケットと、ペイロードのないパケットは、検査しません。
  - `discontinuity_indicator` が立ったパケット (アダプテーションフィールドの長さがパケットに収まる場合に限る) は、カウンターの不連続を許し、そこから数え直します。
  - **TEI のパケットは、中身が信頼できないため、連続性の検査へ使いません。** そのため、TEI のパケットの次の同じ PID のパケットで、連続性の飛びが 1 回数えられることがあります (欠けたパケットとして扱います)。
- **ISDB-C の TSMF:** 複数の TS が多重された搬送波では、同じ PID が複数の TS に現れます。PID ごとの連続性は、単純には比較できず、`cc` が大きくなります。TSMF の TS を選んで (`-m`)、TS を取り出した後の値でも、同様に評価できないことがあります。ISDB-C では、`tei` と `sync` を中心に見てください。
- **C/N (`cnr_raw`):** `PTX_GET_CNR` が返す生の値で、dB には換算していません。値の単位は、機種ごとに異なります。同じ機種の、同じ条件の比較にだけ使ってください。取得できない機種 (`ENOSYS` を返す機種、PX-S1UD V2.0 など) では、`n/a` と表示します。

過去の記録 (各機種の別紙) の連続性エラーの数と、ここでの `cc` は、数え方が違うことがあります (旧来のツールは、同じ値の重複を飛びに数える、TEI のパケットも検査する、など)。値を比べるときは、同じツールで測り直してください。

## 動作の確認

次の環境で、3 つのツールの動作を確認しました。

- **環境:** Debian 13、カーネル 6.12.117、実機のホスト。機材は、PX-Q3U4 2 台、Sony CECH-ZD1 J 3 台、PIX-DT295 Rev.01 と Rev.02、GV-MVP/XZ、TBS 5530、PX-S1UD V2.0。
- **`ptx_tei_monitor`:** PX-Q3U4 (ISDB-T) と GV-MVP/XZ の受信、`-r`、`-T`、`-e`、Ctrl+C での終了、TBS 5530 の ISDB-C (`-s C -f 141000`)。
- **`ptx_chrdev_test`:** GV-MVP/XZ の受信と TS の保存、TBS 5530 の ISDB-T (`-p`、飛びの多い PID の表示) と ISDB-C (`-s C -c 18`)。ISDB-S は、アンテナがなく、PX-Q3U4 の ISDB-S のノードでは `EAGAIN` で失敗し、ISDB-T のノードでは `EINVAL` で失敗しました (いずれも、期待どおりのエラーの表示)。
- **`px4card_test`:** 7 つのカードリーダー (Sony CECH-ZD1 J の 3 台、PIX-DT295 の 2 台、GV-MVP/XZ、PX-Q3U4 の 1 つ目のカード) のうち、B-CAS カードを入れた 6 つで、電源投入、検出、ATR、19200 bps への切り替え、往復 15 回が、失敗 0、平均 78〜93 ms で成功しました。PX-Q3U4 の 1 つは、カードが入っておらず、`DETECT` で終了しました。
- **自己試験:** `make -C tools check` が通りました (21 項目。AddressSanitizer と UBSan でも通りました)。
- **`pcscd` が動いているときの `px4card_test`:** `pcsc-lite` 2.3.3 の `pcscd` (`px4-drv-pcsc` の IFD ハンドラーを、バンドルとして導入し、USB のホットプラグで検出) が、Sony CECH-ZD1 J の 3 台、PIX-DT295 の 2 台、GV-MVP/XZ、PX-Q3U4 の 1 つ目のカードリーダーを開いている状態で、7 つすべてのデバイスファイルへ実行しました。すべて `EBUSY` で失敗し、終了コード 3 でした。その後も、`pcscd` 経由の APDU (B-CAS の初期化コマンド、SW=9000) が成功し、`pcscd` を止めると、`px4card_test` が成功し、`pcscd` を起動し直すと、リーダーが元どおり使えました。
- **確認していないこと:** `-L` (LNB の電源) と ISDB-S の受信 (アンテナがない)、`-s J`、`-m` が 0 以外の TSMF の選択、PX-MLT 系、ISDB2056 系、ISDBT2071 系の実機、T=0 のカードでの `px4card_test`、systemd のソケット起動の環境。
