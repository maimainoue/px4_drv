# VIXS XCode6 搭載 STB の内蔵 ACAS/B-CAS リーダー

## 安全上の注意

この機能は実験的なものであり、確認できた範囲は限られている。

本機の内蔵カードリーダーは、STB 自身の `/usr/bin/card`・`/usr/bin/adsb` が常時使っているハードウェアと同じ物理カード・同じ UART を共有する。T=1 の送受信シーケンス(N(S)/N(R))は `/dev/shm/smartcard` のグローバルな `tx_seq`/`rx_seq` で追跡されており、この機能からの `ResetCard()` はそのグローバル状態を直接書き換える。確認できたのは「`card`/`adsb` が稼働中(ただし能動的な通信をしていない状態)での RESET・APDU 往復が安全だったこと」までで、**STB がライブ視聴や録画で実際にカードを使っている最中**に同時アクセスした場合にプロトコルが衝突しないかは未確認である。この条件での利用は利用者自身の判断に委ねる。

## 対応範囲

I-O DATA HVT-4KBC、SHARP 4S-C00AS1 など、VIXS XCode6 SoC を搭載する STB の内蔵 ACAS/B-CAS カードリーダーが対象になる。両機種はカード制御バイナリ(`/usr/bin/card`、`/usr/vixs/lib/libsmartcard_api.so`)が MD5 完全一致であり、OEM・ファームウェアビルド日付が異なってもスマートカード制御プロトコルは共通と確認できた。

この機種(`XCODE_CARD`)はチューナーを持たず、`Receiver` を公開しない。USB デバイスでもないため、`DriverHost_PX4.ini` の `VendorId`/`ProductId` ではなく、固定デバイスパス(`Config` セクションの `DevicePath`、既定値 `/dev/vixs/xcodedrv`)で識別する。

対応は Linux のユーザーランド版(`PX4_USERLAND` マクロ、[20261119_linux_userland_host.md](../20261119_linux_userland_host.md))限定で、Windows(WinUSB)・macOS 実機・Linux カーネルモジュール(`driver/linux/`)には対象の内蔵デバイスが存在しないため提供しない。

## プロトコル

`/dev/vixs/xcodedrv` への `ioctl(fd, 0x25, &ctrl)` で、カード制御・チューナー・デコーダー・HDMI・音声を多重化している。カード制御は次の 271 バイト構造体(`host/devices/xcode_card_device.hpp` の `ScardCtrl`)で行う。

```c
struct scard_ctrl {
    uint32_t cmd;           // 1=INIT, 2=RESET, 4=生T=1ブロック送受信, 5=GET_ATR
    uint8_t  data[258];     // ペイロード(送受信共用)
    uint32_t data_len;      // 長さ(入出力共用)
    uint16_t timeout_ms;
    uint16_t reserved;
    uint8_t  cold_warm;     // RESET時に1(実証済みの値)
} __attribute__((packed));
```

- `cmd=5`(GET_ATR)は、`/usr/bin/card` が既にキャッシュしている ATR を破壊的操作なしに読み取れる。カード検出(`DetectCard()`)はこれだけを判定材料にする。MMIO オフセット `0x1034` の直接読み(bit `0x100`)はカード未挿入時にも立っており、判定には使わない。
- `cmd=2`(RESET)は、カード未挿入状態で発行すると実機が再起動した事例があるため、`DetectCard()` でカード有りを確認した後にしか呼ばない。カード挿入状態での RESET は実機で安全と確認できている。
- `cmd=4` は「T=1 ブロックを 1 個送って応答を 1 個受け取る」同期トランザクションで、I/R/S ブロックいずれもこの経路を通る。全機種共通の `host/core/smart_card.cpp`(`SmartCard`)が `SendBlock()`/`ReceiveBlock()` として分けて呼ぶ 2 ステップを、`XcodeCardDevice::WriteCardData()` がこの 1 回の ioctl にまとめて処理し、応答を内部バッファへ保持して `IsCardDataReady()`/`ReadCardData()` が読み出す。

出典は、`/usr/bin/card`・`libsmartcard_api.so` の逆アセンブル(`.dynsym` に残る `scard_*`/`xcode_*` シンボルからの解析)で、公開資料・ベンダーのソースコードは参照していない。

## 実装

`host/devices/xcode_card_device.{hpp,cpp}` が `DeviceBase`(`CardDevice` 相当のバイト単位 UART I/F)を実装する。`CardDevice` の抽象化は他機種(torne、GV-MVP/XZ2 など)と共通で、ATR 解析・T=1 の再送・絶対期限は `SmartCard` が担う。この機種固有層が持つのは、`ioctl` の呼び出しと内部バッファの管理だけになる。

USB VID/PID を持たないため、`DeviceManager`(`macos/src/DriverHost_PX4/device_manager.cpp`、ユーザーランド版 Linux ビルドもこれを共有)の通常の探索(`Search()`)では見つからない。コンストラクタで `DeviceType::XCODE_CARD` の定義だけを別に集め、`Config` の `DevicePath` を使って 1 回だけ `Add()` する。`DeviceBase` のコンストラクタ(`macos/src/DriverHost_PX4/device_base.cpp`)も、`DeviceInterfaceGUID` が全ゼロの定義では USB 関連の処理(`libusb_open` 等)を行わず、カードリーダー名の設定だけで返る分岐を持つ。

## 未確認の事項

- ライブ視聴・録画で `/usr/bin/adsb` が実際にカードを使っている最中の、この機能からの同時アクセス。
- `cmd=4` を使った S ブロック(IFS 要求・RESYNCH)の往復。実機で確認できているのは APDU(I ブロック)の往復のみで、S ブロックは同じ ioctl 経路を通る設計上の前提に基づく。
- SHARP 4S-C00AS1 以外の VIXS XCode6 搭載機種での動作。
