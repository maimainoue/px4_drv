# 2026-11-13 共通の修正 (ptx_chrdev の cdev の寿命、torne の制御転送のバッファ)

PIX-DT295 の試作で、デバッグ用のカーネルと Linux 6.12 の実機を使って見つかった、機種に共通する 2 件の修正の記録です。
PIX-DT295 自体の対応は、この記録の対象外です。

## 検証環境

- 仮想マシン: Proxmox 上の Debian 13 (カーネル 6.12.116)。USB 機材は仮想マシンへパススルーしました。
- デバッグ用カーネル: 同じ版のカーネル (Debian の `linux-source-6.12`) に、lockdep (`PROVE_LOCKING`)、`DEBUG_ATOMIC_SLEEP`、`DEBUG_MUTEXES`、`DEBUG_SPINLOCK`、KASAN (generic)、`DEBUG_KMEMLEAK`、UBSAN を足して構築しました。
- 通常のカーネル: Debian 13 の `linux-image-amd64` (6.12.116+deb13-amd64、デバッグ無し)。

## ptx_chrdev: 開いたままのファイルを閉じると解放済みの cdev を参照する

### 現象

受信中にデバイスを取り外す (unbind、USB 切断) と、開いたままだったプロセスが終了するときに、KASAN が `slab-use-after-free in cdev_put` を報告しました。
解放された領域は、`ptx_chrdev_context_add_group()` で確保した `struct ptx_chrdev_group` です。

### 原因

`struct cdev` を `ptx_chrdev_group` へ埋め込んでいました。
group は最後の `close` の `ptx_chrdev_release()` で、参照が 0 になって `kfree()` されます。
一方、VFS は `release` の後に `cdev_put()` で cdev の kobject を参照するため、group と一緒に解放された cdev を読み書きします。
取り外しの前に閉じた場合は、`cdev_del()` が先に済んでいないので問題が現れません。

`px4_usb_disconnect()` は、開いているファイルがすべて閉じるまで待ちます。
このため、取り外しの処理の後で最後のファイルが閉じられる順序は、利用者のプロセスが長く開いていれば普通に起こります。

### 修正

cdev を `cdev_alloc()` で確保して、group からはポインターで参照する形に変えました (`driver/linux/ptx_chrdev.c`、`ptx_chrdev.h`)。
kobject の参照が切れたとき (最後の `cdev_put()`) に、cdev が解放されます。
`cdev_add()` が失敗したときは、`kobject_put()` で解放します。

### 確認

- 再現手順: 別のプロセスがデバイスファイルを開いたまま、USB インターフェースを unbind し、そのプロセスを終了させます。
  自分で開いたまま unbind すると、`px4_usb_disconnect()` が閉じるのを待つため、自己待ちになります。
- KASAN の報告は 1 起動に 1 回だけなので、起動直後の 1 回目の結果で比べました。
  修正前は 1 回目で再現しました。修正後は、起動直後の 1 回目と、続く 3 周で報告が出ていません。
- 受信中の unbind と再接続の繰り返し (数十周) でも、lockdep、KASAN、UBSAN の報告と、kmemleak の検出はありませんでした。
- 上の確認の実機は PIX-DT295 だけです。`ptx_chrdev` は他の機種も共通で使うため、PX-W3U4 など他機種の実機での確認は、次の追加確認の範囲を除いて未実施です (一覧は [linux_pending_verification.md](linux_pending_verification.md))。

#### 追加確認 (2026-11-21、デバッグ無しのカーネル)

Debian 13 (カーネル 6.12.117、デバッグ用の設定なし) の実機のホスト (仮想マシンを使わない) で、PX-Q3U4 2 台、Sony CECH-ZD1 J 3 台、PX-S1UD V2.0 1 台について確認しました。
KASAN などの検査がないカーネルのため、確認できたのは、警告と oops が出ないことと、再接続後に受信できることまでです。

- 別のプロセスがデバイスファイルを開いたまま unbind すると、unbind は閉じるまで待たされ、そのプロセスを強制終了した 1 秒後に完了しました。
  再接続後に、再び受信できました。
  PX-Q3U4 (`3-8`)、CECH-ZD1 J (`3-10`)、PX-S1UD V2.0 (`3-11`) で、それぞれ 1 回ずつ実施しました。`dmesg` に警告と oops はありませんでした。
- CECH-ZD1 J では、読み出し中の unbind で、読み手の `read` が `EIO` で返り、プロセスが終了すると unbind が完了することも確認しました。
- PX-Q3U4 (ISDB-T の 4 ノード)、CECH-ZD1 J 2 台、PX-S1UD V2.0 の 7 本の ISDB-T を同時に 120 秒受信しました。2 回目は、全チューナーで同期ずれ 0、TEI 0、連続性エラー 0、約 17.26 Mbps でした。
  1 回目は、2 台目の PX-Q3U4 の 2 ノードだけに、TEI が約 6,000 件、連続性エラーが 65 件出ました。再現せず、原因は未確認です。
- 確認できていないのは、PX-Q3U4 の ISDB-S のノード (BS/CS アンテナなし)、B-CAS カード、30 分以上の受信、USB の物理的な抜去、PX-MLT 系、ISDB2056 系、ISDBT2071 系、T230C です。

## torne: 制御転送へスタック上のバッファを渡して probe が失敗する

### 現象

Linux 6.12 で、Sony CECH-ZD1 J (torne、`054c:04b2`) の probe が `-11` (`-EAGAIN`) で失敗しました。

### 原因

共有コアと `card_read32()` / `card_write32()` は、スタック上の配列を `usb_control_msg()` の転送バッファとして渡しています。
転送バッファは DMA の対象で、スタックの領域を渡すと `usb_hcd_map_urb_for_dma()` が拒否します。

### 修正

`torne_control_timeout()` (`driver/linux/torne_device.c`) が、64 バイト以下という既存の制限の範囲で、ヒープの一時領域を経由して転送するようにしました。
読み出しでは、転送した長さの分だけ呼び出し元のバッファへ戻します。

### 確認

通常のカーネル (6.12.116) の仮想マシンで、torne をパススルーして、UHF 27ch を受信しました。
18.45 Mbps、TEI 0、連続性エラー 0、C/N 24 dB でした。
2026-11-21 に、カーネル 6.12.117 の実機のホスト (仮想マシンを使わない) で、CECH-ZD1 J 3 台の probe が通ることを確認しました。
2 台に信号があり、UHF 21ch を 120 秒受信して、約 17.26 Mbps、同期ずれ 0、TEI 0、連続性エラー 0 でした。
PX-Q3U4 の 4 ノードと PX-S1UD V2.0 との同時受信も、同じ条件で TEI 0、連続性エラー 0 でした (残りの 1 台は、アンテナがつながっておらず、全パケットが NULL でした)。
他のカーネル版と、30 分以上の受信は未確認です。
