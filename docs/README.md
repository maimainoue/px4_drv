# 文書の索引

`docs/` と関連ディレクトリの文書を、読む目的ごとに分けた一覧です。
利用方法と対応機種は [README.md](../README.md)、変更時の方針は [AGENTS.md](../AGENTS.md) にあります。

日付で始まる文書は、その日付の時点で何を確認し、何を未確認として残したかを保存する記録です。
後の変更でファイルの場所や判断が変わっている場合があるため、現在の仕様は日付のない文書とコードで確認してください。

## 機種別の資料

| 文書 | 内容 |
|---|---|
| [t230c.md](t230c.md) | MyGica T230C2 (`0572:C68A`) のファームウェア、設定、ISDB-C / J.83B の制限、実機確認 |
| [t230c_linux_reference.md](t230c_linux_reference.md) | Linux 公開ドライバーから読み取った T230C の制御手順と方式の制限 |
| [t230c_requirements.md](t230c_requirements.md) | T230C 対応を始める時点の要件 (2026-10-30) |
| [siano_tuners.md](siano_tuners.md) | PX-S1UD V2.0 / PX-Q1UD (Siano RIO) の OS ごとの経路、ファームウェア、検証状況 |
| [devices/torne.md](devices/torne.md) | Sony CECH-ZD1 J (torne) の制御、カード UART、OS ごとの利用、実機試験 |
| [devices/dt295.md](devices/dt295.md) | PIXELA PIX-DT295 の仕組み (起動直後の機材、窓、機材内のポンプ、取り込みブロック)、Windows と macOS の提供方法と確認の要点、実装元と権利、実機試験 |
| [devices/xz.md](devices/xz.md) | I-O DATA GV-MVP/XZ2 の仕組み (起動直後の機材の識別、書き込みが 4 バイトまでのリンク、初期化の表、機材内のポンプ、カード)、Windows と macOS の提供方法と確認の要点、実装元と権利、実機試験、前面の最小化が成立しなかった経緯 |
| [devices/tbs5530.md](devices/tbs5530.md) | TurboSight TBS 5530 (ISDB-T) の構成 (FX2 のブリッジ、CXD6822 と内蔵チューナー)、ファームウェアの扱い、実機で分かった落とし穴 (`SET_INTERFACE`、FIFO の不整合)、実機試験 |
| [devices/hz2w.md](devices/hz2w.md) | I-O DATA GV-MVP/HZ2W (ASICEN ASV5211) の実装資料: firmware の取り出し (`fwtool/asv5211fw`)、実機で分かったこと (起動、基板の情報、I2C の応答)、メーカー製ドライバーの静的解析、未確認の事項、実装の段階 |

## 仕様

| 文書 | 内容 |
|---|---|
| [isdbc_tuning.md](isdbc_tuning.md) | PX-MLT 系の ISDB-C / J.83 Annex B の選局、チャンネル定義、Linux の拡張 ioctl |
| [../pcsc/README.md](../pcsc/README.md) | macOS / Linux 版の内蔵カードリーダー (IFD ハンドラー) の設計 |
| [../winusb/src/WinSCard_PX4/README.md](../winusb/src/WinSCard_PX4/README.md) | Windows 版の WinSCard_PX4 の設計 |

## 計画と残作業

| 文書 | 内容 |
|---|---|
| [restructure_plan.md](restructure_plan.md) | Linux / WinUSB / macOS の共通化、ログ、信号品質、ISDB-C / J.83B の計画と進捗 |
| [linux_pending_verification.md](linux_pending_verification.md) | Linux 実機での確認が残っている変更の一覧 |

## 内蔵カードリーダーの記録 (2026年7月)

[20260724_card_reader_work_index.md](20260724_card_reader_work_index.md) がこの日付の文書群の入口です。

| 文書 | 内容 |
|---|---|
| [20260724_decision_record.md](20260724_decision_record.md) | 確定した判断と、採用しなかった案 |
| [20260724_external_change_assessment.md](20260724_external_change_assessment.md) | Issue、Pull Request、派生ブランチの取り込み可否の調査 |
| [20260724_validation_record.md](20260724_validation_record.md) | 実機、コード、外部報告で確認した範囲と未確認の範囲 |
| [20260724_winusb_card_reader_implementation_history.md](20260724_winusb_card_reader_implementation_history.md) | WinUSB 版の実装が成立するまでの設計の選択 |
| [20260724_winusb_card_reader_debugging_history.md](20260724_winusb_card_reader_debugging_history.md) | WinUSB 版の初期実装で現れた症状の切り分け |
| [20260724_linux_card_reader_handover.md](20260724_linux_card_reader_handover.md) | Linux 版の実装を再開するための引き継ぎ |

## その他の記録

| 文書 | 内容 |
|---|---|
| [20260725_prepush_review.md](20260725_prepush_review.md) | WinUSB 版の大規模改修に対する push 前レビュー |
| [20260725_ringbuffer_purge_deadlock.md](20260725_ringbuffer_purge_deadlock.md) | 選局の繰り返しで TS リングバッファのパージが受信を止めた不具合 |
| [20261014_linux_card_reader_implementation_history.md](20261014_linux_card_reader_implementation_history.md) | Linux 版の内蔵カードリーダー対応の取り込みと修正 |
| [20261014_linux_card_reader_validation.md](20261014_linux_card_reader_validation.md) | Linux 版の内蔵カードリーダーの実機確認 |
| [20261018_isdbc_validation.md](20261018_isdbc_validation.md) | PX-MLT 系の ISDB-C 対応の取り込みと実機確認 |
| [20261018_official_windows_driver_analysis.md](20261018_official_windows_driver_analysis.md) | PX-Q3U4 の公式 Windows ドライバーの静的解析 |
| [20261113_common_fixes.md](20261113_common_fixes.md) | `ptx_chrdev` の cdev の寿命 (KASAN の use-after-free) と、torne の制御転送のバッファの修正 |
| [20261119_linux_userland_host.md](20261119_linux_userland_host.md) | macOS 版のホスト (`libusb`) を Linux でもビルドして動かす構成 (`PX4_USERLAND` マクロ) と、DT295 での確認結果 |
| [20261121_linux_other_kernels_validation.md](20261121_linux_other_kernels_validation.md) | Linux の別カーネル版 (Debian 12 のカーネル 6.1、Ubuntu 24.04 のカーネル 6.8) での検証 (QEMU の仮想マシンへ 10 台を USB パススルー。ビルド、DKMS、開始と停止、`kill -9`、`unbind`、`rmmod` と `insmod`、10 機材の 10 分の同時受信) |
| [tools.md](tools.md) | 確認用ツール (`tools/`) の使い方: `ptx_tei_monitor` (長時間の受信を一定間隔で集計)、`ptx_chrdev_test` (1 回の受信の集計、TS の保存)、`px4card_test` (内蔵カードリーダーの試験)。選局のオプション、集計の仕様、終了コード |
