# 26K66_CapstanDrive

キャプスタン駆動の1関節を、PCからUSBでODrive S1を直接制御し、MCPサーバーとしてLLM（Claude Codeなど）から操作するためのコードです。

## ハードウェア

- ODrive S1（ファームウェア 0.6.11）、24 V電源 LRS-350-24、15 Aヒューズ、ブレーキ抵抗 2 Ω
- モーター: EaglePower LA8308 KV90（極対数20）。位置はODrive基板上のエンコーダで測る
- 減速: ピニオンから出力ドラムへ 8:1（キャプスタン）
- 接続: PC と ODrive を USB-C で直結。当初のTeensy 4.1 + CAN構成は今は使っていない

## 機構の測定値（2026-11-25）

- **向き**: モーターの+方向がドラムの「下」（`config.py` の `MOTOR_DIR_UP = -1`）
- **可動範囲**: 端に当たり始めるまで約2.43〜2.47モーター回転（設定値は2.47）。端は柔らかく、手で強く押し込むと2.62回転まで動く
- **負荷**: ほぼ摩擦だけ（ゆっくり動かすのに0.3〜1 A）。モーターを切ってもドラムは落ちない
- **原点**: ODriveは起動のたびに位置を0にリセットする。そのため起動後に原点合わせ（`home`）が必要

## ODriveに保存した設定の変更（2026-11-25）

| 項目 | 変更前 | 変更後 |
|---|---|---|
| `axis0.controller.config.vel_limit` | 26 turns/s | 8 turns/s |
| `axis0.config.motor.current_soft_max` | 20 A | 10 A |
| `axis0.config.motor.torque_constant` | 0.0766（KV 108） | 0.0919（KV 90） |

変更前と変更後の設定は `odrive_config/` に保存してある。`odrivetool restore-config <ファイル>` で戻せる。

## ファイル

| ファイル | 内容 |
|---|---|
| `config.py` | 機構の値と安全上の上限（ギア比、向き、可動範囲、速度、電流、ゲイン、原点合わせ） |
| `joint.py` | 制御と安全装置の本体。角度は「下端からのドラム角度」で扱う |
| `server.py` | MCPサーバー（stdio） |
| `measure_range.py` | 手でドラムを動かして、向きと可動範囲を測る。モーターは動かさない |
| `tune_gains.py` | ゲイン調整。**モーターが動く**ので `--i-am-watching` が必要 |
| `tests/sim_test.py` | 実機なしで `joint.py` を検証するシミュレーションテスト |
| `odrive_config/` | ODriveの設定のバックアップ |

## セットアップ

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# USBのアクセス権（Linux、最初の1回だけ）
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Claude Code に登録
claude mcp add capstan -- "$PWD/.venv/bin/python" "$PWD/server.py"

# 実機なしのテスト
.venv/bin/python tests/sim_test.py
```

## MCPツール

角度は下端からのドラム角度 [deg] で、上が+。指令できるのは 5.4〜105.8° の範囲。

| ツール | 動作 |
|---|---|
| `status` | 状態を読む（動かない） |
| `home` | 下端を探して原点にする（ゆっくり下へ動く） |
| `set_bottom_here` | 手で下端に当てた位置を原点にする（`home` の代わり） |
| `enable` / `disable` | モーターON（その場で保持）/ OFF（すぐ脱力） |
| `move_to` / `move_by` | 絶対角度 / 相対角度で動かす。止まるまで待ち、最終誤差を返す |
| `clear_errors` | ODriveのエラーを解除する |
| `release_usb` | USBを手放す（Web GUIや `odrivetool` を使うとき） |

よくある流れ: `status` → （原点がなければ）`home` → `enable` → `move_to` → `disable`

## 安全設計

- 速度は最大45°/s、電流は最大6 A。いずれも実行時にだけ設定し、ODriveには保存しない。ODriveに保存した8 turns/s・10 Aは最後の歯止め
- 範囲外の目標は、動かす前に拒否する
- 監視スレッド（約50 Hz）が、ODriveのエラー、閉ループからの離脱、可動範囲外、速度超過、ON直後のずれを見つけたら、すぐ脱力させる
- ODriveのウォッチドッグ: サーバーが止まると0.5秒で自動的に脱力する（実機で0.48秒を確認）
- 原点はODriveの起動ごとに無効になり、原点合わせをするまで動かせない
- 原点合わせ（リミットスイッチ不要）: 13.5°/s、最大2 Aで下端に当たるまで下げ、止まったらその位置を原点にして脱力する。逆向きに動いた、3回転以内に端が見つからない、速度が出すぎた、のいずれかで中止する
- ODriveに元からある Homing 機能はリミットスイッチが前提。下端にスイッチを付ければ、そちらに切り替えられる

## 注意

- MCPサーバーがUSBを使っている間は、Web GUIからつながらない（`release_usb` で解放する）
- 元のTeensy用サンプルコードは向きの前提が逆（+が上）なので使わない。下端に押し付けてしまう
- モーターを動かすときは、必ず人が見ていて、24 V電源をすぐ切れる状態で

## 状況（2026-11-25）

- 実機で確認済み: 保持、ウォッチドッグによる停止、±5°・30°・85°への移動、範囲外指令の拒否
- 未実施: `home` の実機テスト、`tune_gains.py` によるゲイン調整。今のゲインはODriveの初期値で、止まる位置の誤差は約0.4〜0.5°
