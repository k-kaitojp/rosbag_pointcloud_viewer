# rosbag_pointcloud_viewer

ブラウザ上で動作する **ROS2 (Humble) rosbag の `sensor_msgs/msg/PointCloud2` ビューア** です。
記録済み rosbag（`.db3` / `.mcap`）をドラッグ&ドロップするだけで、点群を 3D 表示・フレーム送り・再生できます。
**すべてブラウザ内で完結**し、点群データはサーバーへ送信されません（オフライン利用可）。

> A browser-based viewer for `sensor_msgs/msg/PointCloud2` topics recorded in ROS2 (Humble) rosbags.
> Everything runs client-side — no data leaves your machine.

## 特長 / Features

- 📦 **ROS2 Humble の rosbag に対応**
  - `.db3`（SQLite3, Humble のデフォルトストレージ）
  - `.mcap`（無圧縮 / lz4 / zstd 圧縮チャンク対応）
- 🧩 **CDR デシリアライズを内蔵** — `PointCloud2` を直接デコード（外部 ROS 環境不要）
- 🎨 **カラーリング切り替え** — 高さ(Z) / X / Y 軸グラデーション、埋め込み RGB、`intensity` など任意フィールド、単色
- ▶️ **タイムライン再生** — スライダーでフレーム送り、再生 / 一時停止
- 🖱️ **3D 操作** — 回転 / ズーム / パン（Z 軸が上の ROS 座標系）
- 🔒 **完全クライアントサイド** — 点群はアップロードされません

## 必要環境 / Requirements

- Node.js 18 以上（開発・ビルド時のみ）
- WebGL 対応のモダンブラウザ（Chrome / Edge / Firefox / Safari）

## セットアップ / Getting Started

```bash
# 依存関係のインストール
npm install

# 開発サーバーを起動（http://localhost:5173）
npm run dev

# 本番ビルド（dist/ に出力。静的ホスティングにそのまま配置可）
npm run build
npm run preview
```

ブラウザで開いたら、画面右上の領域に rosbag ファイルをドラッグ&ドロップ
（またはクリックして選択）してください。

## 使い方 / Usage

1. **ファイルを開く**: `.db3` または `.mcap` をドロップ
2. **トピックを選択**: 左サイドバーに `PointCloud2` トピック一覧（メッセージ数付き）が表示されます
3. **表示を調整**:
   - **Color by**: カラーリング方法（Height(Z) / RGB / intensity など）
   - **Point size**: 点のサイズ
4. **フレーム送り / 再生**: タイムラインスライダー、または ▶ Play ボタン

> **`.db3` のファイル選択について**: ROS2 の rosbag はフォルダ内に `metadata.yaml` と
> `<bag>_0.db3` を含みますが、本ツールはトピック型・メッセージを `.db3` から直接読み取るため、
> **`.db3` ファイル単体**を選択すれば動作します。

## 対応している点群フィールド / Supported fields

- 必須: `x`, `y`, `z`（`FLOAT32` / `FLOAT64` ほか各 `PointField` データ型に対応）
- 色: `rgb` / `rgba`（PCL 慣習のパック表現 `0x00RRGGBB` / `0xAARRGGBB`）
- スカラー: `intensity`, `ring` など数値フィールドはカラーマップ表示の対象になります
- `is_bigendian`、`point_step`、`row_step` を尊重してデコードします
- 非有限値（NaN/Inf）の点は描画から除外されます

## 仕組み / How it works

```
ファイル選択
  ├─ .db3  → sql.js (WASM SQLite) で topics / messages テーブルを読む
  └─ .mcap → @mcap/core でストリーム読み出し（@mcap/support で解凍）
        ↓
  serialized message (CDR)
        ↓
  src/cdr.js         … RTPS/CDR エンコード(4byteカプセル化, アラインメント)を解釈
  src/pointcloud2.js … sensor_msgs/msg/PointCloud2 をデコードし、座標/色/スカラーを抽出
        ↓
  src/viewer.js      … three.js で BufferGeometry の Points として描画
```

主要ファイル:

| ファイル | 役割 |
| --- | --- |
| `src/cdr.js` | 最小 CDR リーダー（ROS2 シリアライズ形式） |
| `src/pointcloud2.js` | `PointCloud2` デコードと点群抽出 |
| `src/sources/db3.js` | `.db3`（SQLite3）読み出し |
| `src/sources/mcap.js` | `.mcap` 読み出し |
| `src/viewer.js` | three.js による 3D 表示・カラーリング |
| `src/main.js` | UI とアプリ全体の制御 |

## テスト / Test

合成した `PointCloud2` メッセージをデコードしてバイトレベルの正しさを検証します。

```bash
npm test
```

## 制限事項 / Limitations

- 大きな rosbag はファイル全体をメモリに読み込みます（ブラウザのメモリに依存）。
- `.mcap` の圧縮チャンクは解凍 WASM の読み込みが必要です（`@mcap/support`）。
- 表示は単一トピックの単一フレームごとです（複数トピックの同時重ね合わせは未対応）。

## ライセンス / License

MIT
