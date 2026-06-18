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
- 🐘 **大容量 `.db3` 対応（数GB）** — ファイル全体を読み込まず、SQLite のページを**必要な分だけ遅延読み込み**するため、2〜3GB 級の bag もメモリを使い切らずに開けます
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

## GitHub Pages での公開 / Deploy to GitHub Pages

本ツールは静的サイトとして **GitHub Pages（github.io）で動作します**。
`SharedArrayBuffer` を必要としない実装のため、追加のヘッダ設定なしでそのままホスティングできます。

公開URL（このリポジトリの場合）:

```
https://k-kaitojp.github.io/rosbag_pointcloud_viewer/
```

### 初回の有効化手順

1. このブランチを `main` にマージ（または GitHub の **Actions → Deploy to GitHub Pages → Run workflow** で手動実行）
2. リポジトリの **Settings → Pages → Build and deployment → Source** を **「GitHub Actions」** に設定
3. `.github/workflows/deploy.yml` が自動で `npm ci → npm run build → dist/ を Pages へデプロイ` します
4. 数十秒〜数分後に上記URLで公開されます

以降、`main` へ push するたびに自動で再デプロイされます。
独自ドメインやユーザーサイト（`<user>.github.io`）でもそのまま動作します
（アセットは相対パスで出力されるため `base` 設定の変更は不要です）。

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
  ├─ .db3  → 自作の遅延 SQLite リーダーで topics / messages を読む
  │           （File.slice() で必要なページだけ取得 → 数GBでも低メモリ）
  └─ .mcap → @mcap/core でストリーム読み出し（@mcap/support で解凍）
        ↓
  serialized message (CDR)
        ↓
  src/cdr.js         … RTPS/CDR エンコード(4byteカプセル化, アラインメント)を解釈
  src/pointcloud2.js … sensor_msgs/msg/PointCloud2 をデコードし、座標/色/スカラーを抽出
        ↓
  src/viewer.js      … three.js で BufferGeometry の Points として描画
```

`.db3`（SQLite）は **ファイル全体を読み込みません**。SQLite ファイルフォーマット
（ページ / B-tree / レコード / オーバーフローページ）を直接パースし、`File.slice()`
で必要なページだけを取り出します。トピック選択時に `messages` テーブルの B-tree を
一度だけ走査してフレーム索引を作り（巨大な BLOB を含むオーバーフローページはスキップ）、
表示するフレームの点群データだけをその都度取得します。これにより 2〜3GB 級の bag でも
メモリ消費は数MB程度に収まります。

主要ファイル:

| ファイル | 役割 |
| --- | --- |
| `src/cdr.js` | 最小 CDR リーダー（ROS2 シリアライズ形式） |
| `src/pointcloud2.js` | `PointCloud2` デコードと点群抽出 |
| `src/sqlite/reader.js` | 遅延読み込み SQLite リーダー（ページ / B-tree / オーバーフロー） |
| `src/sources/db3.js` | `.db3`（rosbag2 スキーマ）読み出し |
| `src/sources/mcap.js` | `.mcap` 読み出し |
| `src/viewer.js` | three.js による 3D 表示・カラーリング |
| `src/main.js` | UI とアプリ全体の制御 |

## テスト / Test

```bash
npm test
```

- `test/decode.test.mjs` — 合成 `PointCloud2` メッセージのバイトレベルなデコード検証
- `test/sqlite.test.mjs` — sql.js が生成した rosbag2 スキーマの DB に対し、遅延 SQLite リーダーがトピック一覧・フレーム索引・BLOB 取得（オーバーフローページ含む）を正しく行うか検証

## トラブルシューティング / Troubleshooting

**`Could not read ... NotReadableError` / 「ファイルを読み取れませんでした」**

- ファイルが **選択後に移動・上書き** されると発生します（記録中の bag など）。読み込み直してください。
- **OneDrive / Google Drive / ネットワークドライブ（SMB）** 上のファイルはブラウザが読めないことがあります。**ローカルフォルダにコピー**してから選択してください。
- `.db3` は遅延読み込みのためサイズ上限は実質ありませんが、ファイルが選択後に移動・上書きされると発生します。ローカルにコピーして再選択してください。
- `.mcap` は現状ファイル全体をメモリに読み込むため、巨大な `.mcap` では発生し得ます（`.db3` は影響を受けません）。

## 制限事項 / Limitations

- **`.db3`（SQLite）**: ページ単位の遅延読み込みのため、数GB級の大容量 bag でも低メモリで開けます。
- **`.mcap`**: 現状はファイル全体をメモリに読み込みます（大容量 mcap 非対応）。大容量で mcap を使う場合は `.db3` での記録を推奨します。
- `.mcap` の圧縮チャンクは解凍 WASM の読み込みが必要です（`@mcap/support`）。
- 表示は単一トピックの単一フレームごとです（複数トピックの同時重ね合わせは未対応）。
- 1 フレームの点群は `position`/`color` 属性として GPU に載せるため、極端に巨大な単一クラウド（数千万点〜）は描画が重くなる場合があります。

## ライセンス / License

MIT
