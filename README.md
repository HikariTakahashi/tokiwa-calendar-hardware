# 概要

「部屋を丸ごとリマインダーにする」タスク管理に特化したカレンダーアプリ、『TokiWa Calendar』のレポジトリです。
従来のタスク管理ツールが抱える「通知の見逃し」「タスクの後回し」という根源的な課題に対し、ソフトウェアとハードウェアの融合で解決する新しいタスク管理システムです。デジタル上のタスクを現実世界の「光・音・モノの動き」に変換し、ユーザーの物理環境に直接働きかけることで、確実な行動を促します。
『ToKiWa calendar』は、ソフトウェアによる「認知」のサポートと、ハードウェアによる物理的な「行動喚起」を組み合わせ、“わかっていても行動できない”という課題（意図と行動の乖離）を解消します。タスク管理に「環境そのものをデザインする」という新たな選択肢を提示します。
## 関連URL
YouTube動画:
https://www.youtube.com/watch?v=OgA2TAeWcS0 <br>
URL:
https://www.tokiwa-calendar.com/welcome <br>
# 導入準備
## 前提

動作には以上の~~四つ~~三つのレポジトリのインストールが必要です

**フロントエンド**:https://github.com/HikariTakahashi/tokiwa-calendar-frontend <br>
**バックエンド**:https://github.com/HikariTakahashi/tokiwa-calendar-backend <br>
**ハードウェア**:https://github.com/HikariTakahashi/tokiwa-calendar-hardware  ← いまここ <br>
~~**DiscordBot**~~:https://github.com/HikariTakahashi/tokiwa-calendar-discordbot

## 起動準備(フロントエンド)

1. レポジトリのクローン

```bash
git clone https://github.com/HikariTakahashi/simple-calendar-frontend.git
```

2. 必要なもののインストール

```bash
// フロントエンドのプロジェクトに移動(いつもの開き方でOK)
cd simple-calendar-frontend

// Node.jsのパッケージ管理システムをインストール
npm install
```

3. .env ファイルの書き込み
   .env ファイルの中身を担当者から貰ってください
   注:アップデート等により.env ファイルは頻繁に変更されるため、こまめに確認してください。

## 起動準備(バックエンド)

1. レポジトリのクローン

```bash
git clone https://github.com/HikariTakahashi/tokiwa-calendar-backend.git
```

2. バックエンドのプロジェクトに移動(個人のいつもの開き方で OK)

```bash
cd tokiwa-calendar-backend
```

3. .env ファイルの書き込み
   .env ファイルの中身を担当者から貰ってください
   注:アップデート等により.env ファイルは頻繁に変更されるため、こまめに確認してください。

## 起動準備(ハードウェア)
1. PlatformIO IDEの導入
VSCode の「拡張機能」から PlatformIO IDE を検索してインストール
2. レポジトリのクローン
```bash
git clone https://github.com/HikariTakahashi/tokiwa-calendar-hardware.git
```
3. ハードウェアのプロジェクトに移動(個人のいつもの開き方でOK)
```bash
cd tokiwa-calendar-hardware
```
3. PCにESP32C6を接続
4. ファームウェアを書き込む
　コンパイルボタンを押す
5. 外部から操作したい端末でアクセスポイントに接続
```bash 
http://192.168.4.1/
```
3. ssidとパスワードを入力しwifiに接続
　シリアルモニターを確認
　```bash
　http://<シリアルモニターに表示されたIPアドレス>/control
```

## 開発サーバーの起動

1. バックエンド起動（Go 言語）

```bash
// バックエンドのターミナルで実行
go run --tags=local .
```

注: 以前は `go run main.go` でしたが、ファイル分割により `go run .`に変更されました。プロジェクト内のすべての .go ファイルがビルド対象になります。
注 2: `go run .`からさらに`go run --tags=local .`に変更されました。

2. フロントエンド起動（Nuxt.js）

```bash
// フロントエンドのターミナルで実行
npm run dev
```

3. 開発用のサーバーにアクセス



