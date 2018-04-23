# how to build
vcpkgに依存パッケージのビルドを任せてAlembic込みでビルドする手順を作りました。

* 20180424 更新: Alembicがvcpkgでビルドできるようになったので手順を省略

# 手順

## vcpkgをsetupする
依存ライブラリのソースダウンロードからビルド、bin, lib, includeへの格納を自動実行できる。

* [vcpkg](https://github.com/Microsoft/vcpkg)をcloneする。
* bootstrap-vcpkg.batを実行してvcpkgをビルドする。

## vcpkgで依存ライブラリをインストールする

コマンドラインの例

```powershell
VCPKG_DIR> .\vcpkg.exe install alembic:x64-windows pybind11:x64-windows
```

``VCPKG_DIR/installed``以下にビルド成果物が格納されるので以降の手順でこれを利用します。

```
vcpkg # ここをVCPKG_DIRにする
    installed
        x64-windows
            bin
                Alembic.dll
                python36.dll
                など
            include
                pybind11
            lib
```

環境変数``VCPKG_DIR``にvcpkgのトップレベルを設定してください。

## DirectX SDK

* https://www.microsoft.com/en-us/download/details.aspx?id=6812

``DXSDK_Jun10.exe``

### インストールがエラーになる場合
* https://support.microsoft.com/en-us/help/2728613/s1023-error-when-you-install-the-directx-sdk-june-2010

### インストール後に環境変数``DXSDK_DIR``を設定してください。

``C:/Program Files (x86)/Microsoft DirectX SDK (June 2010)``

### DXSDKのヘッダの修正が必要です。

参考

* https://gist.github.com/t-mat/1540248#d3dx9coreh

を修正

## Python3.6

64ビット版をインストールしてください(CMake実行時にPythonインタープリタが見つからないエラー)

## MikuMikuDance_x64フォルダの準備
64bit版のMMDをMikuMikuDance_x64フォルダに展開します。

```
mmdbridge
    MikuMikuDance_x64
        MikuMikuDance.exe
        Data
            MMDExport.h
            MMDExport.lib
など
```

## mmdbridgeのビルド
``cmake_vs2017_64.bat``を実行して生成された``build_vs2017_64/mmdbridge.sln``をビルドしてください。``INSTALL``をビルドすると実行に必要なdllとpyをMikuMikuDance_x64にコピーします。

## mmdbridgeのデバッグ実行
INSTALLプロジェクトのプロパティ - デバッグ - コマンド - 参照で``MikuMikuDance_x64/MikuMikuDance.exe``を指定して``F5``実行するとデバッガをアタッチできます。デバッグビルドには、``/Z7``コンパイルオプションでpdbを埋め込んであります。

