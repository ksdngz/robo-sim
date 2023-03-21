# robo-sim

Welcome to the robo-sim wiki!
### リンク
- [github/mujoco](https://github.com/deepmind/mujoco)
- [mujoco documentation](https://mujoco.readthedocs.io/en/latest/overview.html)
- [mujocopy bootcamp](https://pab47.github.io/mujocopy.html)
  - mujoco 2.2.1が必要。手順に沿って、`run_linux`を叩けば良い。（中身はmakeとバイナリ実行）  
[リンク](https://github.com/deepmind/mujoco/releases/download/2.2.1/mujoco-2.2.1-linux-x86_64.tar.gz)
から解凍後`~/.mujoco`以下に配置。下記コマンドで解凍する。
    ```
     tar -zxvf ./mujoco-2.2.1-linux-x86_64.tar.gz
    ```
  - tutorialのmainプログラムをビルドするため、GLFW(OpenGL)の導入が必要となる。
    ```
    sudo apt -y install libglfw3-dev
    ```
- [mujoco python bindings](https://mujoco.readthedocs.io/en/2.3.1/python.html#)
  - mujocoをpythonから使用可能。mujoco-pyはサポート終了し、こちらに一本化される

## 初期設定
### wsl2環境にmujocoとGUI(XSrv)を導入する
1. [リンク](https://qiita.com/momomo_rimoto/items/2841f1f15d364c2377a1)
を参考に、mujocoをダウンロードし、`.mujoco`に展開する。
    ```
    mkdir exp-mujoco && cd exp-mujoco
    wget https://www.roboti.us/download/mujoco200_linux.zip
    unzip mujoco.zip -d /home/{username}/.mujoco
    mv ~/.mujoco/mujoco200_linux/ ~/.mujoco/mujoco200
    rm mujoco200_linux.zip 
    ```
2. 環境変数を追加
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco200/bin
    ```

3. 依存ソフトウェアの導入  
`libosmesa6`が何者か不明だが、リンク手順にしたがって導入する。ついでに、pip, mujoco-pyも導入する。
    ```
    sudo apt update
    sudo apt install gcc
    sudo apt install libosmesa6-dev
    sudo apt install python3-pip
    sudo apt install net-tools
    pip install mujoco-py
    ```
3. GUI環境構築  
- [リンク](https://qiita.com/momomo_rimoto/items/51d533ae9529872696ce#wsl2%E3%81%A7GUI%E3%82%92%E4%BD%BF%E3%81%86)
を参考に、VcXsrvを導入する。
  - ubuntu側
    ```
    sudo apt install libgl1-mesa-dev xorg-dev
    ```
  - win側
    1. X Serverの導入。`https://sourceforge.net/projects/vcxsrv/ `からダウンロード。チェックボックスはそのままインストール。
    2. インストールしたXLaunchを起動する。  
      - 注意点1.  additional parametersの設定に`-ac`の付与が必要。  他の起動時設定の変更は必要ない。
![image](https://user-images.githubusercontent.com/127618260/224553969-8490e180-9f4f-4637-bc1c-56285ecd6364.png)
      - 注意点2.  
  VcXsrvのファイアウォールを無効化する。  `パブリック プロファイル->保護されているネットワーク接続: カスタマイズ`からWSLを外す。
![image](https://user-images.githubusercontent.com/127618260/224554301-766e0522-41cb-4bc5-9116-d209a2a752af.png)
    3. GUIインストールテスト用に下記を導入する。`xeyes`を実行しGUIアプリケーション(目玉)が動くことを確認する
```
sudo apt install x11-apps
```

5. アクティベーションキーの導入  
キーが無いと後述の手順でエラーとなる。  
mujocoインストールディレクトリ`~/.mujoco/mujoco200/bin`以下に`mjkey.txt`格納する。
[リンク](https://qiita.com/momomo_rimoto/items/2841f1f15d364c2377a1)先
記載のディレクトリは間違っているようである。
```
curl -O https://www.roboti.us/file/mjkey.txt
```

6. simulationを起動
```
cd ~/.mujoco/mujoco200/bin
./simulate ../model/humanoid.xml
```
4.の手順がされてない場合、下記エラーが発生した。
4.のファイアウォール設定と、XServer設定により解決した。
```
MuJoCo Pro version 2.00
ERROR: could not initialize GLFW
```

### Tips
- wslの再起動
powershell(管理者権限)から`wsl.exe --shutdown`を実行する


