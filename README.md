# robo-sim

Welcome to the robo-sim.
### panda-arm task
* アーム初期姿勢を設定する
* アームをGUIから任意位置へ変換できるようにする
* pidController　I/F設計
* 時系列データの出力
* Jog機能
* Controller設計
  * 重力補償PI制御のゲイン調整
  * PyDrake導入
  * 重力補償器
  * 手先インピーダンス
  * 経路・軌道計画
* 台車拡張
#### optional
* 動力学・運動学
* KinematicTreeの解析
* MujocoVisualizerの表示拡張

#### 近くまとめる
* urdf->mjdfの変換
* 


### リンク
- [github/mujoco](https://github.com/deepmind/mujoco)
- [mujoco documentation](https://mujoco.readthedocs.io/en/latest/overview.html)
- [mujocopy bootcamp](https://pab47.github.io/mujocopy.html)
- [Mujoco-Tutorial](https://github.com/tayalmanan28/Mujoco-Tutorial)
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

## mujoco on wsl2環境構築
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
## mujoco on Windows 環境構築
環境構築が必要なソフトウェア
* [MuJoCo Python Bindings](https://pypi.org/project/mujoco/)
* [Robotics Toolbox for Python](https://pypi.org/project/roboticstoolbox-python/)

### 事前設定
* python3環境を構築済    
    * python version
        ```
        Python 3.10.6 (tags/v3.10.6:9c7b4bd, Aug  1 2022, 21:53:49) [MSC v.1932 64 bit (AMD64)] on win32
        Type "help", "copyright", "credits" or "license" for more information.
        ```
    * pip
        ```
        pip 22.2.1 from xxxxxxxxx\mujoco_env\lib\site-packages\pip (python 3.10)
        ```
### mujoco環境
1. `venv`で環境をつくる（任意）
    ```
    python -m venv mujoco_env
    ; mujoco_envに環境をつくる
    cd mujoco_env\Scripts
    .\activate
    ; 実行後、promptに(mujoco_env) が表示されればOK
    ```
1. pipでmujocoをインストール
    ```
    pip install mujoco
    ```
1. 動作確認
    下記コマンド実行後、mujoco GUIが起動すればOK
    ```
    python -m mujoco.viewer
    ```
### robotics toolbox環境
1. 下記コマンドを実行する。
    ```
    pip install roboticstoolbox-python
    ```
    * 上記mujoco環境構築後の`venv`作成後を想定


2. 起動確認
  * pythonで下記コマンドを実行する
    ```
    import roboticstoolbox as rtb
    robot = rtb.models.Panda()
    print(robot)
    ```
  * `Panda`の情報表示されればOK
    ```
    ERobot: panda (by Franka Emika), 7 joints (RRRRRRR), 1 gripper, geometry, collision
    ┌─────┬──────────────┬───────┬─────────────┬────────────────────────────────────────────────┐
    │link │     link     │ joint │   parent    │              ETS: parent to link               │
    ├─────┼──────────────┼───────┼─────────────┼────────────────────────────────────────────────┤
    │   0 │ panda_link0  │       │ BASE        │ SE3()                                          │
    │   1 │ panda_link1  │     0 │ panda_link0 │ SE3(0, 0, 0.333) ⊕ Rz(q0)                      │
    │   2 │ panda_link2  │     1 │ panda_link1 │ SE3(-90°, -0°, 0°) ⊕ Rz(q1)                    │
    │   3 │ panda_link3  │     2 │ panda_link2 │ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2)       │
    │   4 │ panda_link4  │     3 │ panda_link3 │ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3)       │
    │   5 │ panda_link5  │     4 │ panda_link4 │ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) │
    │   6 │ panda_link6  │     5 │ panda_link5 │ SE3(90°, -0°, 0°) ⊕ Rz(q5)                     │
    │   7 │ panda_link7  │     6 │ panda_link6 │ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6)        │
    │   8 │ @panda_link8 │       │ panda_link7 │ SE3(0, 0, 0.107)                               │
    └─────┴──────────────┴───────┴─────────────┴────────────────────────────────────────────────┘

    ┌─────┬─────┬────────┬─────┬───────┬─────┬───────┬──────┐
    │name │ q0  │ q1     │ q2  │ q3    │ q4  │ q5    │ q6   │
    ├─────┼─────┼────────┼─────┼───────┼─────┼───────┼──────┤
    │  qr │  0° │ -17.2° │  0° │ -126° │  0° │  115° │  45° │
    │  qz │  0° │  0°    │  0° │  0°   │  0° │  0°   │  0°  │
    └─────┴─────┴────────┴─────┴───────┴─────┴───────┴──────┘
    ```

### その他
```
pip install control
```

#### memo
* `import roboticstoolbox as rtb`で下記エラーメッセージが発生した
  ```
  ImportError: cannot import name 'randn' from 'scipy' (C:\Users\Dai Mat\mujoco\mujoco_env\lib\site-packages\scipy\__init__.py)
  ```
    * 現象発生時の`scipy`のversionは以下
      > scipy                  1.13.0
    * 暫定回避：`scipy`のダウングレードのため、下記コマンドを実行する
      > pip install "scipy<1.12.0"
    * Githubの[issue](https://github.com/petercorke/RVC3-python/issues/16)あり。


## Panda armの導入
* urdfの入手
https://github.com/justagist/franka_panda_description

* meshファイル内のdae/stlを.urdfと同じ階層ディレクトリに配置すること

### Tips
* wslの再起動
  powershell(管理者権限)から`wsl.exe --shutdown`を実行する

* powershellでのgrep相当操作
  `Select-String`で代用可能
  > pip list | Select-String "scipy"

* pythonデバッグ on VSCodeで指定の実行環境(venv)を指定する
  * VSCode下隅から環境を選択可能。指定するvenvが表示されない場合、フォルダから`Python.exe`のパスを指定すればよい。
  * ![alt text](image.png)
  * [参考リンク](https://qiita.com/watahani/items/7c1b3b6c470b2f08bf51) 