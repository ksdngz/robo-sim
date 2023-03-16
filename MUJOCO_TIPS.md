## 勉強することメモ
- 状態フィードバックとPD制御の関係
- mujoco-py
- 倒立振子運動方程式導出（ニュートン/ラグランジアン）を使用
- 二重倒立振子
- 倒立振子台車軌道生成
- 学習とシミュレーションの組み合わせ
- pythonモジュール化とctypes使用方法
- mujoco(C++)呼び出し

## 倒立振子運動方程式
$$ 
\dot{\boldsymbol{x}} = A\boldsymbol{x}+B\boldsymbol{u}
$$

$$
A=\left[
\begin{array}{cccc}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1\\
0 & -\frac{3mg}{M} & -\frac{4D}{M} & 0\\
0 & \frac{3(m_c+m)g}{Ml} & \frac{3D}{Ml} & 0 \\
\end{array}
\right],  
B = \left[
\begin{array}{c}
0 \\
0 \\
\frac{4}{M} \\
-\frac{3}{Ml} \\
\end{array}
\right] 
$$ 

ここで、 $M=4M_c+m$ である。





## mujocoシミュレーションの理解メモ

- コントローラの設定
  - `mjcb_control`にコールバック関数をセットする

- システム定義
  - 状態変数 $x$   `x = (mjData.time, mjData.qpos, mjData.qvel, mjData.act)`
  - 入力変数 $u$   `u = (mjData.ctrl, mjData.qfrc_applied, mjData.xfrc_applied)` 

- 制御入力 $u$ の設定
  - actuatorのトルクを直接制御する -> `ctrl`
  - joint座標系で力を入力 -> `mjData.qfrc_applied`
  - cartesian座標系でwrenchを入力 -> `mjData.xfrc_applied`

  <p>These input fields are set by the user and affect the physics simulation, but are untouched by the simulator. All input
  fields except for MoCap poses default to 0.</p>
  <blockquote>
  <div><dl>
  <dt>Controls: <code class="docutils literal notranslate"><span class="pre">ctrl</span></code></dt><dd><p>Controls are defined by the <a class="reference internal"  href="XMLreference.html#actuator"><span class="std std-ref">actuator</span></a> section of the XML. <code class="docutils literal notranslate"><span  class="pre">mjData.ctrl</span></code> values either produce
  generalized forces directly (stateless actuators), or affect the actuator activations in <code class="docutils literal notranslate"><span class="pre">mjData.act</span>   </code>, which then
  produce forces.</p>
  </dd>
  <dt>Auxillary Controls: <code class="docutils literal notranslate"><span class="pre">qfrc_applied</span></code> and <code class="docutils literal notranslate"><span  class="pre">xfrc_applied</span></code></dt><dd><div class="line-block">
  <div class="line"><code class="docutils literal notranslate"><span class="pre">mjData.qfrc_applied</span></code> are directly applied generalized forces.</div>
  <div class="line"><code class="docutils literal notranslate"><span class="pre">mjData.xfrc_applied</span></code> are Cartesian wrenches applied to the CoM of individual  bodies. This field is used for
  example, by the <a class="reference internal" href="programming/samples.html#sasimulate"><span class="std std-ref">native viewer</span></a> to apply mouse  perturbations.</div>
  <div class="line">Note that the effects of <code class="docutils literal notranslate"><span class="pre">qfrc_applied</span></code> and <code class="docutils literal  notranslate"><span class="pre">xfrc_applied</span></code> can usually be recreated by appropriate actuator
  definitions.</div>
  </div>
  </dd>
  </dl>
  </div></blockquote>
