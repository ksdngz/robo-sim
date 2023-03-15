
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
