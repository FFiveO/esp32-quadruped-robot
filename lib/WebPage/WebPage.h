#pragma once

const char MAIN_PAGE[] PROGMEM = R"=====(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover, user-scalable=no">
  <meta name="screen-orientation" content="landscape">
  <meta name="x5-orientation" content="landscape">
  <title>Robot Dog Control</title>
  <style>
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; user-select: none; }
    html, body { width: 100%; height: 100%; margin: 0; overflow: hidden; background: #10141f; color: #eef3ff; font-family: Arial, "Microsoft YaHei", sans-serif; }
    body { touch-action: none; }
    .app { width: 100vw; height: 100vh; padding: 12px max(12px, env(safe-area-inset-right)) 12px max(12px, env(safe-area-inset-left)); display: grid; grid-template-columns: 1fr 310px; gap: 12px; }
    .stage { position: relative; border-radius: 22px; background: radial-gradient(circle at 30% 20%, #25365c, #121827 65%); box-shadow: inset 0 0 0 1px rgba(255,255,255,.08); overflow: hidden; }
    .title { position: absolute; left: 18px; top: 14px; font-size: 18px; font-weight: 700; letter-spacing: .5px; }
    .hint { position: absolute; left: 18px; top: 42px; opacity: .68; font-size: 12px; }
    .status { position: absolute; left: 18px; bottom: 14px; display: grid; gap: 4px; font-size: 13px; opacity: .8; }
    .joyWrap { position: absolute; right: 22px; bottom: 22px; width: 168px; height: 168px; border-radius: 50%; background: rgba(255,255,255,.09); border: 2px solid rgba(255,255,255,.22); box-shadow: 0 10px 30px rgba(0,0,0,.35); }
    .joyBase { position: absolute; inset: 18px; border-radius: 50%; background: rgba(0,0,0,.18); border: 1px dashed rgba(255,255,255,.22); }
    .stick { position: absolute; left: 54px; top: 54px; width: 60px; height: 60px; border-radius: 50%; background: linear-gradient(180deg,#55d6ff,#3276ff); box-shadow: 0 8px 22px rgba(50,118,255,.45); }
    .side { min-width: 0; border-radius: 22px; background: rgba(255,255,255,.07); border: 1px solid rgba(255,255,255,.09); padding: 12px; overflow: auto; }
    .group { margin-bottom: 12px; padding: 10px; border-radius: 16px; background: rgba(0,0,0,.17); }
    .group h3 { margin: 0 0 8px; font-size: 14px; color: #cfe2ff; }
    .row { display: grid; grid-template-columns: 72px 1fr 42px; align-items: center; gap: 8px; margin: 8px 0; font-size: 12px; }
    input[type="range"] { width: 100%; accent-color: #55d6ff; }
    .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button { border: 0; border-radius: 13px; padding: 12px 8px; color: white; font-weight: 700; background: #2f74ff; box-shadow: 0 5px 14px rgba(0,0,0,.22); }
    button:active { transform: scale(.97); }
    button.danger { background: #e23a4e; }
    button.gray { background: #607083; }
    button.warn { background: #e09b24; }

    .gaitBtn {opacity: 0.65;}
    .gaitBtn.activeGait {opacity: 1; outline: 2px solid white; transform: scale(1.03); }
    
    .rotateTip { display: none; position: fixed; inset: 0; place-items: center; text-align: center; padding: 24px; background: #10141f; z-index: 9; }
    @media (orientation: portrait) { .rotateTip { display: grid; } }
  </style>
</head>


<body>
  <div class="rotateTip">请把手机横过来使用<br>打开 192.168.4.1 后就是类似游戏手柄的横屏控制界面</div>

  <main class="app">
    <aside class="side">
      <div class="group">
        <h3>姿态调整</h3>
        <div class="row"><span>速度</span><input id="speed" type="range" min="0" max="1" step="0.01" value="0.75"><b id="speedV">0.5</b></div>
        <div class="row"><span>机身高度</span><input id="height" type="range" min="-30" max="15" step="1" value="0"><b id="heightV">0</b></div>
        <div class="row"><span>前后倾</span><input id="roll" type="range" min="-18" max="18" step="1" value="0"><b id="rollV">0</b></div>
        <div class="row"><span>左右倾</span><input id="pitch" type="range" min="-18" max="18" step="1" value="0"><b id="pitchV">0</b></div>
        <div class="row"><span>抬腿高度</span><input id="lift" type="range" min="0.5" max="1.5" step="0.01" value="1"><b id="liftV">1.00</b></div>
        
        <div class="row"><label>音量</label><input type="range" min="0" max="100" value="0" oninput="setVolume(this.value)"></div>      
        </div>

      <div class="group">
        <h3>基础功能</h3>
        <div class="buttons">
          <button class="danger" onclick="sendCmd('display')">切换显示屏</button>
          <button class="danger" onclick="sendCmd('bored_mode_change')">切换无聊模式</button>
          <button class="gray" onclick="sendCmd('balance_normal')">普通平衡</button>
          <button class="gray" onclick="sendCmd('balance_strong')">超强平衡(演示)</button>
          <button class="warn" onclick="sendCmd('balance')">切换平衡模式</button>
          <button class="warn" onclick="resetRobot()">姿态复位</button>
          <button class="warn" onclick="sendCmd('info')">打印信息</button>

        </div>
      </div>

      <div class="group">
        <h3>步态选择</h3>
        <div class="buttons">
          <button id="gaitTrot" class="gray gaitBtn activeGait" onclick="selectGait('trot')"> Trot 小跑</button>
          <button id="gaitWalk" class="gray gaitBtn" onclick="selectGait('walk')"> Swing 摆动</button>
          </div>
      </div>

      <div class="group">
        <h3>互动</h3>
        <div class="buttons">
          <button class="gray" onclick="runAction('march')">踏步</button>
          <button class="gray" onclick="stopRobot()">停止踏步</button>
          <button class="gray" onclick="runAction('lie')">趴下</button>
          <button class="gray" onclick="runAction('stand')">站立</button>
          <button class="gray" onclick="runAction('wave')">挥手</button>
          <button class="gray" onclick="runAction('pushup')">俯卧撑</button>
          <button class="gray" onclick="runAction('wavebody')">扭动</button>
          <button class="gray" onclick="runAction('pee')">小便</button>

        </div>
      </div>

      <div class="group">
        <h3>声音</h3>
        <div class="buttons">

          <button class="gray" onclick="sendCmd('V hello')">打招呼</button>
          <button class="gray" onclick="sendCmd('V happy')">开心</button>
          <button class="gray" onclick="sendCmd('V curious')">好奇</button>
          <button class="gray" onclick="sendCmd('V sleepy')">困</button>
          <button class="gray" onclick="sendCmd('V long')">长叫</button>
        </div>
      </div>

    </aside>

    <section class="stage">
      <div class="title">悦动狗狗</div>
      <div class="hint">右下角摇杆: Y 前进/后退, X 左转/右转；松手自动回中并停止。</div>
      <div class="status">
        <div id="xyText">X: 0.00 Y: 0.00</div>
        <div id="sendText">通信：等待操作</div>
      </div>

      <div id="joy" class="joyWrap">
        <div class="joyBase"></div>
        <div id="stick" class="stick"></div>
      </div>
    </section>
  </main>

<script>
  const state = { x: 0, y: 0, touching: false, lastSend: 0, gait: 'trot'};
  const joy = document.getElementById('joy');
  const stick = document.getElementById('stick');
  const xyText = document.getElementById('xyText');
  const sendText = document.getElementById('sendText');
  const sliders = ['speed','height','pitch','roll','lift'];

  function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

  function updateLabels() {
    sliders.forEach(id => {
      const el = document.getElementById(id);
      const out = document.getElementById(id + 'V');
      out.textContent = (id === 'speed' || id === 'lift') ? Number(el.value).toFixed(2) : el.value;
    });
  }

  function setStick(x, y) {
    state.x = clamp(x, -1, 1);
    state.y = clamp(y, -1, 1);
    const max = 54;
    stick.style.transform = `translate(${state.x * max}px, ${-state.y * max}px)`;
    xyText.textContent = `X: ${state.x.toFixed(2)} Y: ${state.y.toFixed(2)}`;
  }

  function pointerToJoystick(e) {
    const r = joy.getBoundingClientRect();
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    const dx = e.clientX - cx;
    const dy = e.clientY - cy;
    const max = r.width * 0.36;
    const len = Math.hypot(dx, dy);
    const scale = len > max ? max / len : 1;
    setStick((dx * scale) / max, -(dy * scale) / max);
  }

  function setVolume(v){ 
      fetch( "/action?cmd=VOLUME "+ v );
  }



  joy.addEventListener('pointerdown', e => {
    state.touching = true;
    joy.setPointerCapture(e.pointerId);
    pointerToJoystick(e);
    sendControl(true);
  });

  joy.addEventListener('pointermove', e => {
    if (!state.touching) return;
    pointerToJoystick(e);
    sendControl(false);
  });

  function releaseJoystick() {
    state.touching = false;
    setStick(0, 0);
    sendControl(true);
  }

  joy.addEventListener('pointerup', releaseJoystick);
  joy.addEventListener('pointercancel', releaseJoystick);

  sliders.forEach(id => {
    document.getElementById(id).addEventListener('input', () => {
      updateLabels();
      sendControl(false);
    });
  });

  function params() {
    return new URLSearchParams({
      x: state.x.toFixed(3),
      y: state.y.toFixed(3),
      speed: document.getElementById('speed').value,
      height: document.getElementById('height').value,
      pitch: document.getElementById('pitch').value,
      roll: document.getElementById('roll').value,
      lift: document.getElementById('lift').value
    });
  }

  function sendControl(force) {
    const now = Date.now();
    if (!force && now - state.lastSend < 60) return; // 约 16Hz, ESP32 WebServer 更稳定
    state.lastSend = now;
    fetch('/control?' + params().toString())
      .then(r => r.text())
      .then(() => sendText.textContent = '通信：已发送 ' + new Date().toLocaleTimeString())
      .catch(() => sendText.textContent = '通信：失败');
  }

  function sendCmd(cmd) {
    fetch('/action?cmd=' + encodeURIComponent(cmd)).catch(() => {});
  }

  function selectGait(name){
      state.gait = name;

      sendCmd('gait ' + name);

      
      document
          .querySelectorAll('.gaitBtn')
          .forEach(btn => {
              btn.classList.remove('activeGait');
          });


      if(name === 'trot')
      {
          document
              .getElementById('gaitTrot')
              .classList.add('activeGait');
      }

      if(name === 'walk')
      {
          document
              .getElementById('gaitWalk')
              .classList.add('activeGait');
      }


      sendText.textContent =
          '当前步态：' + name;
  }    

  function stopRobot() {
    setStick(0, 0);
    sendCmd('stop_march');
    sendControl(true);
  }

  function resetRobot() {
    ['height','pitch','roll'].forEach(id => document.getElementById(id).value = 0);
    document.getElementById('speed').value = 0.5;
    document.getElementById('lift').value = 1;
    updateLabels();
    setStick(0, 0);
    sendCmd('reset');
    sendControl(true);
  }

  function runAction(name) {
    if (Math.abs(state.x) > 0.03 || Math.abs(state.y) > 0.03) {
      sendText.textContent = '动作：请先松开摇杆回中';
      return;
    }
    fetch('/moveAction?name=' + encodeURIComponent(name))
      .then(r => sendText.textContent = r.ok ? '动作：已发送 ' + name : '动作：摇杆未回中或动作不可用')
      .catch(() => sendText.textContent = '动作：发送失败');
  }

  updateLabels();
  setStick(0, 0);
  setInterval(() => sendControl(false), 120); // 心跳，滑杆姿态保持实时同步
</script>
</body>
</html>
)=====";
