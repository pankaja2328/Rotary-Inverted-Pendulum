#pragma once
#include <Arduino.h>

const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MPU6050 3D Visualizer</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&display=swap');
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#07090f;--surface:rgba(255,255,255,.04);--border:rgba(255,255,255,.08);
  --pitch:#00e5ff;--roll:#69ff47;--yaw:#d05aff;--text:#e0e8f0;--sub:#6b7a92;
}
body{background:var(--bg);color:var(--text);font-family:'Outfit',sans-serif;
  min-height:100vh;display:flex;flex-direction:column;align-items:center;
  padding:2rem 1rem;gap:2.5rem}
header{text-align:center}
h1{font-size:clamp(1.4rem,4vw,2rem);font-weight:700;letter-spacing:.04em}
header p{color:var(--sub);font-size:.9rem;margin-top:.3rem}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;
  background:#3ddc84;margin-right:6px;box-shadow:0 0 8px #3ddc84;
  animation:blink 1.4s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
.scene{perspective:600px;width:220px;height:220px}
.cube{width:220px;height:220px;position:relative;transform-style:preserve-3d;
  transition:transform .08s linear}
.face{position:absolute;width:220px;height:220px;
  border:1px solid rgba(255,255,255,.18);display:flex;
  align-items:center;justify-content:center;
  font-size:1rem;font-weight:600;letter-spacing:.08em}
.face.front {background:rgba(0,229,255,.12);transform:translateZ(110px);color:var(--pitch)}
.face.back  {background:rgba(0,229,255,.06);transform:rotateY(180deg) translateZ(110px);color:var(--pitch)}
.face.left  {background:rgba(105,255,71,.12);transform:rotateY(-90deg) translateZ(110px);color:var(--roll)}
.face.right {background:rgba(105,255,71,.06);transform:rotateY(90deg)  translateZ(110px);color:var(--roll)}
.face.top   {background:rgba(208,90,255,.12);transform:rotateX(90deg)  translateZ(110px);color:var(--yaw)}
.face.bottom{background:rgba(208,90,255,.06);transform:rotateX(-90deg) translateZ(110px);color:var(--yaw)}
.face span{text-shadow:0 0 14px currentColor}
.cards{display:flex;gap:1.4rem;flex-wrap:wrap;justify-content:center}
.card{background:var(--surface);border:1px solid var(--border);border-radius:1.2rem;
  padding:1.2rem 1.4rem;display:flex;flex-direction:column;align-items:center;
  gap:.5rem;width:148px;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:-55px;left:50%;
  transform:translateX(-50%);width:130px;height:130px;border-radius:50%;
  filter:blur(38px);opacity:.32}
.card.pitch::before{background:var(--pitch)}
.card.roll::before {background:var(--roll)}
.card.yaw::before  {background:var(--yaw)}
.card label{font-size:.72rem;color:var(--sub);text-transform:uppercase;letter-spacing:.1em}
svg.ring{width:100px;height:100px}
.trk{fill:none;stroke:rgba(255,255,255,.07);stroke-width:9}
.arc{fill:none;stroke-width:9;stroke-linecap:round;
  transition:stroke-dashoffset .12s ease;stroke-dasharray:251.3;stroke-dashoffset:251.3}
.pitch .arc{stroke:var(--pitch);filter:drop-shadow(0 0 5px var(--pitch))}
.roll  .arc{stroke:var(--roll); filter:drop-shadow(0 0 5px var(--roll))}
.yaw   .arc{stroke:var(--yaw);  filter:drop-shadow(0 0 5px var(--yaw))}
.val{font-size:1.55rem;font-weight:700}
.pitch .val{color:var(--pitch)} .roll .val{color:var(--roll)} .yaw .val{color:var(--yaw)}
.unit{font-size:.72rem;color:var(--sub)}
.raw{background:var(--surface);border:1px solid var(--border);border-radius:.8rem;
  padding:.65rem 1.4rem;font-size:.78rem;color:var(--sub);letter-spacing:.04em;
  display:flex;gap:1.4rem;flex-wrap:wrap;justify-content:center}
.raw span{color:var(--text)}
footer{font-size:.75rem;color:var(--sub)}
</style>
</head>
<body>
<header>
  <h1><span class="dot"></span>MPU6050 &mdash; 3D Visualizer</h1>
  <p>Real-time Kalman filtered orientation &bull; ESP32 WiFi AP</p>
</header>

<div class="scene">
  <div class="cube" id="cube">
    <div class="face front"><span>FRONT</span></div>
    <div class="face back"><span>BACK</span></div>
    <div class="face left"><span>LEFT</span></div>
    <div class="face right"><span>RIGHT</span></div>
    <div class="face top"><span>TOP</span></div>
    <div class="face bottom"><span>BOTTOM</span></div>
  </div>
</div>

<div class="cards">
  <div class="card pitch">
    <label>Pitch (X)</label>
    <svg class="ring" viewBox="0 0 100 100">
      <circle class="trk" cx="50" cy="50" r="40"/>
      <circle class="arc" cx="50" cy="50" r="40" id="pArc" transform="rotate(-90 50 50)"/>
    </svg>
    <div class="val" id="pVal">0.0</div>
    <div class="unit">degrees</div>
  </div>
  <div class="card roll">
    <label>Roll (Y)</label>
    <svg class="ring" viewBox="0 0 100 100">
      <circle class="trk" cx="50" cy="50" r="40"/>
      <circle class="arc" cx="50" cy="50" r="40" id="rArc" transform="rotate(-90 50 50)"/>
    </svg>
    <div class="val" id="rVal">0.0</div>
    <div class="unit">degrees</div>
  </div>
  <div class="card yaw">
    <label>Yaw (Z)</label>
    <svg class="ring" viewBox="0 0 100 100">
      <circle class="trk" cx="50" cy="50" r="40"/>
      <circle class="arc" cx="50" cy="50" r="40" id="yArc" transform="rotate(-90 50 50)"/>
    </svg>
    <div class="val" id="yVal">0.0</div>
    <div class="unit">degrees</div>
  </div>
</div>

<div class="raw">
  Pitch:&nbsp;<span id="rp">—</span>&deg; &nbsp;|&nbsp;
  Roll:&nbsp;<span id="rr">—</span>&deg; &nbsp;|&nbsp;
  Yaw:&nbsp;<span id="ry">—</span>&deg;
</div>

<footer>Connected via WiFi &bull; ~20 Hz refresh</footer>

<script>
const cube=document.getElementById('cube');
const pArc=document.getElementById('pArc'),rArc=document.getElementById('rArc'),yArc=document.getElementById('yArc');
const pVal=document.getElementById('pVal'),rVal=document.getElementById('rVal'),yVal=document.getElementById('yVal');
const rp=document.getElementById('rp'),rr=document.getElementById('rr'),ry=document.getElementById('ry');
const C=251.3;
function arc(el,deg){el.style.strokeDashoffset=C*(1-Math.min(Math.abs(deg)/180,1))}
async function poll(){
  try{
    const d=await(await fetch('/data')).json();
    cube.style.transform=`rotateX(${-d.pitch}deg) rotateY(${d.yaw}deg) rotateZ(${d.roll}deg)`;
    arc(pArc,d.pitch);arc(rArc,d.roll);arc(yArc,d.yaw);
    pVal.textContent=d.pitch.toFixed(1);rVal.textContent=d.roll.toFixed(1);yVal.textContent=d.yaw.toFixed(1);
    rp.textContent=d.pitch.toFixed(2);rr.textContent=d.roll.toFixed(2);ry.textContent=d.yaw.toFixed(2);
  }catch(e){}
}
setInterval(poll,50);poll();
</script>
</body>
</html>)rawhtml";
