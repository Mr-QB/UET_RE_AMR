#pragma once
#include <Arduino.h>
//  DO NOT TOUCH!
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, minimum-scale=1, user-scalable=no, viewport-fit=cover">
<title>Motor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; touch-action: none; -webkit-tap-highlight-color: transparent; }
  html, body {
    width: 100%; height: 100%;
    overflow: hidden;
    background: #111827;
    color: #e5e7eb;
    font-family: system-ui, sans-serif;
    user-select: none;
  }
  body {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 48px;
  }
  .v-block, .h-block {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 10px;
  }
  .lbl {
    font-size: 0.65rem;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: #6b7280;
  }
  .v-track {
    position: relative;
    width: 64px;
    height: 320px;
    background: #1f2937;
    border-radius: 32px;
    border: 1px solid #374151;
    cursor: pointer;
  }
  .v-track::after {
    content: '';
    position: absolute;
    left: 10px; right: 10px;
    top: 50%; height: 1px;
    background: #374151;
  }
  .h-track {
    position: relative;
    height: 64px;
    width: 320px;
    background: #1f2937;
    border-radius: 32px;
    border: 1px solid #374151;
    cursor: pointer;
  }
  .h-track::after {
    content: '';
    position: absolute;
    top: 10px; bottom: 10px;
    left: 50%; width: 1px;
    background: #374151;
  }
  .thumb {
    position: absolute;
    width: 48px; height: 48px;
    border-radius: 50%;
    transform: translate(-50%, -50%);
  }
  .v-track .thumb {
    background: #38bdf8;
    box-shadow: 0 0 14px #38bdf850;
    left: 50%; top: 50%;
  }
  .h-track .thumb {
    background: #a78bfa;
    box-shadow: 0 0 14px #a78bfa50;
    top: 50%; left: 50%;
  }
  .val {
    font-size: 1.4rem;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
    min-width: 56px;
    text-align: center;
  }
  #fV { color: #38bdf8; }
  #tV { color: #a78bfa; }
</style>
</head>
<body>
 
<div class="v-block">
  <div class="lbl">▲ Tiến / Lùi ▼</div>
  <div class="v-track" id="fT"><div class="thumb" id="fTh"></div></div>
  <div class="val" id="fV">0</div>
</div>
 
<div class="h-block">
  <div class="lbl">◀ Trái / Phải ▶</div>
  <div class="h-track" id="tT"><div class="thumb" id="tTh"></div></div>
  <div class="val" id="tV">0</div>
</div>
 
<script>
var MAX = 100;
var fwd = 0, turn = 0, timer = null;
 
function makeVertical(trackId, thumbId, valId, onChange) {
  var track = document.getElementById(trackId);
  var thumb = document.getElementById(thumbId);
  var valEl = document.getElementById(valId);
  var on = false;
 
  function calc(clientY) {
    var r = track.getBoundingClientRect();
    var pad = 24;
    var y = Math.max(pad, Math.min(r.height - pad, clientY - r.top));
    var ratio = 1 - (y - pad) / (r.height - 2 * pad); // 1=top, 0=bottom
    return Math.round(ratio * 2 * MAX - MAX);
  }
 
  function set(v) {
    var pad = 24;
    var r = track.getBoundingClientRect();
    var ratio = (v + MAX) / (2 * MAX);
    thumb.style.top = (pad + (1 - ratio) * (r.height - 2 * pad)) + 'px';
    valEl.textContent = v;
    onChange(v);
    clearTimeout(timer); timer = setTimeout(send, 30);
  }
 
  function reset() { on = false; set(0); }
 
  track.addEventListener('mousedown',   function(e){ on = true; set(calc(e.clientY)); });
  window.addEventListener('mousemove',  function(e){ if(on) set(calc(e.clientY)); });
  window.addEventListener('mouseup',    reset);
  track.addEventListener('touchstart',  function(e){ e.preventDefault(); on = true; set(calc(e.touches[0].clientY)); }, {passive:false});
  window.addEventListener('touchmove',  function(e){ if(on){ e.preventDefault(); set(calc(e.touches[0].clientY)); } }, {passive:false});
  window.addEventListener('touchend',   reset);
  window.addEventListener('touchcancel',reset);
}
 
function makeHorizontal(trackId, thumbId, valId, onChange) {
  var track = document.getElementById(trackId);
  var thumb = document.getElementById(thumbId);
  var valEl = document.getElementById(valId);
  var on = false;
 
  function calc(clientX) {
    var r = track.getBoundingClientRect();
    var pad = 24;
    var x = Math.max(pad, Math.min(r.width - pad, clientX - r.left));
    var ratio = (x - pad) / (r.width - 2 * pad); // 0=left, 1=right
    return Math.round(ratio * 2 * MAX - MAX);
  }
 
  function set(v) {
    var pad = 24;
    var r = track.getBoundingClientRect();
    var ratio = (v + MAX) / (2 * MAX);
    thumb.style.left = (pad + ratio * (r.width - 2 * pad)) + 'px';
    valEl.textContent = v;
    onChange(v);
    clearTimeout(timer); timer = setTimeout(send, 30);
  }
 
  function reset() { on = false; set(0); }
 
  track.addEventListener('mousedown',   function(e){ on = true; set(calc(e.clientX)); });
  window.addEventListener('mousemove',  function(e){ if(on) set(calc(e.clientX)); });
  window.addEventListener('mouseup',    reset);
  track.addEventListener('touchstart',  function(e){ e.preventDefault(); on = true; set(calc(e.touches[0].clientX)); }, {passive:false});
  window.addEventListener('touchmove',  function(e){ if(on){ e.preventDefault(); set(calc(e.touches[0].clientX)); } }, {passive:false});
  window.addEventListener('touchend',   reset);
  window.addEventListener('touchcancel',reset);
}
 
function send() {
  var l = Math.max(-MAX, Math.min(MAX, Math.round(fwd + turn)));
  var r = Math.max(-MAX, Math.min(MAX, Math.round(fwd - turn)));
  fetch('/set?left=' + l + '&right=' + r);
}
 
makeVertical  ('fT', 'fTh', 'fV', function(v){ fwd  = v; });
makeHorizontal('tT', 'tTh', 'tV', function(v){ turn = v; });
</script>
</body>
</html>
)rawliteral";