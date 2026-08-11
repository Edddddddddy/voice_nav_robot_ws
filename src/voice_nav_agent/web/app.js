const $ = (id) => document.getElementById(id);
const canvas = $('map');
const ctx = canvas.getContext('2d');
let lastMapRevision = -1;
let latestMap = null;
let latestPose = null;
let toastTimer;

function toast(message, failed = false) {
  const node = $('toast');
  node.textContent = message;
  node.style.borderColor = failed ? 'rgba(255,77,97,.6)' : '';
  node.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => node.classList.remove('show'), 2600);
}

async function request(path, options = {}) {
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload;
}

async function sendCommand(text) {
  try {
    await request('/api/command', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({text})
    });
    toast(`任务已发送 · ${text}`);
  } catch (error) { toast(error.message, true); }
}

async function stopRobot() {
  try {
    const result = await request('/api/stop', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}'
    });
    toast(result.direct_stop ? 'STOP 已直达 Runtime' : 'STOP 已交给 Agent 重试');
  } catch (error) { toast(error.message, true); }
}

function updatePlaces(places) {
  const root = $('places');
  root.replaceChildren(Object.assign(document.createElement('span'), {textContent: 'Named places'}));
  places.forEach((place) => {
    const button = document.createElement('button');
    button.textContent = place;
    button.addEventListener('click', () => sendCommand(`去 ${place}`));
    root.append(button);
  });
}

function updateState(state) {
  latestPose = state.pose;
  $('connection').textContent = state.connected ? 'Runtime 在线' : 'Runtime 离线';
  document.querySelector('.connection').classList.toggle('online', state.connected);
  $('mode').textContent = state.mode;
  $('availability').textContent = state.availability;
  $('gate').textContent = state.gate;
  $('epoch').textContent = state.epoch || '--';
  $('activeStep').textContent = `${state.active_step || 0} / ${state.max_steps || 0}`;
  $('runtimeId').textContent = state.runtime_id ? state.runtime_id.slice(0, 10) : '--';
  $('lastEvent').textContent = state.last_event;
  $('coordinates').textContent = state.pose ? `X ${state.pose.x.toFixed(2)} · Y ${state.pose.y.toFixed(2)}` : 'X -- · Y --';
  updatePlaces(state.named_places || []);
  if (state.map_revision !== lastMapRevision) loadMap(state.map_revision);
  drawMap();
}

async function loadMap(revision) {
  try {
    latestMap = await request('/api/map');
    lastMapRevision = revision;
    $('mapEmpty').style.display = latestMap.available ? 'none' : 'grid';
    drawMap();
  } catch (error) { toast(error.message, true); }
}

function fitCanvas() {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
  }
}

function drawMap() {
  fitCanvas();
  ctx.fillStyle = '#030807'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!latestMap?.available) return;
  const {width, height, cells} = latestMap;
  const scale = Math.min(canvas.width / width, canvas.height / height) * .91;
  const ox = (canvas.width - width * scale) / 2;
  const oy = (canvas.height - height * scale) / 2;
  cells.forEach((value, index) => {
    if (value < 0) return;
    const x = index % width;
    const y = Math.floor(index / width);
    ctx.fillStyle = value > 55 ? '#18312a' : `rgba(117,255,197,${.12 + (100 - value) / 270})`;
    ctx.fillRect(ox + x * scale, oy + (height - y - 1) * scale, Math.ceil(scale), Math.ceil(scale));
  });
  if (!latestPose) return;
  const mx = (latestPose.x - latestMap.origin.x) / latestMap.resolution;
  const my = (latestPose.y - latestMap.origin.y) / latestMap.resolution;
  const x = ox + mx * scale;
  const y = oy + (height - my) * scale;
  const yaw = 2 * Math.atan2(latestPose.z, latestPose.w);
  ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw);
  ctx.shadowBlur = 18; ctx.shadowColor = '#67dff4'; ctx.fillStyle = '#67dff4';
  ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-8, -8); ctx.lineTo(-4, 0); ctx.lineTo(-8, 8); ctx.closePath(); ctx.fill(); ctx.restore();
}

async function poll() {
  try { updateState(await request('/api/state')); }
  catch (_) {
    $('connection').textContent = '控制台离线';
    document.querySelector('.connection').classList.remove('online');
  }
}

$('commandForm').addEventListener('submit', (event) => {
  event.preventDefault();
  const input = $('command');
  const text = input.value.trim();
  if (text) { sendCommand(text); input.value = ''; }
});
document.querySelectorAll('[data-command]').forEach((button) => button.addEventListener('click', () => sendCommand(button.dataset.command)));
$('stop').addEventListener('click', stopRobot);
window.addEventListener('resize', drawMap);
setInterval(() => $('clock').textContent = new Date().toLocaleTimeString('zh-CN', {hour12: false}), 1000);
setInterval(poll, 1000);
poll();
