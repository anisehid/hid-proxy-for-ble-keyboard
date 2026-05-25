const $ = id => document.getElementById(id);
const j = async (m,u,b) => fetch(u,{method:m,headers:{'Content-Type':'application/json'},
                                    body:b?JSON.stringify(b):undefined}).then(r=>r.json());

const BLE = {1:'connected',2:'scan',3:'connecting'};

async function tick() {
  let s;
  try {
    const r = await fetch('/api/status');
    if (!r.ok) { location = '/login'; return; }
    s = await r.json();
  } catch (e) {
    $('status').innerHTML = '<span style=color:#a33>Network error: '+e.message+'</span>';
    return;
  }
  let conn = '';
  if (s.connected_device) {
    const nm = s.connected_name ? `<b>${s.connected_name}</b> ` : '';
    conn = ` &middot; connected: ${nm}<code>${s.connected_device}</code>`;
  }
  $('status').innerHTML =
    `Mode: <b>${s.mode}</b> &middot; BLE: <b>${BLE[s.ble_status]||s.ble_status}</b>` +
    conn +
    ` &middot; uptime ${s.uptime_s}s &middot; AP idle remaining ${s.ap_idle_remaining_s}s`;
  $('savedN').textContent = s.saved_count;

  try {
    const devs = await fetch('/api/devices').then(r=>r.json());
    $('saved').innerHTML = devs.map(d=>{
      const badge = d.is_connected
        ? '<b style="color:#0a0">online</b>'
        : '<span style="color:#a44">offline</span>';
      return `<div class=row><span>${d.name||'(no name)'} &middot; ${badge}</span>
       <code>${d.mac}</code>
       <button data-slot=${d.slot} class=del>Remove</button></div>`;
    }).join('') || '<div class=muted>No saved keyboards yet.</div>';
    for (const b of document.querySelectorAll('.del')) b.onclick = async () => {
      await fetch('/api/devices/'+b.dataset.slot, {method:'DELETE'});
      tick();
    };
  } catch (e) { /* keep prior list */ }

  if ($('discT').checked) {
    try {
      const d = await fetch('/api/discovery').then(r=>r.json());
      $('disc').innerHTML = d.results.length
        ? d.results.map(r=>`<div class=row><span>${r.name||'(no name)'}</span>
              <code>${r.mac}</code> <span class=muted>${r.rssi} dBm</span>
              <button data-mac="${r.mac}" data-at="${r.addr_type}" class=conn>Connect</button></div>`).join('')
        : '<div class=muted>Scanning… (this can take 10-20 s on first tick when Wi-Fi is active)</div>';
      for (const b of document.querySelectorAll('.conn')) b.onclick = async () => {
        await j('POST','/api/devices/connect',
                {mac:b.dataset.mac, addr_type: parseInt(b.dataset.at,10)});
      };
    } catch (e) { /* swallow */ }
  } else {
    $('disc').textContent = 'Discovery off.';
  }
}

$('discT').onchange = async e => {
  await j('POST','/api/discovery', {enabled: e.target.checked});
};
$('logout').onclick = async () => {
  await fetch('/api/auth/logout', {method:'POST'}); location='/login';
};
$('shut').onclick = async () => {
  await fetch('/api/admin/shutdown_ap', {method:'POST'});
};
$('reset').onclick = async () => {
  if (!confirm('Factory reset will erase all saved keyboards and the admin password. Continue?')) return;
  await j('POST','/api/admin/factory_reset', {confirm:'WIPE'});
};
tick();
setInterval(tick, 1000);
