const $ = id => document.getElementById(id);
const j = async (m,u,b) => fetch(u,{method:m,headers:{'Content-Type':'application/json'},
                                    body:b?JSON.stringify(b):undefined}).then(r=>r.json());

async function tick() {
  const s = await fetch('/api/status').then(r=>r.ok?r.json():null);
  if (!s) { location = '/login'; return; }
  $('status').innerHTML =
    `Mode: <b>${s.mode}</b> &middot; uptime ${s.uptime_s}s &middot; ` +
    `AP idle remaining ${s.ap_idle_remaining_s}s`;
  $('savedN').textContent = s.saved_count;

  const devs = await fetch('/api/devices').then(r=>r.json());
  $('saved').innerHTML = devs.map(d=>
    `<div class=row><span>${d.name||'(no name)'}</span>
     <code>${d.mac}</code>
     <button data-slot=${d.slot} class=del>Remove</button></div>`).join('') ||
    '<div class=muted>No saved keyboards yet.</div>';
  for (const b of document.querySelectorAll('.del')) b.onclick = async () => {
    await fetch('/api/devices/'+b.dataset.slot, {method:'DELETE'});
    tick();
  };

  if ($('discT').checked) {
    const d = await fetch('/api/discovery').then(r=>r.json());
    $('disc').innerHTML = d.results.length
      ? d.results.map(r=>`<div class=row><span>${r.name||'(no name)'}</span>
            <code>${r.mac}</code> <span class=muted>${r.rssi} dBm</span>
            <button data-mac="${r.mac}" data-at="${r.addr_type}" class=conn>Connect</button></div>`).join('')
      : '<div class=muted>Scanning…</div>';
    for (const b of document.querySelectorAll('.conn')) b.onclick = async () => {
      await j('POST','/api/devices/connect',
              {mac:b.dataset.mac, addr_type: parseInt(b.dataset.at,10)});
    };
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
