const nets = await eda.pcb_Net.getAllNets();
const byType = {};
for (const n of nets) {
  try {
    const prims = await eda.pcb_Net.getAllPrimitivesByNet(n.net);
    for (const p of (prims||[])) {
      const t = p.pcbItemPrimitiveType || ('layerId' in p ? 'pad' : '?');
      byType[t] = (byType[t]||0)+1;
    }
  } catch(e){}
}
return JSON.stringify(byType, null, 1);
