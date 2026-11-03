"""Pinned acquisition for local verification; fetched source is never packaged."""
from pathlib import Path
import hashlib,json,urllib.request,urllib.parse

def fetch(target=Path('.upstream')):
    m=json.loads((Path(__file__).parent/'data/upstream.json').read_text());target=Path(target).resolve();n=0
    for f in m['files']:
        name=f['path']
        if not (name.endswith(('.py','.m')) or name=='LICENSE'):continue
        path=target/name;path.parent.mkdir(parents=True,exist_ok=True)
        if not path.exists():
            url=m['repository'].replace('https://github.com/','https://raw.githubusercontent.com/')+'/'+m['commit']+'/'+urllib.parse.quote(name)
            content=urllib.request.urlopen(url,timeout=30).read()
            if hashlib.sha256(content).hexdigest()!=f['sha256']:raise ValueError('Downloaded source hash mismatch: '+name)
            path.write_bytes(content)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=f['sha256']:raise ValueError('Existing source hash mismatch: '+name)
        n+=1
    return {'verified_source_files':n,'commit':m['commit'],'upstream_license_files':m['license_files'],'redistributed_in_public_package':False}
