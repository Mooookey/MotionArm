"""Assemble the complete course submission without altering original evidence."""
from pathlib import Path
import shutil,json,hashlib,zipfile
ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'submission/imu_exam_final'
DEST.mkdir(parents=True,exist_ok=True)
for directory in ['firmware','web','tools','tests','docs','results','outlook']:
    shutil.copytree(ROOT/directory,DEST/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__','*.exe','font-cache'))
for name in ['README.md','PROTOCOL.md']:
    shutil.copy2(ROOT/name,DEST/name)
dep=json.loads((ROOT/'submission_build/dependencies.json').read_text(encoding='utf-8'))
(DEST/'binaries').mkdir(exist_ok=True)
def find_bins(o):
    if isinstance(o,dict):
        if o.get('recommended_for_delivery') and o.get('source_path'):
            source=ROOT/o['source_path'];shutil.copy2(source,DEST/'binaries'/source.name)
        for value in o.values():find_bins(value)
    elif isinstance(o,list):
        for value in o:find_bins(value)
find_bins(dep)
for lib in dep['firmware']['libraries']:
    shutil.copytree(lib['installed_directory'],DEST/'libraries'/lib['directory'],dirs_exist_ok=True,ignore=shutil.ignore_patterns('.git','__pycache__'))
(DEST/'dependencies.json').write_text(json.dumps(dep,ensure_ascii=False,indent=2),encoding='utf-8')
videos=[
 ('c1a0ac3352f12ba8a9b779eae1a525fd.mp4','01_y_axis.mp4',34.066667,'倾转与返回补充'),
 ('d2db614f969a381068f91e0905789476.mp4','02_z_axis.mp4',24.6,'水平转向与返回补充'),
 ('da3ec07ae38437bb66031fd8f0582412.mp4','03_continuous_follow.mp4',64.333333,'实体与网页模型同框 多轴连续跟随主证据'),
 ('fa22c5c36b6e77b669e38087b794c5af.mp4','04_displacement.mp4',31.2,'位移尝试补充'),
]
(DEST/'videos').mkdir(exist_ok=True); video_manifest=[]
for original,name,duration,role in videos:
    src=ROOT/'videos'/original;dst=DEST/'videos'/name;shutil.copy2(src,dst)
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert digest(src)==digest(dst)
    video_manifest.append(dict(original_filename=original,submission_filename=name,duration_s=duration,role=role,sha256=digest(dst),unchanged_original_content=True))
(DEST/'videos/manifest.json').write_text(json.dumps(video_manifest,ensure_ascii=False,indent=2),encoding='utf-8')
(DEST/'videos/README.md').write_text('''# 视频原件说明

优先播放03_continuous_follow.mp4，64.333秒，H264，实体与网页3D模型同框。其余三个视频补充倾转、水平转向与位移尝试。
按时间要求保留完整原片，未切片、转码、变速或合并；文件内容SHA256与原件一致，原文件名映射见manifest.json。
定性检查支持实体与模型同步。丝印细节不够清楚，不能声称逐帧严格核实所有轴的正方向。四步操作与方向说明见docs/AXIS_GUIDE.md。
''',encoding='utf-8')
# Bring older working notes in the copied package up to date; samples are untouched.
for name in ['README.md','docs/ACCEPTANCE.md','docs/RESULTS.md','results/index.html']:
    path=DEST/name;text=path.read_text(encoding='utf-8')
    text=text.replace('视频由操作者保存在另一设备，提交时需另附原件','视频原件已归档于videos/，其中03_continuous_follow.mp4为64.333秒主证据')
    text=text.replace('当前工作区没有视频','视频原件已纳入videos目录')
    text=text.replace('工作区未包含视频','视频原件已纳入videos目录')
    text=text.replace('工作区未收到视频文件，正式提交必须附该设备上的原件','四个未经剪辑的视频原件已归档到videos/；03_continuous_follow.mp4长64.333秒，作为连续跟随主证据')
    text=text.replace('视频录在另一设备，提交仍须附原件，','视频原件已归档于videos/，')
    text=text.replace('用户确认的实体/模型同步视频在另一设备；正式提交需附视频原件，','实体/模型同步视频原件已纳入videos/；')
    text=text.replace('实体与屏幕同时入镜的视频由操作者保存在另一设备，提交时另附原件。','实体与屏幕同框的视频原件已纳入videos/，主片03_continuous_follow.mp4长64.333秒。')
    path.write_text(text,encoding='utf-8')
manifest=DEST/'results/selections/manifest.json'
m=json.loads(manifest.read_text(encoding='utf-8'));m['video']='Four unchanged original videos included in videos/; main continuous-follow clip is 03_continuous_follow.mp4 (64.333s).';manifest.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
readme='''# 六轴IMU实验正式提交包

## 从这里开始

1. 阅读report/acceptance_report.pdf，正文已填完，不署名；可编辑文字源为report/acceptance_report.md。
2. 前六项指标的图在evidence/；前五项来自305秒完整记录，第六项来自第二轮23–66.75秒主展示段。
3. 第七项优先播放videos/03_continuous_follow.mp4（64.333秒）；另三原视频作为补充，未剪辑、未转码。
4. 完整数据与设备时间戳见results/各会话的data.csv、recording.json、metadata.json与metrics.json。results/selections保留推荐展示区间及选择依据，不改原始样本。

## 源码与运行

- firmware/ImuExam：ESP32完整源码与算法说明；web/：完整无依赖本地上位机源码。
- tools/serve.ps1启动网页；在Edge/Chrome打开http://127.0.0.1:8000，选择当前串口。运行460800、8N1；烧录115200。
- tools/check.ps1运行23项JavaScript检查与25项C++数学检查，本次均通过。
- 使用Arduino核心3.3.12、板型ESP32 Dev Module、FQBN esp32:esp32:esp32。libraries/包含实际使用的三项Adafruit库及许可证；dependencies.json列出版本与二进制SHA。
- binaries/提供本次已验证构建。可用merged镜像在0x0烧录，或使用分离文件；准确偏移见dependencies.json。当前开发板已烧录且校验通过，无需为查看报告重新烧录。
- 本工作区构建入口仍是tools/esp32.ps1；若在其他电脑复现，请安装同版本核心并指向本包libraries目录，工程及构建路径保持英文。

## 结果与边界

静态305秒生成/有效接收61000帧，200Hz，零丢帧、零CRC错误、零采样失败。放置为人工近似水平，未独立证明严格水平或全过程绝对无扰动，报告保留这些条件限制。
位移计算与显示已经运行，但量值明显漂移；选段约0.58米不能作为约15厘米定位准确的证据。视频支持定性跟随，丝印细节有限，不宣称逐帧严格校验各轴正方向。

SHA256SUMS.txt覆盖包内文件，可用于检查文件完整性。原始数据和视频没有修改，精选片段没有伪造END或重编号。
'''
(DEST/'README_FIRST.md').write_text(readme,encoding='utf-8')
# Preserve provenance and source requirement audit without internal review thumbnails.
if (ROOT/'submission_build/requirements_audit.json').exists():shutil.copy2(ROOT/'submission_build/requirements_audit.json',DEST/'requirements_audit.json')
required=['report/acceptance_report.pdf','report/acceptance_report.md','firmware/ImuExam/ImuExam.ino','web/app.js','videos/03_continuous_follow.mp4','results/static_20261104_2010/data.csv']
assert all((DEST/p).is_file() and (DEST/p).stat().st_size>0 for p in required)
assert len(list((DEST/'evidence').glob('*.png')))==6
sums=[]
for path in sorted(DEST.rglob('*')):
    if path.is_file() and path.name!='SHA256SUMS.txt':sums.append(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.relative_to(DEST).as_posix())
(DEST/'SHA256SUMS.txt').write_text('\n'.join(sums)+'\n',encoding='utf-8')
archive=ROOT/'submission/imu_exam_final_complete.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for path in sorted(DEST.rglob('*')):
        if path.is_file():z.write(path,Path('imu_exam_final')/path.relative_to(DEST))
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    assert any(n.endswith('videos/03_continuous_follow.mp4') for n in z.namelist())
print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,files=len(sums)+1,sha256=hashlib.sha256(archive.read_bytes()).hexdigest()),indent=2))
