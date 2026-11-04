"""Build the submission PDF from verified report text and original evidence."""
from pathlib import Path
import re, json, html
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak, KeepTogether
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.lib.pagesizes import letter
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from PIL import Image as PILImage

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'submission' / 'imu_exam_final'
(OUT / 'report').mkdir(parents=True, exist_ok=True)
(OUT / 'evidence').mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(TTFont('CN', 'C:/Windows/Fonts/msyh.ttc', subfontIndex=0))
pdfmetrics.registerFont(TTFont('CNBold', 'C:/Windows/Fonts/msyhbd.ttc', subfontIndex=0))
pdfmetrics.registerFontFamily('CN', normal='CN', bold='CNBold', italic='CN', boldItalic='CNBold')
W,H = letter
MARGIN = 43
CONTENT = W-2*MARGIN
styles = {
 'title': ParagraphStyle('title',fontName='CNBold',fontSize=20,leading=29,textColor=colors.black,spaceAfter=12),
 'h1': ParagraphStyle('h1',fontName='CNBold',fontSize=14,leading=21,spaceBefore=14,spaceAfter=9,keepWithNext=True,textColor=colors.black),
 'h2': ParagraphStyle('h2',fontName='CNBold',fontSize=11.5,leading=18,spaceBefore=10,spaceAfter=6,keepWithNext=True,textColor=colors.black),
 'body': ParagraphStyle('body',fontName='CN',fontSize=10.5,leading=16,spaceAfter=7,wordWrap='CJK',allowWidows=0,allowOrphans=0),
 'formula': ParagraphStyle('formula',fontName='CN',fontSize=10.5,leading=17,spaceBefore=3,spaceAfter=8,wordWrap='CJK'),
 'table': ParagraphStyle('table',fontName='CN',fontSize=9,leading=13,wordWrap='CJK'),
 'th': ParagraphStyle('th',fontName='CNBold',fontSize=9,leading=13,textColor=colors.white,wordWrap='CJK'),
 'caption': ParagraphStyle('caption',fontName='CN',fontSize=9,leading=14,alignment=TA_CENTER,spaceAfter=10,wordWrap='CJK'),
 'small': ParagraphStyle('small',fontName='CN',fontSize=9,leading=14,spaceAfter=8,wordWrap='CJK'),
}

def inline(text):
    text = html.escape(text)
    text = re.sub(r'\*\*(.+?)\*\*',r'<b>\1</b>',text)
    text = re.sub(r'`([^`]+)`',r'\1',text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'\1',text)
    return text

def para(text, kind='body'):
    return Paragraph(inline(text),styles[kind])

def table(rows):
    n=len(rows[0]); widths={2:[.36,.64],3:[.23,.42,.35],4:[.17,.30,.30,.23],5:[.15,.22,.22,.20,.21]}.get(n,[1/n]*n)
    # Numeric tables allocate room to units and preserve all recorded digits.
    cells=[[Paragraph(inline(v),styles['th' if i==0 else 'table']) for v in row] for i,row in enumerate(rows)]
    t=Table(cells,colWidths=[CONTENT*w for w in widths],repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([
      ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#233F60')),
      ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#F1F5F9')]),
      ('GRID',(0,0),(-1,-1),.45,colors.HexColor('#D9D9D9')),
      ('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),7),
      ('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
    ])); return t

figures={}
for i,name in enumerate(['01-measurement-error','02-static-variation','03-angle-drift','04-packet-completeness','05-sampling-stability'],1):
    figures[i]=(ROOT/'results/static_20261104_2010/figures'/f'{name}.png',f'图{i} 305秒记录对应指标 设备采样时间与统计窗口见图内')
figures[6]=(ROOT/'submission_build/print_figures/06-main-segment-gyro-response.png','图6 第二轮主展示段角速度响应 原会话23至66.75秒 绝对设备时间461985936至505735936微秒')

def figure(number):
    path,caption=figures[number]
    target=OUT/'evidence'/f'{number:02d}_{path.name}'
    target.write_bytes(path.read_bytes())
    iw,ih=PILImage.open(path).size
    max_h=H-2*MARGIN-47
    width=min(CONTENT,CONTENT if number!=6 else 465)
    height=width*ih/iw
    if height>max_h: width*=max_h/height;height=max_h
    image=Image(str(path),width=width,height=height)
    return KeepTogether([Spacer(1,8),image,para(caption,'caption')])

text=(ROOT/'submission_build/report_sections.md').read_text(encoding='utf-8')
text=text.replace('基于 ESP32 的六轴 IMU 数据采集、滤波与性能评测','基于 ESP32 的六轴 IMU 数据采集滤波与性能评测验收报告')
text=text.replace('轴向验证录像录制于独立设备。当前工程未包含视频原件，第七项影像证据须以归档后的原件验收，不能用曲线、操作确认或相关分析代替。传感器数据本身也不能自动证明外部物理方向。六轴Yaw只相对于初始方向，短时同向跟随不等于绝对航向精度。',
 '轴向验证录像已归档为四个未经剪辑的原视频。主视频 videos/03_continuous_follow.mp4 长64.333秒，同框拍摄手持模块与网页三维模型，可作为不少于30秒的定性连续跟随证据；其中约13至20秒包含X方向抬起与返回。videos/01_y_axis.mp4 长34.067秒，补充倾转与返回；videos/02_z_axis.mp4 长24.600秒，补充水平转向与返回；videos/04_displacement.mp4 长31.200秒，记录位移尝试。所有视频保持原内容，无拼接或变速。代表帧核验可见模型与实体定性同步，但模块丝印不够清晰，不能声称逐帧严格核实每轴正方向。实际验收应播放原片并结合操作说明。六轴Yaw仍是相对方向，短时跟随不等于绝对航向精度。')
text += '\n\n## 提交文件索引\n\n正式验收报告见report/acceptance_report.pdf；可编辑文字源见report/acceptance_report.md。前六项指标图见evidence/，完整数据与选段见results/，ESP32源码见firmware/，上位机源码见web/，构建与采集入口见tools/。binaries/提供本次已验证构建的Flash文件，libraries/保留实际使用的三项离线库及其原许可证，videos/包含四段原视频。README_FIRST.md列出复现方法和文件用途，SHA256SUMS.txt用于检查包内文件完整性。\n'
(OUT/'report/acceptance_report.md').write_text(text,encoding='utf-8')

story=[]; current_metric=None; pending=[]
section_to_figure={4:1,5:2,6:3,7:4,8:5,9:6}
def finish_metric():
    global current_metric
    if current_metric in section_to_figure:
        if current_metric==9:story.append(PageBreak())
        story.append(figure(section_to_figure[current_metric]))
    current_metric=None
def flush():
    if pending:story.append(para(''.join(pending)));pending.clear()

lines=text.splitlines();i=0
while i<len(lines):
    line=lines[i].strip()
    if not line:flush();i+=1;continue
    if line.startswith('# '):
        flush();story.append(para(line[2:],'title'));story.append(para('嵌入式系统期中实验  |  实验日期 2026年11月4日','small'));i+=1;continue
    if line.startswith('## '):
        flush();finish_metric();title=line[3:];match=re.match(r'(\d+)\s',title)
        number=int(match.group(1)) if match else None
        if number in range(4,10):
            if number==4:story.append(PageBreak())
            current_metric=number
        elif number in (10,11,13):story.append(PageBreak())
        title=title.replace('：',' ').replace('、',' ')
        story.append(para(title,'h1'));i+=1;continue
    if line.startswith('### '):
        flush();title=re.sub(r'^\d+(?:\.\d+)*\s*','',line[4:]).replace('：',' ').replace('、',' ')
        story.append(para(title,'h2'));i+=1;continue
    if line.startswith('|'):
        flush();rows=[]
        while i<len(lines) and lines[i].strip().startswith('|'):
            row=[c.strip() for c in lines[i].strip().strip('|').split('|')]
            if not all(re.match(r'^:?-+:?$',c) for c in row):rows.append(row)
            i+=1
        if rows:story.extend([table(rows),Spacer(1,9)])
        continue
    if line.startswith('`') and line.endswith('`'):
        flush();story.append(para(line[1:-1],'formula'));i+=1;continue
    pending.append(line);i+=1
flush();finish_metric()

def footer(c,doc):
    c.saveState();c.setFont('CN',8);c.setFillColor(colors.HexColor('#526071'))
    c.drawString(MARGIN,25,'六轴 IMU 实验验收报告')
    c.drawRightString(W-MARGIN,25,f'{doc.page}');c.restoreState()

pdf=OUT/'report/acceptance_report.pdf'
doc=SimpleDocTemplate(str(pdf),pagesize=letter,rightMargin=MARGIN,leftMargin=MARGIN,topMargin=40,bottomMargin=43,
    title='基于ESP32的六轴IMU数据采集滤波与性能评测验收报告',author='',subject='六项基本任务 七项比较指标 完整实测数据')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(pdf)
