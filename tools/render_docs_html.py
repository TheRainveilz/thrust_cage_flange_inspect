#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_docs_html.py —— 把本仓库的 Markdown 文档渲染成离线可看的静态 HTML。

**生成器零依赖**(只用标准库), 跑在仓库现有的任何 Python 3.8+ 上, 不装任何包、不联网。
把 README.md / docs/*.md / tools/README.md 转成 docs_html/ 下同构的 .html, 文档之间的
`.md` 相对链接自动改写成 `.html`, 双击就能在浏览器里看(表格/代码块/引用/列表都带样式)。

样式走"仪表/测量报告"风(对齐 docs/reports/capability_report.html): 测量青主色、信号绿/琥珀/红
语义色、等宽数字、大写字距标签、侧栏目录 + 主题切换。字体用 Google Fonts 的 IBM Plex Mono /
Noto Sans SC, 但**只是渐进增强**——断网时自动回退到系统字体(font-family 里排了 system-ui/微软雅黑
等兜底), 页面照样能看, 所以"离线可用"不破。想彻底不碰网就把 PAGE 里那两行 <link> 删掉即可。

用法:
    python tools/render_docs_html.py            # 转全部, 输出到 <repo>/docs_html/
    python tools/render_docs_html.py --out DIR  # 换输出目录
    python tools/render_docs_html.py --open     # 转完顺手用默认浏览器打开首页

为什么零依赖手写、而不是 pip 装 markdown:
    这些文档要跟代码一起走, 在没网/没 pip 的产线机上也能随时重渲染。手写解析器**只覆盖本仓库
    文档实际用到的语法子集**(标题/GFM 表格/围栏代码/引用/有序无序列表/加粗/行内码/链接/分隔线),
    刻意不做完整 CommonMark。新写文档若用了这里没覆盖的语法, **补这里**, 别去引依赖。
"""
import argparse
import html
import os
import re
import sys
import webbrowser
from pathlib import Path

CODE_SPAN = re.compile(r"`([^`]+)`")
LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
BOLD = re.compile(r"\*\*([^*]+)\*\*")
LI = re.compile(r"^(\s*)([-*]|\d+\.)\s+(.*)$")


def rewrite_link(url: str) -> str:
    """站内 .md 链接(含 #锚点)改写成 .html; 外链/锚点/非 .md 一律原样。"""
    if url.startswith(("http://", "https://", "mailto:", "#")):
        return url
    m = re.match(r"^(.*?\.md)(#.*)?$", url)
    if m:
        return m.group(1)[:-3] + ".html" + (m.group(2) or "")
    return url


def render_inline(text: str) -> str:
    """行内: 行内码 -> 链接 -> 转义 -> 加粗 -> 还原。占位符用 \\x00 免被转义/格式化误伤。"""
    stash = []

    def put(frag: str) -> str:
        stash.append(frag)
        return "\x00%d\x00" % (len(stash) - 1)

    text = CODE_SPAN.sub(lambda m: put("<code>%s</code>" % html.escape(m.group(1))), text)
    text = LINK.sub(
        lambda m: put('<a href="%s">%s</a>'
                      % (html.escape(rewrite_link(m.group(2)), quote=True), html.escape(m.group(1)))),
        text,
    )
    text = html.escape(text)
    text = BOLD.sub(lambda m: "<strong>%s</strong>" % m.group(1), text)
    ph = re.compile(r"\x00(\d+)\x00")  # 反复还原, 兼容嵌套(如链接文字里含行内码)
    while ph.search(text):
        text = ph.sub(lambda m: stash[int(m.group(1))], text)
    return text


def is_table_sep(s: str) -> bool:
    s = s.strip()
    return bool(s) and set(s) <= set("|:- ") and "-" in s and "|" in s


def split_row(row: str):
    row = row.strip()
    if row.startswith("|"):
        row = row[1:]
    if row.endswith("|"):
        row = row[:-1]
    return [c.strip() for c in row.split("|")]


def render_table(header: str, rows) -> str:
    thead = "<tr>" + "".join("<th>%s</th>" % render_inline(c) for c in split_row(header)) + "</tr>"
    body = "\n".join(
        "<tr>" + "".join("<td>%s</td>" % render_inline(c) for c in split_row(r)) + "</tr>"
        for r in rows
    )
    return "<table><thead>%s</thead><tbody>%s</tbody></table>" % (thead, body)


def render_list(block) -> str:
    """块内每行一项; 缩进(>=2 空格一级)决定嵌套。同一级的有序/无序按该级首项定。"""
    items = []  # [indent, ordered, text]
    for ln in block:
        m = LI.match(ln)
        if m:
            items.append([len(m.group(1)), m.group(2).endswith("."), m.group(3)])
        elif items:  # 续行并入上一项
            items[-1][2] += " " + ln.strip()

    def build(idx, base):
        tag = "ol" if items[idx][1] else "ul"
        parts = ["<%s>" % tag]
        while idx < len(items):
            indent = items[idx][0]
            if indent < base:
                break
            if indent > base:  # 更深一级 -> 塞进上一个 <li>
                sub, idx = build(idx, indent)
                parts[-1] = parts[-1][:-5] + sub + "</li>"
                continue
            parts.append("<li>%s</li>" % render_inline(items[idx][2]))
            idx += 1
        parts.append("</%s>" % tag)
        return "".join(parts), idx

    return build(0, items[0][0])[0] if items else ""


def is_block_start(line: str, nxt: str) -> bool:
    s = line.strip()
    return bool(
        s.startswith("```") or s.startswith(">")
        or re.match(r"^#{1,6}\s", line)
        or re.match(r"^\s*([-*]|\d+\.)\s+", line)
        or re.match(r"^(-{3,}|\*{3,}|_{3,})$", s)
        or ("|" in line and is_table_sep(nxt))
    )


def md_to_html(md: str, collect=True):
    """块级解析。返回 (body_html, title, toc)。title 取首个一级标题(去掉反引号);
    toc 是 [(level, id, inline_html)] 列表(仅 h2/h3, 供侧栏目录用)。collect=False 时
    (如 blockquote 递归)不派发 id、不收 toc, 避免嵌套里的标题污染主目录。"""
    lines = md.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    out, title, i, n = [], None, 0, len(lines)
    toc, hid = [], [0]
    while i < n:
        line = lines[i]
        s = line.strip()
        if s.startswith("```"):  # 围栏代码: 内部一律不解析
            lang, j, buf = s[3:].strip(), i + 1, []
            while j < n and lines[j].strip() != "```":
                buf.append(lines[j]); j += 1
            cls = ' class="language-%s"' % html.escape(lang, quote=True) if lang else ""
            out.append("<pre><code%s>%s</code></pre>" % (cls, html.escape("\n".join(buf))))
            i = j + 1
            continue
        if s == "":
            i += 1
            continue
        m = re.match(r"^(#{1,6})\s+(.*)$", line)
        if m:
            lvl, txt = len(m.group(1)), m.group(2).strip()
            if title is None and lvl == 1:
                title = txt.replace("`", "")
            inner = render_inline(txt)
            if collect and 1 <= lvl <= 4:
                hid[0] += 1
                hid_s = "s%d" % hid[0]
                if lvl in (2, 3):
                    toc.append((lvl, hid_s, inner))
                out.append('<h%d id="%s">%s<a class="permalink" href="#%s" '
                           'aria-hidden="true">#</a></h%d>' % (lvl, hid_s, inner, hid_s, lvl))
            else:
                out.append("<h%d>%s</h%d>" % (lvl, inner, lvl))
            i += 1
            continue
        if re.match(r"^(-{3,}|\*{3,}|_{3,})$", s):
            out.append("<hr>")
            i += 1
            continue
        if "|" in line and i + 1 < n and is_table_sep(lines[i + 1]):
            header, i = line, i + 2
            rows = []
            while i < n and "|" in lines[i] and lines[i].strip():
                rows.append(lines[i]); i += 1
            out.append(render_table(header, rows))
            continue
        if s.startswith(">"):
            buf = []
            while i < n and lines[i].strip().startswith(">"):
                buf.append(re.sub(r"^\s*>\s?", "", lines[i])); i += 1
            inner_md = "\n".join(buf)
            first = inner_md.lstrip()
            # 以 ⚠ / ！开头的引用当作"警告 callout"高亮(这些文档里全是安全铁律)
            cls = " warn" if first[:1] in ("⚠", "!", "！") else ""
            out.append('<blockquote class="callout%s">%s</blockquote>'
                       % (cls, md_to_html(inner_md, collect=False)[0]))
            continue
        if re.match(r"^\s*([-*]|\d+\.)\s+", line):
            block = []
            while i < n and (re.match(r"^\s*([-*]|\d+\.)\s+", lines[i])
                             or (block and lines[i].startswith("  ") and lines[i].strip())):
                block.append(lines[i]); i += 1
            out.append(render_list(block))
            continue
        buf = [line]  # 段落
        i += 1
        while i < n and lines[i].strip() and not is_block_start(lines[i], lines[i + 1] if i + 1 < n else ""):
            buf.append(lines[i]); i += 1
        out.append("<p>%s</p>" % render_inline(" ".join(x.strip() for x in buf)))
    return "\n".join(out), (title or "Document"), toc


CSS = """
:root{
  color-scheme: light dark;
  --ground:#eef1f4; --panel:#ffffff; --panel-2:#f6f8fa;
  --ink:#1a2028; --ink-2:#4a5563; --ink-3:#7a8794;
  --line:#d6dde4; --line-soft:#e7ecf1; --line-strong:#b9c3cc;
  --accent:#0e8ea3; --accent-soft:#d3ebef;
  --safe:#12855a; --safe-soft:#d3ede0;
  --warn:#b5731a; --warn-soft:#f2e4cf;
  --danger:#c0392b; --danger-soft:#f3d9d5;
  --shadow:0 1px 2px rgba(26,32,40,.06),0 8px 24px rgba(26,32,40,.06);
  --radius:10px;
  --mono:"IBM Plex Mono",ui-monospace,"SFMono-Regular",Consolas,"Cascadia Code",monospace;
  --sans:"Noto Sans SC","Segoe UI",system-ui,"Microsoft YaHei","PingFang SC",sans-serif;
  --topbar:rgba(255,255,255,.85);
}
@media (prefers-color-scheme:dark){ :root:not([data-theme="light"]){
  --ground:#0d1116; --panel:#151b22; --panel-2:#1b232c;
  --ink:#e7edf3; --ink-2:#a9b6c2; --ink-3:#71808d;
  --line:#28323d; --line-soft:#1f2831; --line-strong:#3a4754;
  --accent:#3bc0d6; --accent-soft:#123842;
  --safe:#3fcf8e; --safe-soft:#123528;
  --warn:#e0a44a; --warn-soft:#3a2c14;
  --danger:#f0685a; --danger-soft:#3a1c18;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 30px rgba(0,0,0,.35);
  --topbar:rgba(13,17,22,.85);
}}
:root[data-theme="dark"]{
  --ground:#0d1116; --panel:#151b22; --panel-2:#1b232c;
  --ink:#e7edf3; --ink-2:#a9b6c2; --ink-3:#71808d;
  --line:#28323d; --line-soft:#1f2831; --line-strong:#3a4754;
  --accent:#3bc0d6; --accent-soft:#123842;
  --safe:#3fcf8e; --safe-soft:#123528;
  --warn:#e0a44a; --warn-soft:#3a2c14;
  --danger:#f0685a; --danger-soft:#3a1c18;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 30px rgba(0,0,0,.35);
  --topbar:rgba(13,17,22,.85);
}
*{box-sizing:border-box}
html{scroll-behavior:smooth}
body{margin:0;background:var(--ground);color:var(--ink);font-family:var(--sans);
  line-height:1.72;-webkit-font-smoothing:antialiased}
.mono{font-family:var(--mono);font-variant-numeric:tabular-nums}

hr{border:0;border-top:1px solid var(--line);margin:2.4em 0}
ul,ol{padding-left:1.5em}
li{margin:.32em 0}
li::marker{color:var(--accent)}
.foot{margin-top:42px;padding-top:18px;border-top:1px solid var(--line-soft);
  color:var(--ink-3);font-size:12.5px;font-family:var(--mono);letter-spacing:.03em}
/* 首页卡片 */
.lead{color:var(--ink-2);font-size:16px;margin:.2em 0 1.8em;max-width:66ch;text-wrap:balance}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(272px,1fr));gap:16px;
  margin:.6em 0 1.6em}
.card{display:block;padding:20px 20px 18px;border:1px solid var(--line);border-radius:var(--radius);
  background:var(--panel);text-decoration:none;color:inherit;box-shadow:var(--shadow);
  border-top:3px solid var(--accent);transition:transform .16s,border-color .16s}
.card:hover{transform:translateY(-3px);border-color:var(--accent);text-decoration:none}
.card .card-t{font-weight:700;font-size:1.06em;color:var(--ink)}
.card:hover .card-t{color:var(--accent)}
.card .card-d{color:var(--ink-2);font-size:13.5px;margin:9px 0 13px;line-height:1.55}
.card .card-p{font-family:var(--mono);font-size:.8em;color:var(--ink-3);background:var(--panel-2);
  border:1px solid var(--line-soft);border-radius:6px;padding:3px 9px;display:inline-block}

/* 引用 / callout */
blockquote{margin:1.15em 0;padding:.7em 1.1em;color:var(--ink-2);background:var(--panel-2);
  border:1px solid var(--line);border-left:3px solid var(--accent);border-radius:0 8px 8px 0}
blockquote p{margin:.4em 0}
blockquote p:first-child{margin-top:0} blockquote p:last-child{margin-bottom:0}
blockquote code{background:var(--panel)}
blockquote.warn{color:var(--warn);background:var(--warn-soft);border-left-color:var(--warn)}
blockquote.warn strong{color:var(--warn)}
blockquote.warn code{background:rgba(181,115,26,.14);color:var(--warn);border-color:transparent}
/* 表格 */
table{width:100%;border-collapse:collapse;margin:1.3em 0;display:block;overflow-x:auto;
  border:1px solid var(--line);border-radius:var(--radius)}
th,td{padding:10px 15px;text-align:left;vertical-align:top;
  border-bottom:1px solid var(--line-soft);border-right:1px solid var(--line-soft)}
tr td:last-child,tr th:last-child{border-right:0}
tbody tr:last-child td{border-bottom:0}
th{font-family:var(--mono);font-size:11px;font-weight:500;letter-spacing:.07em;
  text-transform:uppercase;color:var(--ink-3);background:var(--panel-2);
  border-bottom:1px solid var(--line-strong);white-space:nowrap}
td{font-size:14.5px}
td code{font-size:.9em}
tbody tr:nth-child(2n) td{background:var(--panel-2)}
tbody tr:hover td{background:var(--accent-soft)}

/* 正文面板 */
.content{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);
  padding:10px 42px 42px;box-shadow:var(--shadow);min-width:0}
.content>*:first-child{margin-top:26px}
h1,h2,h3,h4{line-height:1.28;margin:1.7em 0 .6em;scroll-margin-top:80px;letter-spacing:-.01em;
  text-wrap:balance}
h1{font-size:clamp(26px,4.4vw,38px);font-weight:900;padding-bottom:.32em;
  border-bottom:2px solid var(--line);margin-top:.2em}
h2{font-size:1.4em;font-weight:700;padding-bottom:.26em;border-bottom:1px solid var(--line-soft);
  display:flex;align-items:baseline;gap:.5em}
h2::before{content:"";width:9px;height:9px;border-radius:2px;background:var(--accent);
  transform:translateY(-1px);flex:none}
h3{font-size:1.16em;font-weight:700}
h4{font-size:1em;font-weight:600;color:var(--ink-2)}
p{margin:.85em 0}
a{color:var(--accent);text-decoration:none}
a:hover{text-decoration:underline}
.permalink{margin-left:.4em;color:var(--line-strong);text-decoration:none;font-weight:400;
  font-family:var(--mono);opacity:0;transition:opacity .15s}
h1:hover .permalink,h2:hover .permalink,h3:hover .permalink,h4:hover .permalink{opacity:1}
.permalink:hover{color:var(--accent);text-decoration:none}
code{font-family:var(--mono);font-size:.86em;background:var(--panel-2);color:var(--accent);
  padding:.14em .42em;border-radius:6px;border:1px solid var(--line-soft)}
pre{background:var(--panel-2);border:1px solid var(--line);border-radius:var(--radius);
  padding:16px 18px;overflow-x:auto;margin:1.15em 0}
pre code{background:none;border:0;color:var(--ink);padding:0;font-size:.85em;line-height:1.62}

/* 顶栏 */
.topbar{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:14px;
  height:56px;padding:0 20px;background:var(--topbar);backdrop-filter:saturate(1.6) blur(10px);
  border-bottom:1px solid var(--line)}
.topbar .brand{font-family:var(--mono);font-size:12px;font-weight:600;letter-spacing:.14em;
  text-transform:uppercase;color:var(--ink);text-decoration:none;white-space:nowrap}
.topbar .brand:hover{color:var(--accent)}
.topbar .crumb{font-family:var(--mono);font-size:12px;color:var(--ink-3);letter-spacing:.03em;
  overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.topbar .crumb::before{content:"//";margin-right:10px;color:var(--line-strong)}
.topbar .theme{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--panel);
  color:var(--ink-2);width:34px;height:34px;border-radius:8px;cursor:pointer;font-size:15px;
  line-height:1;display:inline-flex;align-items:center;justify-content:center}
.topbar .theme:hover{border-color:var(--accent);color:var(--accent)}
.menu{display:none;flex:none;border:1px solid var(--line);background:var(--panel);font-size:16px;
  color:var(--ink);cursor:pointer;width:34px;height:34px;line-height:1;border-radius:8px;
  align-items:center;justify-content:center}
.menu:hover{border-color:var(--accent);color:var(--accent)}
/* 布局 */
.layout{max-width:1200px;margin:0 auto;display:grid;grid-template-columns:248px minmax(0,1fr);
  gap:36px;padding:34px 22px 100px;align-items:start}
.layout.solo{grid-template-columns:minmax(0,1fr);max-width:960px}
/* 侧栏目录 */
.side{position:sticky;top:80px;max-height:calc(100vh - 104px);overflow:auto;padding-right:4px}
.toc .toc-h{display:flex;align-items:center;gap:9px;margin:0 0 12px;font-family:var(--mono);
  font-size:11px;font-weight:600;letter-spacing:.16em;text-transform:uppercase;color:var(--accent)}
.toc .toc-h::before{content:"";width:20px;height:2px;background:var(--accent);border-radius:2px}
.toc ul{list-style:none;margin:0;padding:0}
.toc .sub{margin:2px 0 6px 14px;border-left:1px solid var(--line)}
.toc a{display:block;padding:5px 11px;color:var(--ink-3);text-decoration:none;font-size:13px;
  line-height:1.5;border-radius:7px;border-left:2px solid transparent;overflow:hidden;
  text-overflow:ellipsis}
.toc a code{font-family:var(--mono);font-size:.9em}
.toc a:hover{color:var(--ink);background:var(--panel-2)}
.toc a.active{color:var(--accent);background:var(--accent-soft);border-left-color:var(--accent);
  font-weight:600}

/* 移动端：抽屉式侧栏 + 汉堡菜单（放最后，靠源顺序压过上面的基础规则） */
@media (max-width:860px){
  .menu{display:inline-flex}
  .layout{grid-template-columns:minmax(0,1fr);gap:0}
  .side{position:fixed;top:56px;left:0;bottom:0;width:284px;z-index:30;background:var(--panel);
    border-right:1px solid var(--line);max-height:none;padding:20px;transform:translateX(-100%);
    transition:transform .2s;box-shadow:var(--shadow)}
  body.nav-open .side{transform:none}
  body.nav-open::after{content:"";position:fixed;inset:56px 0 0 0;background:rgba(0,0,0,.4);z-index:25}
  .content{padding:6px 22px 32px}
}
"""

PAGE = ("<!doctype html>\n<html lang=\"zh-CN\">\n<head>\n<meta charset=\"utf-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
        "<title>%(title)s</title>\n"
        "<link rel=\"preconnect\" href=\"https://fonts.googleapis.com\">\n"
        "<link rel=\"preconnect\" href=\"https://fonts.gstatic.com\" crossorigin>\n"
        "<link rel=\"stylesheet\" href=\"https://fonts.googleapis.com/css2?"
        "family=IBM+Plex+Mono:wght@400;500;600&family=Noto+Sans+SC:wght@400;500;700;900&display=swap\">\n"
        "<script>%(headjs)s</script>\n"
        "<style>%(css)s</style>\n</head>\n<body>\n"
        "<header class=\"topbar\">%(menu)s"
        "<a class=\"brand\" href=\"%(home)s\">推力保持架 · 双站检测线</a>"
        "<span class=\"crumb\">%(crumb)s</span>"
        "<button class=\"theme\" id=\"theme\" type=\"button\" aria-label=\"切换明暗主题\">\u25d0</button>"
        "</header>\n"
        "<div class=\"layout%(solo)s\">%(side)s"
        "<main class=\"content\">\n%(body)s\n"
        "<div class=\"foot\">GENERATED BY tools/render_docs_html.py · 离线静态 · 零依赖生成</div>"
        "</main></div>\n%(script)s\n</body>\n</html>\n")

# 头部提前应用已存主题, 避免加载时明暗闪一下
HEAD_THEME = ("try{var _t=localStorage.getItem('doc-theme');"
              "if(_t&&_t!=='auto')document.documentElement.setAttribute('data-theme',_t);}catch(e){}")

SCRIPT = ("<script>\n(function(){\n"
          " var root=document.documentElement;\n"
          " function gt(){try{return localStorage.getItem('doc-theme')||'auto'}catch(e){return 'auto'}}\n"
          " function st(v){try{localStorage.setItem('doc-theme',v)}catch(e){}}\n"
          " var tb=document.getElementById('theme');\n"
          " function lab(v){return v==='dark'?'\\u263e':v==='light'?'\\u2600':'\\u25d0';}\n"
          " function ap(v){if(v==='auto')root.removeAttribute('data-theme');"
          "else root.setAttribute('data-theme',v);if(tb)tb.textContent=lab(v);}\n"
          " ap(gt());\n"
          " if(tb)tb.addEventListener('click',function(){"
          "var o=['auto','dark','light'],v=o[(o.indexOf(gt())+1)%3];st(v);ap(v);});\n"
          " var mn=document.getElementById('menu');\n"
          " if(mn)mn.addEventListener('click',function(){document.body.classList.toggle('nav-open');});\n"
          " var links=[].slice.call(document.querySelectorAll('.toc a'));\n"
          " if(!links.length||!('IntersectionObserver' in window))return;\n"
          " var map={};links.forEach(function(a){map[a.getAttribute('href').slice(1)]=a;});\n"
          " links.forEach(function(a){a.addEventListener('click',function(){"
          "document.body.classList.remove('nav-open');});});\n"
          " var seen=[];\n"
          " var ob=new IntersectionObserver(function(es){\n"
          "  es.forEach(function(e){var i=seen.indexOf(e.target.id);\n"
          "   if(e.isIntersecting){if(i<0)seen.push(e.target.id);}else if(i>=0)seen.splice(i,1);});\n"
          "  if(seen.length){links.forEach(function(l){l.classList.remove('active');});\n"
          "   var a=map[seen[0]];if(a)a.classList.add('active');}\n"
          " },{rootMargin:'-70px 0px -75% 0px'});\n"
          " document.querySelectorAll('.content h2[id],.content h3[id]').forEach(function(h){ob.observe(h);});\n"
          "})();\n</script>")


def build_toc(toc) -> str:
    """toc: [(level, id, inline_html)] -> 侧栏目录 <nav>。h3 收进本级 h2 下方的子列表。"""
    if not toc:
        return ""
    out = ['<aside class="side"><nav class="toc"><p class="toc-h">本页目录</p><ul>']
    depth = 2
    for lvl, hid, txt in toc:
        if lvl == 3 and depth == 2:
            out.append('<ul class="sub">'); depth = 3
        elif lvl == 2 and depth == 3:
            out.append("</ul>"); depth = 2
        out.append('<li><a href="#%s">%s</a></li>' % (hid, txt))
    if depth == 3:
        out.append("</ul>")
    out.append("</ul></nav></aside>")
    return "".join(out)


def wrap_html(title: str, body: str, index_href: str, toc=None) -> str:
    side = build_toc(toc or [])
    home = html.escape(index_href or "index.html", quote=True)
    menu = '<button class="menu" id="menu" type="button" aria-label="目录">&#9776;</button>' if side else ""
    return PAGE % {"title": html.escape(title), "css": CSS, "home": home,
                   "crumb": html.escape(title), "menu": menu, "side": side,
                   "solo": "" if side else " solo", "headjs": HEAD_THEME,
                   "body": body, "script": SCRIPT}


REPO_ROOT = Path(__file__).resolve().parent.parent
GROUPS = [(".", "主文档"), ("docs", "说明文档"), ("tools", "工具索引")]

# 首页卡片一句话简介(找不到就留空)——纯展示用, 不参与真值
DESCS = {
    "README.md": "整线总览：双站架构、安全铁律、跑起来、目录日志、硬件链、现场排查。",
    "docs/camera_config.md": "第一站相机成像标定实验记录（带日期变更账本）。",
    "docs/dbg_report.md": "第一站阈值调优外挂 dbg_report 的完整说明。",
    "docs/missing_camera_config.md": "第二站（缺粒）相机成像记录 + .202 现场标定清单。",
    "docs/tuning_notes.md": "第一站算法调参史 / 证伪账：为什么这么选、试过哪些没走通。",
    "tools/README.md": "tools 目录索引：验证闸 / 硬件调试 / 调参诊断 / 文档渲染。",
}


def collect_docs():
    """要渲染的 .md 集合(存在才收): README.md / docs/*.md / tools/README.md。"""
    rels = ["README.md"] + sorted(
        (p.relative_to(REPO_ROOT).as_posix() for p in (REPO_ROOT / "docs").glob("*.md"))
    ) + ["tools/README.md"]
    return [r for r in rels if (REPO_ROOT / r).is_file()]


def build_index(entries) -> str:
    """entries: [(rel, title, html_relhref)]。按 GROUPS 分组, 每篇一张卡片。"""
    by_dir = {}
    for rel, title, href in entries:
        top = rel.split("/")[0] if "/" in rel else "."
        by_dir.setdefault(top, []).append((title, href, rel))
    parts = ["<h1>文档索引</h1>",
             '<p class="lead">金属推力保持架垫片 · 双站视觉检测线 —— 离线文档集。'
             '由 <code>tools/render_docs_html.py</code> 零依赖生成，双击即看、断网可用。</p>']
    for top, label in GROUPS:
        if top not in by_dir:
            continue
        parts.append("<h2>%s</h2>\n<div class=\"cards\">" % label)
        for title, href, rel in by_dir[top]:
            desc = DESCS.get(rel, "")
            parts.append(
                '<a class="card" href="%s"><div class="card-t">%s</div>'
                '%s<div class="card-p">%s</div></a>'
                % (html.escape(href, quote=True), html.escape(title),
                   ('<div class="card-d">%s</div>' % html.escape(desc)) if desc else "",
                   html.escape(rel)))
        parts.append("</div>")
    return "\n".join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description="把仓库 Markdown 文档渲染成离线静态 HTML")
    ap.add_argument("--out", default=str(REPO_ROOT / "docs_html"), help="输出目录(默认 <repo>/docs_html)")
    ap.add_argument("--open", action="store_true", help="转完用默认浏览器打开首页")
    args = ap.parse_args(argv)

    out_root = Path(args.out).resolve()
    docs = collect_docs()
    if not docs:
        print("没找到任何 .md, 退出", file=sys.stderr)
        return 2

    entries = []
    for rel in docs:
        src = REPO_ROOT / rel
        dst = out_root / Path(rel).with_suffix(".html")
        dst.parent.mkdir(parents=True, exist_ok=True)
        body, title, toc = md_to_html(src.read_text(encoding="utf-8"))
        index_href = os.path.relpath(out_root / "index.html", dst.parent).replace(os.sep, "/")
        dst.write_text(wrap_html(title, body, index_href, toc), encoding="utf-8")
        entries.append((rel, title, Path(rel).with_suffix(".html").as_posix()))
        print("  %-28s -> %s" % (rel, dst.relative_to(out_root)))

    index = out_root / "index.html"
    index.write_text(wrap_html("文档索引", build_index(entries), "index.html"), encoding="utf-8")
    print("共 %d 篇 + 首页 -> %s" % (len(entries), index))
    if args.open:
        webbrowser.open(index.as_uri())
    return 0


if __name__ == "__main__":
    sys.exit(main())




