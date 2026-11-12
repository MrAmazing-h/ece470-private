#!/usr/bin/env python3
"""Export the current Lab 5 code/configuration as a GradeScope code document."""

from datetime import datetime, timezone
import html
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'ece470lab/lab_files/src/ece470labs'
FILES=[PACKAGE/'ece470labs'/name for name in
       ['blob_search.py','lab5_func.py','lab5_exec.py','lab5_calibrate.py','lab_func.py']]
FILES += [PACKAGE/'package.xml',PACKAGE/'setup.py',ROOT/'lab5/config.json']


def main():
    sections=[]
    for path in FILES:
        relative=str(path.relative_to(ROOT))
        sections.append('<section><h2>'+html.escape(relative)+'</h2><pre>'+
                        html.escape(path.read_text())+'</pre></section>')
    document='''<!doctype html><html lang="en"><meta charset="utf-8">
<title>ECE 470 Lab 5 Code Submission</title><style>
@page { size: letter; margin: .7in; }
body { font: 11pt serif; } h1 { font-size: 18pt; } h2 { font: 11pt monospace; overflow-wrap: anywhere; }
section { break-before: page; }
pre { white-space: pre-wrap; overflow-wrap: anywhere; font: 8.5pt/1.2 monospace; }
</style><h1>ECE 470 Lab 5 Code Submission</h1>
<p>Name: [fill in] &nbsp; NetID: xliu173<br>Partner: [fill in] &nbsp; TA: [fill in]<br>Section: [fill in]</p>
<p>Includes the vision code, camera calibration fitter, Lab 5 task/ROS code,
shared Lab 4 kinematics, package metadata, and current bench configuration.
Regenerate this document after final calibration and tuning.</p>
<p>Generated UTC: '''+datetime.now(timezone.utc).isoformat()+'</p>'+''.join(sections)+'</html>'
    output=ROOT/'lab5/Submission_Code.html'
    output.write_text(document)
    chrome=shutil.which('google-chrome') or shutil.which('chromium')
    if chrome:
        completed=subprocess.run([chrome,'--headless','--no-sandbox','--disable-gpu',
                                  '--no-pdf-header-footer','--print-to-pdf='+str(output.with_suffix('.pdf')),
                                  output.as_uri()],capture_output=True,text=True,timeout=45)
        if completed.returncode:
            raise RuntimeError(completed.stderr[-1000:])
    print('Code document:',output)
    if chrome:
        print('PDF:',output.with_suffix('.pdf'))


if __name__=='__main__':
    main()
