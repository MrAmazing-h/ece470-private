#!/usr/bin/env python3
"""Regenerate original diagrams, FK checks, and the report from measurements."""

import csv
import html
from pathlib import Path
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'ece470lab/lab_files/src/ece470labs'))
from ece470labs.lab_func import forward_kinematics, lab_ik_angles  # noqa: E402

POSES = [
    ('P1', .1, .1, .15, 90),
    ('P2', .2, 0, .1, 0),
    ('P3', 0, .25, .2, -90),
    ('P4', -.3, .2, .15, 45),
    ('P5', -.3, -.05, .12, 180),
    ('P6', .15, .2, .12, 135),
]
FIELDS = ['id', 'x_m', 'y_m', 'z_m', 'yaw_deg',
          'measured_x_m', 'measured_y_m', 'measured_z_m']


def arrow(ax, start, end, label, color, offset=(0, 0)):
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='->', color=color, lw=1.8))
    midpoint = (np.array(start) + end) / 2 + offset
    ax.text(*midpoint, label, color=color, fontsize=11,
            bbox=dict(facecolor='white', edgecolor='none', alpha=.85))


def save_figure(fig, name):
    svg = HERE / 'figures' / (name + '.svg')
    fig.savefig(svg, bbox_inches='tight')
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines()) + '\n')
    fig.savefig(HERE / 'figures' / (name + '.png'), dpi=180, bbox_inches='tight')
    plt.close(fig)


def make_figures():
    (HERE / 'figures').mkdir(exist_ok=True)
    plt.rcParams.update({'font.size': 11, 'svg.fonttype': 'none'})
    fig, ax = plt.subplots(figsize=(6.5, 2.3))
    ow, ob = np.array([0., 0.]), np.array([-.150, .150])
    for origin, name in [(ow, '$O_W$'), (ob, '$O_0$')]:
        ax.scatter(*origin, color='#174a7e')
        ax.text(*(origin + [.008, -.018]), name)
        arrow(ax, origin, origin + [.09, 0], '$x$', '#a64238', (0, .008))
        arrow(ax, origin, origin + [0, .09], '$y$', '#36714d', (.008, 0))
    arrow(ax, ow, ob, '', '#555555')
    ax.text(-.30, .035, '(-0.150, +0.150) m', fontsize=10)
    ax.text(.07, .17, 'Aligned axes; $z$ points out of the table.\nBase origin is 0.010 m above the world plane.', fontsize=11)
    ax.set(xlim=(-.3, .45), ylim=(-.04, .27), aspect='equal')
    ax.axis('off')
    save_figure(fig, 'world_base')

    # Top view in base coordinates, shown in a conventional x-right/y-up plot.
    theta, r, d, b, plate, yaw = .45, .21, .131, .085, .0535, 1.0
    er = np.array([np.cos(theta), np.sin(theta)])
    et = np.array([-np.sin(theta), np.cos(theta)])
    origin = np.zeros(2)
    p3 = r * er
    a = (r + b) * er
    center = a + d * et
    grip = center + plate * np.array([np.cos(yaw), np.sin(yaw)])
    fig, ax = plt.subplots(figsize=(6.5, 3.3))
    ax.plot([0, a[0]], [0, a[1]], '--', color='#999999')
    ax.plot([0, center[0]], [0, center[1]], ':', color='#999999')
    arrow(ax, origin, [.12, 0], '$x_0$', '#777777', (0, -.028))
    arrow(ax, origin, [0, .12], '$y_0$', '#777777', (-.03, 0))
    arrow(ax, p3, a, '$b=0.085$', '#36714d', (0, -.04))
    arrow(ax, a, center, '$d=0.131$', '#174a7e', (.018, 0))
    arrow(ax, center, grip, '$p=0.0535$', '#a64238', (.035, -.01))
    for p, label, off in [(origin, '$O_0$', (-.025, -.025)),
                          (p3, '$P_3$', (-.04, .015)),
                          (center, '$C$', (-.026, .012)),
                          (grip, '$G$', (.008, .005))]:
        ax.scatter(*p, s=30, color='#222222')
        ax.text(*(p+off), label)
    arc = np.linspace(0, theta, 40)
    ax.plot(.075*np.cos(arc), .075*np.sin(arc), color='#555555')
    ax.text(.09, .016, '$\u03b8_1$')
    ax.set(xlim=(-.06, .39), ylim=(-.06, .36), aspect='equal')
    ax.axis('off')
    save_figure(fig, 'top_view')

    t2, t3 = -.95, 1.4
    shoulder = np.array([0., .152])
    elbow = shoulder + .244*np.array([np.cos(t2), -np.sin(t2)])
    p3 = elbow + .213*np.array([np.cos(t2+t3), -np.sin(t2+t3)])
    wrist = p3 + [.085, 0]
    center = wrist - [0, .151]
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    ax.plot([0, shoulder[0], elbow[0], p3[0], wrist[0]],
            [0, shoulder[1], elbow[1], p3[1], wrist[1]],
            '-o', color='#174a7e', lw=2.7)
    ax.plot([wrist[0], center[0]], [wrist[1], center[1]], color='#a64238', lw=2)
    ax.plot([0, p3[0]], [shoulder[1], shoulder[1]], '--', color='#999999')
    ax.plot([0, p3[0]], [shoulder[1], p3[1]], ':', color='#999999')
    ax.plot([p3[0], p3[0]], [shoulder[1], p3[1]], '--', color='#999999')
    for p, name, off in [(shoulder, 'Shoulder', (-.07, -.035)),
                         (elbow, 'Elbow', (-.02, .02)),
                         (p3, '$P_3$', (-.005, .02)),
                         (center, '$C$ (cup height)', (.01, -.01))]:
        ax.text(*(p+off), name)
    ax.text(-.04, .29, '$a=0.244$ m', fontsize=10)
    ax.text(.26, .34, '$c=0.213$ m', fontsize=10)
    arc = np.linspace(0, -t2, 40)
    ax.plot(shoulder[0]+.055*np.cos(arc), shoulder[1]+.055*np.sin(arc), color='#555555')
    ax.text(.06, .17, '$-\\theta_2$', fontsize=10)
    arc = np.linspace(-t2, -t2-t3, 40)
    ax.plot(elbow[0]+.05*np.cos(arc), elbow[1]+.05*np.sin(arc), color='#555555')
    ax.text(elbow[0]+.055, elbow[1], '$\\theta_3$', fontsize=10)
    ax.text(.26, .12, '$r$', fontsize=13)
    ax.text(p3[0]+.012, .2, '$h$', fontsize=13)
    ax.text(wrist[0]+.012, .25, '$v=0.151$ m', fontsize=10)
    ax.text(.04, .025, 'Side view in the radial / vertical plane')
    ax.set(xlim=(-.08, .57), ylim=(0, .43), aspect='equal')
    ax.axis('off')
    save_figure(fig, 'side_view')


def report():
    make_figures()
    data_file = HERE / 'measurements.csv'
    if not data_file.exists():
        with data_file.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            writer.writerows([list(pose) + ['', '', ''] for pose in POSES])
    with data_file.open(newline='') as stream:
        data = list(csv.DictReader(stream))
    numerical, measured, results, collected = [], [], [], []
    max_error = 0.0
    count = 0
    with (HERE / 'numerical_validation.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['id', 'x_m', 'y_m', 'z_m', 'yaw_deg',
                         'fk_x_m', 'fk_y_m', 'fk_z_m', 'fk_residual_m',
                         *['encoder_'+str(i)+'_deg' for i in range(1, 7)]])
        for row in data:
            ident = html.escape(row['id'])
            pose = [float(row[key]) for key in FIELDS[1:5]]
            angles = lab_ik_angles(*pose)
            transform = forward_kinematics(*angles)
            fk = transform[:3, 3] / 1000
            error = float(np.linalg.norm(fk-pose[:3]))
            max_error = max(error, max_error)
            encoder = np.degrees(np.array(angles) + [np.pi, 0, 0, -np.pi/2, 0, 0])
            writer.writerow([row['id'], *pose, *fk, error, *encoder])
            target = '(' + ', '.join(f'{x:.3f}' for x in pose[:3]) + ')'
            numerical.append(f'<tr><td>{ident}</td><td>{target}</td>'
                             f'<td>{pose[3]:g}</td><td>{error:.2e}</td></tr>')
            values = [row[key].strip() for key in FIELDS[5:]]
            if any(values) and not all(values):
                raise ValueError('Fill all three measured coordinates for '+row['id'])
            if all(values):
                actual = np.array([float(x) for x in values])
                if not np.all(np.isfinite(actual)):
                    raise ValueError('Measurements must be finite: '+row['id'])
                measured_error = float(np.linalg.norm(actual-pose[:3]))
                actual_text = '(' + ', '.join(f'{x:.3f}' for x in actual) + ')'
                error_text = f'{measured_error*1000:.3f}'
                collected.append((ident, actual-np.array(pose[:3]), measured_error))
                count += 1
            else:
                measured_error, actual_text, error_text = '', 'Pending', 'Pending'
            measured.append(f'<tr><td>{ident}</td><td>{actual_text}</td><td>{error_text}</td></tr>')
            results.append({**row, 'scalar_error_m': measured_error})
    with (HERE / 'measurement_results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS+['scalar_error_m'])
        writer.writeheader()
        writer.writerows(results)
    status = (f'{count} of {len(data)} physical measurements are recorded. '
              'Complete any remaining measurements and the identifying information before submission.')
    if collected:
        lowest = min(collected, key=lambda item: item[2])
        highest = max(collected, key=lambda item: item[2])
        mean_error = np.mean([item[2] for item in collected]) * 1000
        residuals = '; '.join(
            f'{ident}: (' + ', '.join(f'{v*1000:+.1f}' for v in delta) + ') mm'
            for ident, delta, error in collected)
        physical_discussion = (
            f'The measured-minus-target position residuals are {residuals}. '
            f'The smallest scalar error is {lowest[2]*1000:.3f} mm at {lowest[0]}, '
            f'and the largest is {highest[2]*1000:.3f} mm at {highest[0]}. '
            f'The mean scalar error is {mean_error:.3f} mm. '
            'Position errors are much larger than the numerical IK/FK residuals. '
            'The available measurements do not isolate a dominant error source. '
            'Yaw was commanded but was not independently measured numerically.')
        physical_conclusion = (
            f'The recorded position errors range from {lowest[2]*1000:.3f} '
            f'to {highest[2]*1000:.3f} mm. '
            'The supplied terminal logs for P1 and P2 both report that the requested '
            'joint pose was reached. This confirms the executable reported motion '
            'completion; TA assessment and independent yaw accuracy are not '
            'established by those messages.')
    else:
        physical_discussion = 'Physical measurements have not yet been recorded.'
        physical_conclusion = 'Physical positioning performance remains to be measured.'
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<title>ECE 470 Lab 4 — Inverse Kinematics</title><style>
@page { size: letter; margin: 1in; }
body { font: 12pt "Times New Roman", "Liberation Serif", serif; line-height: 1.2; color: #111; margin: 0; }
h1 { font-size: 17pt; margin: 0 0 10pt; } h2 { font-size: 14pt; margin: 12pt 0 6pt; }
h3 { font-size: 12pt; margin: 10pt 0 4pt; } p { margin: 6pt 0; }
.page { break-after: page; } .page:last-child { break-after: auto; }
.eq { margin: 8pt 0; text-align: center; } figure { margin: 9pt 0; text-align: center; }
figure img { max-width: 100%; } figcaption { font-size: 11pt; }
table { width: 100%; border-collapse: collapse; margin: 7pt 0; font-size: 11pt; }
th, td { border: 1px solid #777; padding: 3pt; text-align: center; }
caption { caption-side: top; margin: 4pt; } .draft { border: 1px solid #777; padding: 7pt; }
code, pre { font: 9pt monospace; white-space: pre-wrap; } a { color: #174a7e; }
</style><body>
<section class="page">
<h1>ECE 470: Lab 4 — Inverse Kinematics</h1>
<p>Name: [fill in] &nbsp; NetID: xliu173<br>Partner: [fill in] &nbsp; TA: [fill in]<br>
Section: [fill in] &nbsp; Date submitted: [fill in]</p>
<p class="draft"><b>Draft — identifying information and TA review pending.</b> STATUS</p>
<h2>1 Introduction</h2>
<p>This lab develops an analytical elbow-up inverse-kinematics solution for the UR3e.
The input is a suction-cup position in the table's world frame and a yaw angle.
The remaining orientation is fixed: θ<sub>5</sub> = −π/2 and the wrist links are
kept parallel to the table. The computed angles are checked with the Lab 3
product-of-exponentials forward model before conversion to robot encoder angles.</p>
<h2>2 Method and derivation</h2>
<h3>2.1 Frames, units, and dimensions</h3>
<p>The world and base axes are aligned. The Lab 3 model locates the base origin
at (−0.150, 0.150, 0.010) m in the world frame. Figure 1 shows the planar translation.
For a world-frame cup position G<sub>W</sub>, the base-frame coordinates are</p>
<div class="eq">x<sub>g</sub> = x<sub>W</sub> + 0.150,&nbsp;
y<sub>g</sub> = y<sub>W</sub> − 0.150,&nbsp; z<sub>g</sub> = z<sub>W</sub> − 0.010. &nbsp; (1)</div>
<figure><img src="figures/world_base.svg" style="height:1.55in"><figcaption>Figure 1. Original drawing of the world-to-base translation.</figcaption></figure>
<p>All derivation distances are in meters and angles are in radians. The repository's
Lab 3 forward transform expresses translation in millimeters, so it is divided by 1000
before comparison with the target. The nominal dimensions used here are
L<sub>1</sub> = 0.152, L<sub>2</sub> = 0.120, L<sub>3</sub> = 0.244,
L<sub>4</sub> = −0.093 (signed transverse offset), L<sub>5</sub> = 0.213,
L<sub>6</sub> = 0.104, L<sub>7</sub> = 0.085, L<sub>8</sub> = 0.092,
L<sub>9</sub> = 0.0535, and L<sub>10</sub> = 0.059.</p>
</section>
<section class="page">
<h3>2.2 Wrist center, waist, and tool yaw</h3>
<p>Let ψ be the desired world yaw, with ψ = π·yaw<sub>deg</sub>/180.
The plate points from the joint-6 axis toward the cup at this yaw.
Subtract its horizontal length p = L<sub>9</sub> to obtain the virtual wrist center C,
which is at the cup's height:</p>
<div class="eq">x<sub>c</sub> = x<sub>g</sub> − p cos ψ,&nbsp;
y<sub>c</sub> = y<sub>g</sub> − p sin ψ,&nbsp; z<sub>c</sub> = z<sub>g</sub>. &nbsp; (2)</div>
<p>In Figure 2, e<sub>r</sub> = (cos θ<sub>1</sub>, sin θ<sub>1</sub>) and
e<sub>t</sub> = (−sin θ<sub>1</sub>, cos θ<sub>1</sub>).
The net transverse offset is d = L<sub>2</sub> + L<sub>4</sub> + L<sub>6</sub> = 0.131.
Writing C<sub>xy</sub> = (r + b)e<sub>r</sub> + d e<sub>t</sub>, with b = L<sub>7</sub>,
gives ρ = √(x<sub>c</sub>² + y<sub>c</sub>²) and the selected waist solution</p>
<div class="eq">θ<sub>1</sub> = atan2(y<sub>c</sub>, x<sub>c</sub>) − asin(d/ρ). &nbsp; (3)</div>
<p>The branch uses r + b ≥ 0. A center with ρ &lt; d is unreachable.
At θ<sub>6</sub> = 0, the plate points along e<sub>t</sub>. With θ<sub>5</sub> = −π/2,
joint 6 rotates about downward vertical, so ψ = θ<sub>1</sub> + π/2 − θ<sub>6</sub>. Thus</p>
<div class="eq">θ<sub>6</sub> = θ<sub>1</sub> + π/2 − ψ &nbsp; (modulo 2π). &nbsp; (4)</div>
<figure><img src="figures/top_view.svg" style="height:2.4in"><figcaption>Figure 2. Original top view showing the radial, transverse, and plate offsets.</figcaption></figure>
<h3>2.3 Projected forearm endpoint</h3>
<p>Remove the radial wrist link and transverse offset, then raise the virtual point by
v = L<sub>8</sub> + L<sub>10</sub> = 0.151. The endpoint P<sub>3</sub> is</p>
<div class="eq">x<sub>3</sub> = x<sub>c</sub> − b cos θ<sub>1</sub> + d sin θ<sub>1</sub>,<br>
y<sub>3</sub> = y<sub>c</sub> − b sin θ<sub>1</sub> − d cos θ<sub>1</sub>,<br>
z<sub>3</sub> = z<sub>c</sub> + v. &nbsp; (5)</div>
</section>
<section class="page">
<h3>2.4 Shoulder, elbow, and wrist pitch</h3>
<p>Figure 3 reduces the arm to a two-link triangle. Define a = L<sub>3</sub> = 0.244,
c = L<sub>5</sub> = 0.213, signed radial distance r, height h, and triangle length D:</p>
<div class="eq">r = x<sub>3</sub> cos θ<sub>1</sub> + y<sub>3</sub> sin θ<sub>1</sub>,&nbsp;
h = z<sub>3</sub> − L<sub>1</sub>,&nbsp; D = √(r² + h²). &nbsp; (6)</div>
<p>The signed projection is retained even when P<sub>3</sub> lies behind the radial origin.
The triangle exists only if |a − c| ≤ D ≤ a + c, or 0.031 ≤ D ≤ 0.457 m.</p>
<figure><img src="figures/side_view.svg" style="height:2.25in"><figcaption>Figure 3. Original side view of the elbow-up two-link construction.</figcaption></figure>
<p>The elbow deflection follows from D² = a² + c² + 2ac cos θ<sub>3</sub>.
The positive deflection selects the elbow-up solution:</p>
<div class="eq">θ<sub>3</sub> = acos[(D² − a² − c²)/(2ac)]. &nbsp; (7)</div>
<p>The target elevation is β = atan2(h, r). The angle between the upper arm and
the shoulder-to-target ray is α = acos[(a² + D² − c²)/(2aD)].
An equivalent expression used in code avoids inconsistent rounding near straight and folded configurations:</p>
<div class="eq">α = atan2(c sin θ<sub>3</sub>, a + c cos θ<sub>3</sub>),<br>
θ<sub>2</sub> = −β − α. &nbsp; (8)</div>
<p>Positive rotation about the model's shoulder y-axis lowers the arm, hence the negative
sign. The elbow-up branch places the upper arm above the target ray.
Keeping the radial wrist link level requires θ<sub>2</sub> + θ<sub>3</sub> + θ<sub>4</sub> = 0.
The two fixed-orientation constraints therefore yield</p>
<div class="eq">θ<sub>4</sub> = −θ<sub>2</sub> − θ<sub>3</sub>,&nbsp;
θ<sub>5</sub> = −π/2. &nbsp; (9)</div>
<p>The geometric angles are passed to <code>lab_fk()</code>. Its encoder conversion is
q = (θ<sub>1</sub> + π, θ<sub>2</sub>, θ<sub>3</sub>, θ<sub>4</sub> − π/2,
θ<sub>5</sub>, θ<sub>6</sub>). These offsets are applied once, after FK verification.</p>
</section>
<section class="page">
<h2>3 Data and results</h2>
<p>Table 1 lists the experiment targets and numerical IK/FK residuals recomputed from
the saved input coordinates. The user supplied terminal logs for two physical runs
on robot bench 7. Both logs report <code>Requested joint pose reached.</code>
The supplied measurements were recorded in centimeters and converted to meters for Table 2.
P2 uses the same centimeter convention as P1; its unit was not repeated in the original note.</p>
<table><caption>Table 1. Numerical validation; position in meters and yaw in degrees.</caption>
<tr><th>Point</th><th>Desired (x<sub>W</sub>, y<sub>W</sub>, z<sub>W</sub>)</th><th>Yaw</th><th>FK residual (m)</th></tr>
NUMERICAL_ROWS
</table>
<p>The largest listed FK position residual is MAX_ERROR m. The returned orientation
is checked against R<sub>z</sub>(ψ), with fixed roll and pitch in the Lab 3 tool-frame
convention. Seventeen regression tests also cover 200 forward-generated poses,
workspace boundaries, encoder conversion, wrapped yaw, and invalid targets.</p>
<p>The independent ruler/square measurements belong in Table 2. Their scalar error is</p>
<div class="eq">e = √[(x<sub>meas</sub> − x<sub>W</sub>)² +
(y<sub>meas</sub> − y<sub>W</sub>)² + (z<sub>meas</sub> − z<sub>W</sub>)²]. &nbsp; (10)</div>
<table><caption>Table 2. Physical measurements in the world frame.</caption>
<tr><th>Point</th><th>Measured (x, y, z), m</th><th>Scalar error, mm</th></tr>
MEASURED_ROWS
</table>
<p><b>Physical-results discussion:</b> PHYSICAL_DISCUSSION</p>
</section>
<section class="page">
<h3>3.1 Sources of error</h3>
<p>Potential position errors include uncertainty in the base/world origin, ruler
resolution and parallax, nominal link lengths differing from calibrated dimensions,
variation in suction-cup compression, and tool mounting offsets. An origin error
can shift all measured points similarly; geometry or tool errors can vary with pose.
These explanations should be compared with the actual measurements before identifying
a dominant source.</p>
<h2>4 Conclusion</h2>
<p>The analytical solver computes all six geometric angles for the selected elbow-up
branch and applies the Lab 3 encoder offsets. Its numerical checks reproduce the
requested position and yaw, and invalid geometric targets are rejected before motion.
The software has been built with ROS 2 Jazzy and exercised using the dry-run command.</p>
<p>PHYSICAL_CONCLUSION</p>
<h2>5 References</h2>
<p>[1] ECE 470, <i>Lab 4: Inverse Kinematics</i>, supplied Lab4_Manual.pdf,
pp. 40–47.<br>
[2] ECE 470, <i>Lab Report Guidelines</i>,
<a href="https://coecsl.ece.illinois.edu/ece470/Labs/LabReportGuidelines.pdf">course report guidelines</a>.<br>
[3] Popilworck, <i>ece470 repository</i>, Lab 3 forward model, commit 839462c,
<a href="https://github.com/Popilworck/ece470">GitHub repository</a>.
All three figures were generated specifically for this report.</p>
<h2>Appendix A: Implementation and reproduction</h2>
<p><code>lab_func.py</code> contains the reusable <code>lab_ik_angles()</code> solver,
<code>forward_kinematics()</code> transform, and <code>lab_invk()</code> integration.
The FK model and nominal dimensions remain consistent with Lab 3.
The Lab 4 executable accepts world coordinates in meters and yaw in degrees.
Its dry-run path does not create a ROS node.</p>
<pre>cd /home/xliu173/ece470/ece470lab/lab_files
source /opt/ros/jazzy/setup.bash
source .lab4-install/local_setup.bash
ros2 run ece470labs lab4_exec 0.1 0.1 0.15 90 --dry-run</pre>
<p>For physical motion, follow <code>lab4/README.html</code> and omit
<code>--dry-run</code>. Record ruler/square measurements in <code>measurements.csv</code>.
Run <code>python3 lab4/generate_report.py</code> to recompute errors and export the report.
Complete the identifying information and physical-results discussion before submission.</p>
</section></body></html>'''
    document = (document.replace('STATUS', status)
                .replace('PHYSICAL_DISCUSSION', physical_discussion)
                .replace('PHYSICAL_CONCLUSION', physical_conclusion)
                .replace('NUMERICAL_ROWS', '\n'.join(numerical))
                .replace('MEASURED_ROWS', '\n'.join(measured))
                .replace('MAX_ERROR', f'{max_error:.3e}'))
    output = HERE / 'Lab4_Report.html'
    output.write_text(document)
    chrome = shutil.which('google-chrome') or shutil.which('chromium')
    if chrome:
        command = [chrome, '--headless', '--no-sandbox', '--disable-gpu',
                   '--no-pdf-header-footer',
                   '--print-to-pdf='+str(HERE / 'Lab4_Report.pdf'), output.as_uri()]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=45)
        if completed.returncode:
            raise RuntimeError('PDF export failed: '+completed.stderr[-1000:])
    print(f'Report generated; maximum numerical residual = {max_error:.3e} m.')
    print(status)


if __name__ == '__main__':
    report()
