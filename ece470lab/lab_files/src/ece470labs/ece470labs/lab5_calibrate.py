"""Fit and save Appendix B calibration from measured pixel/world pairs."""

import argparse
import csv
from dataclasses import asdict
from pathlib import Path

from .blob_search import fit_calibration
from .lab5_func import load_settings, save_settings


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', required=True, help='CSV: col,row,x_world_m,y_world_m.')
    parser.add_argument('--config', required=True, help='Lab 5 JSON configuration to update.')
    parser.add_argument('--image-size', type=int, nargs=2, metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--max-error-mm', type=float, default=5.0)
    parser.add_argument('--check-only', action='store_true', help='Check pairs without changing the configuration.')
    options = parser.parse_args(args)
    try:
        settings = load_settings(options.config)
        with Path(options.points).open(newline='') as stream:
            pairs = list(csv.DictReader(stream))
        pixels = [[float(row['col']), float(row['row'])] for row in pairs]
        world = [[float(row['x_world_m']), float(row['y_world_m'])] for row in pairs]
        width, height = options.image_size or [settings['camera']['width'], settings['camera']['height']]
        calibration, errors = fit_calibration(
            pixels, world, width, height, settings['vision']['flip_horizontal'], options.max_error_mm/1000)
        print('Measured camera calibration:', asdict(calibration))
        print('Point residuals (mm):', (errors*1000).round(3).tolist())
        if not options.check_only:
            settings['camera'] = asdict(calibration)
            save_settings(options.config, settings)
            report_path = Path(options.points).with_suffix('.validation.csv')
            with report_path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['col', 'row', 'x_world_m', 'y_world_m', 'predicted_x_m',
                                 'predicted_y_m', 'residual_m'])
                for pixel, measured, error in zip(pixels, world, errors):
                    writer.writerow([*pixel, *measured, *calibration.image_to_world(*pixel), error])
            print('Saved:', options.config, 'and', report_path)
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.error(str(error)+' Fill at least three non-collinear pixel/world pairs in meters.')


if __name__ == '__main__':
    main()
